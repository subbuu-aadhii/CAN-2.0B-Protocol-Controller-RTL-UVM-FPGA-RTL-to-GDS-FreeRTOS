// -----------------------------------------------------------------------------
// can_tests.sv - UVM tests
//   can_reg_test      : register reset values, access rules
//   can_dut_tx_test   : random frames DUT -> bus (BFM ACKs, monitor + scoreboard check)
//   can_dut_rx_test   : random frames BFM -> DUT with random acceptance filter
//   can_arb_test      : DUT and BFM request simultaneously -> arbitration
//   can_err_test      : BFM corrupts its own frames (stuff/CRC/form errors at DUT),
//                       injector corrupts DUT frames (bit errors), missing ACK
//   can_busoff_test   : repeated DUT bit errors -> bus-off -> recovery
//   can_full_test     : everything above in one run (coverage closure run)
// -----------------------------------------------------------------------------
import uvm_pkg::*;
`include "uvm_macros.svh"
import can_uvm_pkg::*;

class can_base_test extends uvm_test;
    `uvm_component_utils(can_base_test)
    can_env env; can_cfg cfg; can_reg_api regs;
    virtual can_if cvif;
    function new(string name, uvm_component parent); super.new(name, parent); endfunction
    function void build_phase(uvm_phase phase);
        cfg = can_cfg::type_id::create("cfg");
        uvm_config_db#(can_cfg)::set(this, "*", "cfg", cfg);
        env = can_env::type_id::create("env", this);
        if (!uvm_config_db#(virtual can_if)::get(this, "", "can_vif", cvif)) `uvm_fatal("NOVIF", "can_vif")
    endfunction
    function void end_of_elaboration_phase(uvm_phase phase);
        regs = can_reg_api::type_id::create("regs");
        regs.sqr = env.apb_agt.sqr;
        uvm_top.print_topology();
    endfunction

    task bfm_send(can_frame f, int inject = -1, output bit ok);
        can_bfm_tx_seq s = can_bfm_tx_seq::type_id::create("bfm_tx");
        s.frame = f; s.inject = inject; s.start(env.can_agt.sqr); ok = s.success;
    endtask
    task bfm_idle(int nbits);
        can_bfm_simple_seq s = can_bfm_simple_seq::type_id::create("idle");
        s.kind = can_bfm_item::IDLE_BITS; s.n_bits = nbits; s.start(env.can_agt.sqr);
    endtask
    task bfm_corrupt_dut(int skip_bits = -1);
        can_bfm_simple_seq s = can_bfm_simple_seq::type_id::create("corrupt");
        s.kind = can_bfm_item::CORRUPT_DUT;
        s.n_bits = (skip_bits < 0) ? 16 + $urandom % 20 : skip_bits;
        s.start(env.can_agt.sqr);
    endtask
    function can_frame rand_frame(bit force_std = 0);
        can_frame f = can_frame::type_id::create("f");
        if (!f.randomize() with { force_std -> !ide; }) `uvm_fatal("RAND", "frame")
        return f;
    endfunction

    // drain RX FIFO until 'n' frames read or timeout (bits)
    task drain_until_idle(int quiet_bits = 200);
        int n;
        repeat (quiet_bits / 20) begin
            regs.drain_rx(n);
            bfm_idle(20);
        end
        regs.drain_rx(n);
    endtask

    task run_phase(uvm_phase phase);
        phase.raise_objection(this);
        @(posedge cvif.rst_n);
        regs.configure();
        bfm_idle(15);
        body();
        bfm_idle(60);
        drain_until_idle(100);
        regs.poll_status_regs();
        phase.drop_objection(this);
    endtask
    virtual task body(); endtask

    // ---------------- reusable scenarios ----------------
    task scn_dut_tx(int n = 40);
        repeat (n) begin
            regs.send(rand_frame());
            regs.wait_tx_free();
        end
    endtask

    task scn_dut_rx(int n = 40);
        bit ok; int cnt;
        // pass-all phase
        repeat (n / 2) begin
            bfm_send(rand_frame(), -1, ok);
            regs.drain_rx(cnt);
        end
        // acceptance filter: standard IDs 0x2X0..0x2XF accepted (mask low 8 bits), data frames only
        regs.configure(0, {1'b0, 1'b0, 18'd0, 11'h200}, 31'h0000_00FF);
        bfm_idle(15);
        repeat (n / 2) begin
            can_frame f = rand_frame();
            if ($urandom % 2) begin f.ide = 0; f.id = 29'h200 | ($urandom % 256); end
            bfm_send(f, -1, ok);
            regs.drain_rx(cnt);
        end
    endtask

    task scn_arb();
        bit ok; int cnt;
        for (int r = 0; r < 24; r++) begin
            can_frame fd = rand_frame(), fb = rand_frame();
            // make the arbitration interesting: same base ID in a quarter of the rounds
            if (r % 4 == 0) begin fb.id[28:18] = fd.id[28:18]; fb.id[10:0] = fd.id[10:0]; fb.ide = !fd.ide; end
            if (fd.same(fb)) fb.id ^= 1;
            fork
                regs.send(fd);
                bfm_send(fb, -1, ok);
            join
            regs.wait_tx_free();
            regs.drain_rx(cnt);
            regs.poll_status_regs();
        end
    endtask

    task scn_err();
        bit ok; int cnt; bit [31:0] d;
        // 1. BFM corrupts one bit of its own frame (first attempt), then retransmits correctly
        for (int r = 0; r < 20; r++) begin
            can_frame f = rand_frame();
            bit w[$]; int ae, ai;
            f.wire_bits(w, ae, ai);
            bfm_send(f, 1 + ($urandom % (ai - 1)), ok);    // anywhere SOF+1 .. CRC delimiter
            regs.drain_rx(cnt);
            regs.poll_status_regs();
        end
        // 2. injector corrupts DUT frames -> DUT bit error -> retransmission
        for (int r = 0; r < 10; r++) begin
            fork
                regs.send(rand_frame());
                bfm_corrupt_dut();
            join
            regs.wait_tx_free();
            regs.poll_status_regs();
        end
        // 3. missing ACK -> DUT ACK errors until the BFM acknowledges again
        cfg.ack_enable = 0;
        regs.send(rand_frame());
        bfm_idle(400);
        regs.poll_status_regs();
        cfg.ack_enable = 1;
        regs.wait_tx_free();
        regs.rd(R_ECNT, d);
        `uvm_info("ERR", $sformatf("after error test TEC=%0d REC=%0d", d[8:0], d[23:16]), UVM_LOW)
    endtask

    task scn_busoff();
        bit [31:0] d; int k;
        can_frame f = rand_frame();
        f.rtr = 0; f.dlc = 8;
        fork regs.send(f); join
        for (k = 0; k < 40; k++) begin
            regs.rd(R_ECNT, d);         // sample TEC as it climbs (coverage)
            regs.rd(R_STATUS, d);
            if (d[8]) break;
            bfm_corrupt_dut(45);        // past the arbitration field even for 29-bit IDs
        end
        regs.poll_status_regs();
        regs.rd(R_STATUS, d);
        if (!d[8]) `uvm_error("BUSOFF", "DUT did not enter bus-off")
        else `uvm_info("BUSOFF", $sformatf("DUT bus-off after %0d corrupted frames", k), UVM_LOW)
        bfm_idle(128 * 11 + 50);        // recovery: 128 x 11 recessive bits
        regs.poll_status_regs();
        regs.rd(R_STATUS, d);
        if (d[8]) `uvm_error("BUSOFF", "no bus-off recovery")
        regs.wait_tx_free();            // pending frame goes out after recovery
        regs.rd(R_ECNT, d);
        `uvm_info("BUSOFF", $sformatf("after recovery TEC=%0d REC=%0d", d[8:0], d[23:16]), UVM_LOW)
    endtask


    function void report_phase(uvm_phase phase);
        uvm_report_server rs = uvm_report_server::get_server();
        int errs = rs.get_severity_count(UVM_ERROR) + rs.get_severity_count(UVM_FATAL);
        `uvm_info("RESULT", $sformatf("\n*** %s : %s (UVM_ERROR+FATAL = %0d) ***", get_type_name(),
                  errs ? "TEST FAILED" : "TEST PASSED", errs), UVM_NONE)
    endfunction
endclass

// -----------------------------------------------------------------------------
class can_reg_test extends can_base_test;
    `uvm_component_utils(can_reg_test)
    function new(string name, uvm_component parent); super.new(name, parent); endfunction
    task body();
        bit [31:0] d;
        regs.rd(R_VER, d);  if (d != 32'hCA20_B100) `uvm_error("REG", $sformatf("VERSION %h", d))
        regs.rd(R_BTR, d);  if (d != BTR_1M) `uvm_error("REG", $sformatf("BTR %h", d))
        regs.wr(R_BTR, 0);  regs.rd(R_BTR, d); if (d != BTR_1M) `uvm_error("REG", "BTR not protected in operating mode")
        regs.rd(R_STATUS, d); if (d[2:0] != 3'b100) `uvm_error("REG", $sformatf("STATUS %h", d))
        regs.wr(R_TXID, 32'h7FFF_FFFF); regs.rd(R_TXID, d); if (d != 32'h7FFF_FFFF) `uvm_error("REG", "TXID")
        regs.wr(R_IEN, '1); regs.rd(R_IEN, d); if (d != 32'h3FF) `uvm_error("REG", "IEN width")
        `uvm_info("REG", "register checks done", UVM_LOW)
    endtask
endclass

// -----------------------------------------------------------------------------
class can_dut_tx_test extends can_base_test;
    `uvm_component_utils(can_dut_tx_test)
    function new(string name, uvm_component parent); super.new(name, parent); endfunction
    task body(); scn_dut_tx(40); endtask
endclass

// -----------------------------------------------------------------------------
class can_dut_rx_test extends can_base_test;
    `uvm_component_utils(can_dut_rx_test)
    function new(string name, uvm_component parent); super.new(name, parent); endfunction
    task body(); scn_dut_rx(40); endtask
endclass

// -----------------------------------------------------------------------------
class can_arb_test extends can_base_test;
    `uvm_component_utils(can_arb_test)
    function new(string name, uvm_component parent); super.new(name, parent); endfunction
    task body(); scn_arb(); endtask
endclass

// -----------------------------------------------------------------------------
class can_err_test extends can_base_test;
    `uvm_component_utils(can_err_test)
    function new(string name, uvm_component parent); super.new(name, parent); endfunction
    task body(); scn_err(); endtask
endclass

// -----------------------------------------------------------------------------
class can_busoff_test extends can_base_test;
    `uvm_component_utils(can_busoff_test)
    function new(string name, uvm_component parent); super.new(name, parent); endfunction
    task body(); scn_busoff(); endtask
endclass

// -----------------------------------------------------------------------------
class can_full_test extends can_base_test;
    `uvm_component_utils(can_full_test)
    function new(string name, uvm_component parent); super.new(name, parent); endfunction
    task body();
        scn_dut_tx(30);
        scn_dut_rx(30);
        regs.configure(); bfm_idle(15);
        scn_arb();
        scn_err();
        scn_busoff();
    endtask
endclass
