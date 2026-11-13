// -----------------------------------------------------------------------------
// can_uvm_pkg.sv - UVM 1.2 verification environment for the CAN 2.0B controller
//
//  env
//   +-- apb_agent  (driver / monitor / sequencer)     -> programs the DUT
//   +-- can_agent  (bit-level BFM node driver, ACK responder, error injector,
//   |               protocol monitor = independent CAN decoder)
//   +-- can_scoreboard (reference model: TX buffer shadow, acceptance filter,
//   |                   expected-frame queues in both directions)
//   +-- can_coverage   (frame / error / IRQ / fault-confinement covergroups)
// -----------------------------------------------------------------------------
`timescale 1ns/1ps
package can_uvm_pkg;
    import uvm_pkg::*;
    `include "uvm_macros.svh"

    // register map
    localparam bit [7:0] R_CTRL=8'h00, R_BTR=8'h04, R_CMD=8'h08, R_STATUS=8'h0C, R_ISTAT=8'h10,
                         R_IEN=8'h14, R_ECNT=8'h18, R_ECODE=8'h1C, R_ACR=8'h20, R_AMR=8'h24,
                         R_TXID=8'h30, R_TXDLC=8'h34, R_TXD0=8'h38, R_TXD1=8'h3C,
                         R_RXID=8'h40, R_RXDLC=8'h44, R_RXD0=8'h48, R_RXD1=8'h4C, R_VER=8'h50;
    // 20 MHz clock, 1 Mbit/s: BRP=2clk/tq, 10 tq (SYNC1 + TSEG1 7 + TSEG2 2), SJW 2
    localparam bit [31:0] BTR_1M = (1 << 20) | (1 << 16) | (6 << 12) | 1;

    // ======================================================================
    class can_cfg extends uvm_object;
        `uvm_object_utils(can_cfg)
        bit ack_enable = 1;
        function new(string name = "can_cfg"); super.new(name); endfunction
    endclass

    // ======================================================================
    // CAN frame transaction + golden encoder (CRC-15, bit stuffing)
    // ======================================================================
    class can_frame extends uvm_sequence_item;
        rand bit [28:0] id;
        rand bit        ide;
        rand bit        rtr;
        rand bit [3:0]  dlc;
        rand bit [7:0]  data[8];
        // BFM controls
        rand int        inject_idx;      // flip this on-wire bit (-1 = none)
        // observed / status
        bit from_dut, acked, ok, arb_lost;
        string err;

        constraint c_id     { !ide -> id < 29'h800; }
        constraint c_rtr    { rtr dist {0 := 85, 1 := 15}; }
        constraint c_dlc    { dlc dist {[0:8] :/ 85, [9:15] :/ 15}; }
        constraint c_inject { inject_idx == -1; }

        `uvm_object_utils_begin(can_frame)
            `uvm_field_int(id,  UVM_ALL_ON | UVM_HEX)
            `uvm_field_int(ide, UVM_ALL_ON)
            `uvm_field_int(rtr, UVM_ALL_ON)
            `uvm_field_int(dlc, UVM_ALL_ON)
            `uvm_field_sarray_int(data, UVM_ALL_ON | UVM_HEX)
        `uvm_object_utils_end

        function new(string name = "can_frame"); super.new(name); endfunction

        function int nbytes(); return rtr ? 0 : ((dlc > 8) ? 8 : dlc); endfunction

        function string convert2string();
            string s = $sformatf("%s ID=0x%0h %sDLC=%0d DATA=[", ide ? "EXT" : "STD", id, rtr ? "RTR " : "", dlc);
            for (int i = 0; i < nbytes(); i++) s = {s, $sformatf("%02h%s", data[i], (i < nbytes()-1) ? " " : "")};
            return {s, "]"};
        endfunction

        // payload-aware equality (bytes beyond DLC are don't care)
        function bit same(can_frame o);
            if (o.id != id || o.ide != ide || o.rtr != rtr || o.dlc != dlc) return 0;
            for (int i = 0; i < nbytes(); i++) if (o.data[i] != data[i]) return 0;
            return 1;
        endfunction

        static function bit [14:0] crc15(bit q[$]);
            bit [14:0] c = 0; bit fb;
            foreach (q[i]) begin fb = q[i] ^ c[14]; c = {c[13:0], 1'b0}; if (fb) c ^= 15'h4599; end
            return c;
        endfunction

        // build the destuffed SOF..data bits
        function void raw_bits(ref bit q[$], output int arb_last);
            q.delete();
            q.push_back(0);
            if (!ide) begin
                for (int i = 10; i >= 0; i--) q.push_back(id[i]);
                q.push_back(rtr); arb_last = q.size() - 1; q.push_back(0); q.push_back(0);
            end else begin
                for (int i = 28; i >= 18; i--) q.push_back(id[i]);
                q.push_back(1); q.push_back(1);
                for (int i = 17; i >= 0; i--) q.push_back(id[i]);
                q.push_back(rtr); arb_last = q.size() - 1; q.push_back(0); q.push_back(0);
            end
            for (int i = 3; i >= 0; i--) q.push_back(dlc[i]);
            for (int k = 0; k < nbytes(); k++) for (int i = 7; i >= 0; i--) q.push_back(data[k][i]);
        endfunction

        // complete on-wire sequence SOF..EOF (ACK slot recessive as sent by a transmitter)
        // arb_end = on-wire index of the last arbitration bit, ack_idx = ACK slot index
        function void wire_bits(ref bit w[$], output int arb_end, output int ack_idx);
            bit q[$]; int arb_last; bit [14:0] c; int run; int last;
            raw_bits(q, arb_last);
            c = crc15(q);
            for (int i = 14; i >= 0; i--) q.push_back(c[i]);
            w.delete(); run = 0; last = 2; arb_end = 0;
            foreach (q[i]) begin
                w.push_back(q[i]);
                if (i == arb_last) arb_end = w.size() - 1;
                run = (q[i] == last) ? run + 1 : 1; last = q[i];
                if (run == 5) begin w.push_back(!q[i]); last = !q[i]; run = 1; end
            end
            w.push_back(1);            // CRC delimiter
            ack_idx = w.size();
            w.push_back(1);            // ACK slot (transmitter sends recessive)
            w.push_back(1);            // ACK delimiter
            repeat (7) w.push_back(1); // EOF
        endfunction
    endclass

    // ======================================================================
    // APB agent
    // ======================================================================
    class apb_item extends uvm_sequence_item;
        rand bit [7:0]  addr;
        rand bit [31:0] data;
        rand bit        write;
        bit [31:0]      rdata;
        `uvm_object_utils_begin(apb_item)
            `uvm_field_int(addr, UVM_ALL_ON | UVM_HEX)
            `uvm_field_int(data, UVM_ALL_ON | UVM_HEX)
            `uvm_field_int(write, UVM_ALL_ON)
            `uvm_field_int(rdata, UVM_ALL_ON | UVM_HEX)
        `uvm_object_utils_end
        function new(string name = "apb_item"); super.new(name); endfunction
    endclass

    typedef uvm_sequencer #(apb_item) apb_sequencer;

    class apb_driver extends uvm_driver #(apb_item);
        `uvm_component_utils(apb_driver)
        virtual apb_if vif;
        function new(string name, uvm_component parent); super.new(name, parent); endfunction
        function void build_phase(uvm_phase phase);
            if (!uvm_config_db#(virtual apb_if)::get(this, "", "apb_vif", vif)) `uvm_fatal("NOVIF", "apb_vif")
        endfunction
        task run_phase(uvm_phase phase);
            vif.psel <= 0; vif.penable <= 0; vif.pwrite <= 0; vif.paddr <= 0; vif.pwdata <= 0;
            @(posedge vif.rst_n);
            forever begin
                seq_item_port.get_next_item(req);
                @(posedge vif.clk);
                vif.psel <= 1; vif.penable <= 0; vif.pwrite <= req.write; vif.paddr <= req.addr;
                vif.pwdata <= req.data;
                @(posedge vif.clk);
                vif.penable <= 1;
                @(posedge vif.clk);
                while (!vif.pready) @(posedge vif.clk);
                if (!req.write) req.rdata = vif.prdata;
                vif.psel <= 0; vif.penable <= 0;
                seq_item_port.item_done();
            end
        endtask
    endclass

    class apb_monitor extends uvm_monitor;
        `uvm_component_utils(apb_monitor)
        virtual apb_if vif;
        uvm_analysis_port #(apb_item) ap;
        function new(string name, uvm_component parent); super.new(name, parent); ap = new("ap", this); endfunction
        function void build_phase(uvm_phase phase);
            if (!uvm_config_db#(virtual apb_if)::get(this, "", "apb_vif", vif)) `uvm_fatal("NOVIF", "apb_vif")
        endfunction
        task run_phase(uvm_phase phase);
            apb_item t;
            forever begin
                @(posedge vif.clk);
                if (vif.psel && vif.penable && vif.pready) begin
                    t = apb_item::type_id::create("t");
                    t.addr = vif.paddr; t.write = vif.pwrite;
                    t.data = vif.pwdata; t.rdata = vif.prdata;
                    ap.write(t);
                end
            end
        endtask
    endclass

    class apb_agent extends uvm_agent;
        `uvm_component_utils(apb_agent)
        apb_sequencer sqr; apb_driver drv; apb_monitor mon;
        function new(string name, uvm_component parent); super.new(name, parent); endfunction
        function void build_phase(uvm_phase phase);
            sqr = apb_sequencer::type_id::create("sqr", this);
            drv = apb_driver::type_id::create("drv", this);
            mon = apb_monitor::type_id::create("mon", this);
        endfunction
        function void connect_phase(uvm_phase phase);
            drv.seq_item_port.connect(sqr.seq_item_export);
        endfunction
    endclass

    // ======================================================================
    // CAN agent: BFM node driver
    // ======================================================================
    class can_bfm_item extends uvm_sequence_item;
        typedef enum {TX_FRAME, CORRUPT_DUT, IDLE_BITS} kind_e;
        kind_e     kind = TX_FRAME;
        can_frame  frame;
        int        n_bits = 20;          // IDLE_BITS
        int        max_attempts = 16;
        // results
        int        attempts; bit success; bit lost_arb;
        `uvm_object_utils(can_bfm_item)
        function new(string name = "can_bfm_item"); super.new(name); endfunction
    endclass

    typedef uvm_sequencer #(can_bfm_item) can_sequencer;

    class can_driver extends uvm_driver #(can_bfm_item);
        `uvm_component_utils(can_driver)
        virtual can_if vif;
        can_cfg cfg;
        bit transmitting;
        uvm_analysis_port #(can_frame) sent_ap;   // frames the BFM transmitted successfully
        function new(string name, uvm_component parent); super.new(name, parent); sent_ap = new("sent_ap", this); endfunction
        function void build_phase(uvm_phase phase);
            if (!uvm_config_db#(virtual can_if)::get(this, "", "can_vif", vif)) `uvm_fatal("NOVIF", "can_vif")
            if (!uvm_config_db#(can_cfg)::get(this, "", "cfg", cfg)) cfg = new();
        endfunction

        task wait_bit_start(); @(posedge vif.clk iff vif.bit_start); endtask
        task wait_sample();    @(posedge vif.clk iff vif.sample_pt); endtask

        // one transmission attempt; returns 1 if the whole frame went out ACKed
        task automatic attempt(can_frame f, bit allow_inject, output bit ok, output bit lost);
            bit w[$]; int arb_end, ack_idx, i; bit drv, b;
            f.wire_bits(w, arb_end, ack_idx);
            ok = 0; lost = 0;
            // wait for bus idle (11 recessive bits) or join a SOF started by the DUT
            forever begin
                wait_bit_start();
                if (vif.sof_seen) begin i = 1; break; end
                if (vif.ones >= 11) begin i = 0; break; end
            end
            transmitting = 1;
            for (; i < w.size(); i++) begin
                drv = w[i];
                if (allow_inject && i == f.inject_idx) drv = !drv;
                vif.bfm_tx <= drv;
                wait_sample();
                b = vif.bus;
                if (i == ack_idx) begin
                    if (b !== 1'b0) begin f.err = "no ACK"; break; end
                end else if (drv && !b) begin
                    if (i <= arb_end && !(allow_inject && f.inject_idx >= 0)) begin lost = 1; f.arb_lost = 1; end
                    else f.err = $sformatf("error frame/bit error at wire bit %0d", i);
                    break;
                end
                if (i < w.size() - 1) wait_bit_start();
            end
            ok = (i == w.size()) && !(allow_inject && f.inject_idx >= 0);
            vif.bfm_tx <= 1'b1;
            transmitting = 0;
        endtask

        task automatic do_tx(can_bfm_item it);
            bit ok, lost;
            it.attempts = 0; it.success = 0;
            while (!it.success && it.attempts < it.max_attempts) begin
                attempt(it.frame, it.attempts == 0, ok, lost);
                it.attempts++;
                if (lost) it.lost_arb = 1;
                if (ok) begin
                    it.success = 1;
                    sent_ap.write(it.frame);
                end
            end
            if (!it.success) `uvm_warning("BFM", $sformatf("frame %s not delivered after %0d attempts",
                                                           it.frame.convert2string(), it.attempts))
        endtask

        // drive a dominant pulse on a recessive bit of the DUT's next frame (after the
        // arbitration field) -> DUT detects a bit error
        task automatic do_corrupt(can_bfm_item it);
            int k;
            do wait_sample(); while (!(vif.bus === 1'b0 && vif.dut_tx === 1'b0 && vif.ones >= 10));
            // inside a DUT frame: skip it.n_bits bits (beyond the arbitration field for
            // a guaranteed bit error), then hit the next recessive DUT bit
            repeat (it.n_bits) wait_bit_start();
            for (k = 0; k < 60; k++) begin
                wait_bit_start();
                repeat (3) @(posedge vif.clk);
                if (vif.dut_tx === 1'b1 && vif.bus === 1'b1) begin
                    vif.inj <= 1'b1;
                    wait_bit_start();
                    vif.inj <= 1'b0;
                    break;
                end
            end
        endtask

        task ack_responder();
            forever begin
                wait_bit_start();
                if (vif.ack_req) begin
                    vif.ack_req = 0;
                    if (!transmitting && cfg.ack_enable) begin
                        vif.bfm_tx <= 1'b0;
                        wait_bit_start();
                        vif.bfm_tx <= 1'b1;
                    end
                end
            end
        endtask

        task run_phase(uvm_phase phase);
            vif.bfm_tx <= 1'b1; vif.inj <= 1'b0;
            @(posedge vif.rst_n);
            fork ack_responder(); join_none
            forever begin
                seq_item_port.get_next_item(req);
                case (req.kind)
                    can_bfm_item::TX_FRAME:    do_tx(req);
                    can_bfm_item::CORRUPT_DUT: do_corrupt(req);
                    default: repeat (req.n_bits) wait_bit_start();
                endcase
                seq_item_port.item_done();
            end
        endtask
    endclass

    // ======================================================================
    // CAN protocol monitor: independent decoder of the bus
    // ======================================================================
    class can_monitor extends uvm_monitor;
        `uvm_component_utils(can_monitor)
        virtual can_if vif;
        uvm_analysis_port #(can_frame) ap;
        int run; bit last; bit dut_mismatch;
        int n_ok, n_err;
        function new(string name, uvm_component parent); super.new(name, parent); ap = new("ap", this); endfunction
        function void build_phase(uvm_phase phase);
            if (!uvm_config_db#(virtual can_if)::get(this, "", "can_vif", vif)) `uvm_fatal("NOVIF", "can_vif")
        endfunction

        task automatic raw(output bit b);
            @(posedge vif.clk iff vif.sample_pt);
            b = vif.bus;
            if (vif.dut_tx !== b) dut_mismatch = 1;
        endtask

        // next destuffed bit; returns err=1 on stuff violation
        task automatic dbit(output bit b, output bit err);
            err = 0;
            raw(b);
            if (run == 5) begin
                if (b == last) begin err = 1; return; end
                run = 1; last = b;
                raw(b);
            end
            run = (b == last) ? run + 1 : 1; last = b;
        endtask

        task run_phase(uvm_phase phase);
            can_frame f; bit b, e; bit q[$]; bit [14:0] crc_rx; int nb; bit [28:0] base; bit [17:0] ext;
            @(posedge vif.rst_n);
            forever begin
                // ---- SOF after bus idle
                do @(posedge vif.clk iff vif.sample_pt); while (!(vif.bus === 1'b0 && vif.ones >= 10));
                f = can_frame::type_id::create("obs");
                dut_mismatch = (vif.dut_tx !== 1'b0);
                run = 1; last = 0; q.delete(); q.push_back(0); e = 0;
                begin : decode
                    base = 0;
                    for (int i = 0; i < 11; i++) begin dbit(b, e); if (e) disable decode; base = {base[9:0], b}; q.push_back(b); end
                    dbit(b, e); if (e) disable decode; q.push_back(b); f.rtr = b;          // RTR or SRR
                    dbit(b, e); if (e) disable decode; q.push_back(b); f.ide = b;
                    if (f.ide) begin
                        ext = 0;
                        for (int i = 0; i < 18; i++) begin dbit(b, e); if (e) disable decode; ext = {ext[16:0], b}; q.push_back(b); end
                        dbit(b, e); if (e) disable decode; q.push_back(b); f.rtr = b;
                        dbit(b, e); if (e) disable decode; q.push_back(b);                   // r1
                        f.id = {base, ext};
                    end else f.id = base;
                    dbit(b, e); if (e) disable decode; q.push_back(b);                       // r0
                    for (int i = 0; i < 4; i++) begin dbit(b, e); if (e) disable decode; f.dlc = {f.dlc[2:0], b}; q.push_back(b); end
                    nb = f.nbytes();
                    for (int k = 0; k < nb; k++) for (int i = 0; i < 8; i++) begin
                        dbit(b, e); if (e) disable decode; f.data[k] = {f.data[k][6:0], b}; q.push_back(b);
                    end
                    for (int i = 0; i < 15; i++) begin dbit(b, e); if (e) disable decode; crc_rx = {crc_rx[13:0], b}; end
                    if (run == 5) begin raw(b); if (b == last) begin e = 1; disable decode; end end
                    f.from_dut = !dut_mismatch;
                    raw(b);                                             // CRC delimiter
                    if (!b) begin f.err = "form: CRC delimiter"; disable decode; end
                    if (crc_rx != can_frame::crc15(q)) begin f.err = "CRC mismatch"; disable decode; end
                    vif.ack_req = 1;                                    // ask BFM to ACK
                    raw(b); f.acked = !b; vif.ack_req = 0;              // ACK slot
                    if (!f.acked) begin f.err = "ACK error"; disable decode; end
                    raw(b); if (!b) begin f.err = "form: ACK delimiter"; disable decode; end
                    for (int i = 0; i < 7; i++) begin
                        raw(b);
                        if (!b && i < 6) begin f.err = $sformatf("form: EOF bit %0d", i); disable decode; end
                    end
                    f.ok = 1;
                end
                if (e) f.err = "stuff error";
                if (f.ok) n_ok++; else n_err++;
                `uvm_info("MON", $sformatf("%s %s %s", f.ok ? "FRAME" : "ERROR", f.from_dut ? "DUT->bus" : "BFM->bus",
                          f.ok ? f.convert2string() : f.err), UVM_MEDIUM)
                ap.write(f);
            end
        endtask
    endclass

    class can_agent extends uvm_agent;
        `uvm_component_utils(can_agent)
        can_sequencer sqr; can_driver drv; can_monitor mon;
        function new(string name, uvm_component parent); super.new(name, parent); endfunction
        function void build_phase(uvm_phase phase);
            sqr = can_sequencer::type_id::create("sqr", this);
            drv = can_driver::type_id::create("drv", this);
            mon = can_monitor::type_id::create("mon", this);
        endfunction
        function void connect_phase(uvm_phase phase);
            drv.seq_item_port.connect(sqr.seq_item_export);
        endfunction
    endclass

    // ======================================================================
    // Scoreboard + reference model
    // ======================================================================
    `uvm_analysis_imp_decl(_apb)
    `uvm_analysis_imp_decl(_can)

    class can_scoreboard extends uvm_scoreboard;
        `uvm_component_utils(can_scoreboard)
        uvm_analysis_imp_apb #(apb_item,  can_scoreboard) apb_imp;
        uvm_analysis_imp_can #(can_frame, can_scoreboard) can_imp;

        // shadow registers (reference model state)
        bit        reset_mode = 1;
        bit [30:0] acr = 0, amr = '1;
        bit [31:0] txid, txdlc, txd0, txd1;
        bit [31:0] rx_id_r, rx_dlc_r, rx_d0_r, rx_d1_r;
        can_frame  exp_dut_tx[$];     // frames the DUT must put on the bus (in order)
        can_frame  exp_dut_rx[$];     // frames the DUT must store in its RX FIFO (in order)
        int n_tx_match, n_rx_match, n_mismatch, n_filtered;

        function new(string name, uvm_component parent);
            super.new(name, parent);
            apb_imp = new("apb_imp", this); can_imp = new("can_imp", this);
        endfunction

        function bit acf_pass(can_frame f);
            bit [30:0] key = {f.ide, f.rtr, f.ide ? f.id : {18'd0, f.id[10:0]}};
            return ((key ^ acr) & ~amr) == 0;
        endfunction

        function can_frame frame_from_regs(bit [31:0] idr, bit [31:0] dlcr, bit [31:0] d0, bit [31:0] d1);
            can_frame f = can_frame::type_id::create("ref");
            f.id = idr[28:0]; f.rtr = idr[29]; f.ide = idr[30]; f.dlc = dlcr[3:0];
            for (int i = 0; i < 4; i++) begin f.data[i] = d0[i*8 +: 8]; f.data[i+4] = d1[i*8 +: 8]; end
            if (!f.ide) f.id = {18'd0, f.id[10:0]};
            return f;
        endfunction

        function void write_apb(apb_item t);
            if (t.write) begin
                case (t.addr)
                    R_CTRL:  reset_mode = t.data[0];
                    R_ACR:   if (reset_mode) acr = t.data[30:0];
                    R_AMR:   if (reset_mode) amr = t.data[30:0];
                    R_TXID:  txid  = t.data;
                    R_TXDLC: txdlc = t.data;
                    R_TXD0:  txd0  = t.data;
                    R_TXD1:  txd1  = t.data;
                    R_CMD: begin
                        if (t.data[0] && !reset_mode) begin
                            exp_dut_tx.push_back(frame_from_regs(txid, txdlc, txd0, txd1));
                            `uvm_info("SB", $sformatf("expect DUT TX: %s", exp_dut_tx[$].convert2string()), UVM_HIGH)
                        end
                        if (t.data[2]) check_dut_rx();
                    end
                endcase
            end else begin
                case (t.addr)
                    R_RXID:  rx_id_r  = t.rdata;
                    R_RXDLC: rx_dlc_r = t.rdata;
                    R_RXD0:  rx_d0_r  = t.rdata;
                    R_RXD1:  rx_d1_r  = t.rdata;
                endcase
            end
        endfunction

        function void check_dut_rx();
            can_frame got = frame_from_regs(rx_id_r, rx_dlc_r, rx_d0_r, rx_d1_r), exp;
            if (exp_dut_rx.size() == 0) begin
                n_mismatch++;
                `uvm_error("SB", $sformatf("DUT RX FIFO returned unexpected frame %s", got.convert2string()))
                return;
            end
            exp = exp_dut_rx.pop_front();
            if (exp.same(got)) begin
                n_rx_match++;
                `uvm_info("SB", $sformatf("RX MATCH  %s", got.convert2string()), UVM_MEDIUM)
            end else begin
                n_mismatch++;
                `uvm_error("SB", $sformatf("RX MISMATCH exp %s got %s", exp.convert2string(), got.convert2string()))
            end
        endfunction

        function void write_can(can_frame f);
            can_frame exp;
            if (!f.ok) return;                                  // error frames are retried
            if (f.from_dut) begin
                if (exp_dut_tx.size() == 0) begin
                    n_mismatch++;
                    `uvm_error("SB", $sformatf("unexpected DUT frame on bus %s", f.convert2string()))
                    return;
                end
                exp = exp_dut_tx.pop_front();
                if (exp.same(f)) begin
                    n_tx_match++;
                    `uvm_info("SB", $sformatf("TX MATCH  %s", f.convert2string()), UVM_MEDIUM)
                end else begin
                    n_mismatch++;
                    `uvm_error("SB", $sformatf("TX MISMATCH exp %s got %s", exp.convert2string(), f.convert2string()))
                end
            end else begin
                if (reset_mode) return;
                if (acf_pass(f)) exp_dut_rx.push_back(f);
                else n_filtered++;
            end
        endfunction

        function void check_phase(uvm_phase phase);
            if (exp_dut_tx.size()) `uvm_error("SB", $sformatf("%0d DUT TX frame(s) never seen on the bus", exp_dut_tx.size()))
            if (exp_dut_rx.size()) `uvm_error("SB", $sformatf("%0d frame(s) never read from DUT RX FIFO", exp_dut_rx.size()))
        endfunction

        function void report_phase(uvm_phase phase);
            `uvm_info("SB", $sformatf("SCOREBOARD: DUT-TX matched=%0d  DUT-RX matched=%0d  filtered-out=%0d  mismatches=%0d",
                      n_tx_match, n_rx_match, n_filtered, n_mismatch), UVM_NONE)
        endfunction
    endclass

    // ======================================================================
    // Functional coverage
    // ======================================================================
    class can_coverage extends uvm_component;
        `uvm_component_utils(can_coverage)
        uvm_analysis_imp_apb #(apb_item,  can_coverage) apb_imp;
        uvm_analysis_imp_can #(can_frame, can_coverage) can_imp;
        can_frame f; apb_item a;
        bit [2:0] etype; bit edir; bit has_err; bit [9:0] irqs; bit [8:0] tec; bit [7:0] rec; bit [3:0] state;

        covergroup cg_frame;
            option.per_instance = 1;
            cp_dir: coverpoint f.from_dut { bins dut_tx = {1}; bins dut_rx = {0}; }
            cp_ide: coverpoint f.ide { bins std = {0}; bins ext = {1}; }
            cp_rtr: coverpoint f.rtr { bins data = {0}; bins remote = {1}; }
            cp_dlc: coverpoint f.dlc { bins d[] = {[0:8]}; bins over8 = {[9:15]}; }
            cp_id_std: coverpoint f.id[10:0] iff (!f.ide) { bins lo = {[0:'h0FF]}; bins mid = {['h100:'h6FF]}; bins hi = {['h700:'h7FF]}; }
            cp_id_ext: coverpoint f.id[28:18] iff (f.ide) { bins lo = {[0:'h0FF]}; bins mid = {['h100:'h6FF]}; bins hi = {['h700:'h7FF]}; }
            x_dir_ide_rtr: cross cp_dir, cp_ide, cp_rtr;
            x_ide_dlc:     cross cp_ide, cp_dlc;
        endgroup

        covergroup cg_bus_err;
            option.per_instance = 1;
            cp_err: coverpoint has_err { bins error_frame = {1}; bins clean = {0}; }
        endgroup

        covergroup cg_ecode;
            option.per_instance = 1;
            cp_type: coverpoint etype { bins bit_e = {1}; bins stuff = {2}; bins crc = {3}; bins form = {4}; bins ack = {5}; }
            cp_dir:  coverpoint edir  { bins rx = {0}; bins tx = {1}; }
            x_type_dir: cross cp_type, cp_dir {
                // Architecture-unreachable combinations (documented in 00_docs):
                //  * ACK error is a transmitter-only error.
                //  * A receiver only drives the ACK slot, which is not bit-monitored -> no RX bit error.
                //  * The transmitter monitors every bit it sends, so any corruption of its own frame is
                //    reported as BIT error before a stuff/CRC/form check can fire.
                ignore_bins rx_ack   = binsof(cp_type.ack)   && binsof(cp_dir.rx);
                ignore_bins rx_bit   = binsof(cp_type.bit_e) && binsof(cp_dir.rx);
                ignore_bins tx_other = binsof(cp_type) intersect {2, 3, 4} && binsof(cp_dir.tx);
            }
        endgroup

        covergroup cg_irq;
            option.per_instance = 1;
            cp_rx:      coverpoint irqs[0] { bins hit = {1}; }
            cp_txdone:  coverpoint irqs[1] { bins hit = {1}; }
            cp_warn:    coverpoint irqs[2] { bins hit = {1}; }
            cp_passive: coverpoint irqs[3] { bins hit = {1}; }
            cp_busoff:  coverpoint irqs[4] { bins hit = {1}; }
            cp_arblost: coverpoint irqs[5] { bins hit = {1}; }
            cp_buserr:  coverpoint irqs[6] { bins hit = {1}; }
            cp_recover: coverpoint irqs[9] { bins hit = {1}; }
        endgroup

        covergroup cg_fault;
            option.per_instance = 1;
            cp_state: coverpoint state { bins active = {0}; bins warning = {1}; bins passive = {2}; bins bus_off = {3}; }
            cp_tec: coverpoint tec { bins zero = {0}; bins low = {[1:95]}; bins warn = {[96:127]}; bins pas = {[128:255]}; bins off = {[256:511]}; }
        endgroup

        function new(string name, uvm_component parent);
            super.new(name, parent);
            apb_imp = new("apb_imp", this); can_imp = new("can_imp", this);
            cg_frame = new(); cg_bus_err = new(); cg_ecode = new(); cg_irq = new(); cg_fault = new();
        endfunction

        function void write_can(can_frame t);
            f = t;
            has_err = !t.ok;
            cg_bus_err.sample();
            if (t.ok) cg_frame.sample();
        endfunction

        function void write_apb(apb_item t);
            if (t.write) return;
            case (t.addr)
                R_ECODE:  if (t.rdata[2:0] != 0) begin etype = t.rdata[2:0]; edir = t.rdata[3]; cg_ecode.sample(); end
                R_ISTAT:  begin irqs = t.rdata[9:0]; cg_irq.sample(); end
                R_STATUS: begin
                    state = t.rdata[8] ? 3 : t.rdata[7] ? 2 : t.rdata[6] ? 1 : 0;
                    cg_fault.sample();
                end
                R_ECNT:   begin tec = t.rdata[8:0]; cg_fault.sample(); end
            endcase
        endfunction

        function void report_phase(uvm_phase phase);
            `uvm_info("COV", $sformatf("FUNCTIONAL COVERAGE: frame=%0.1f%%  ecode=%0.1f%%  irq=%0.1f%%  fault=%0.1f%%  bus_err=%0.1f%%",
                      cg_frame.get_inst_coverage(), cg_ecode.get_inst_coverage(), cg_irq.get_inst_coverage(),
                      cg_fault.get_inst_coverage(), cg_bus_err.get_inst_coverage()), UVM_NONE)
        endfunction
    endclass

    // ======================================================================
    // Environment
    // ======================================================================
    class can_env extends uvm_env;
        `uvm_component_utils(can_env)
        apb_agent apb_agt; can_agent can_agt; can_scoreboard sb; can_coverage cov;
        function new(string name, uvm_component parent); super.new(name, parent); endfunction
        function void build_phase(uvm_phase phase);
            apb_agt = apb_agent::type_id::create("apb_agt", this);
            can_agt = can_agent::type_id::create("can_agt", this);
            sb      = can_scoreboard::type_id::create("sb", this);
            cov     = can_coverage::type_id::create("cov", this);
        endfunction
        function void connect_phase(uvm_phase phase);
            apb_agt.mon.ap.connect(sb.apb_imp);
            apb_agt.mon.ap.connect(cov.apb_imp);
            can_agt.mon.ap.connect(sb.can_imp);
            can_agt.mon.ap.connect(cov.can_imp);
        endfunction
    endclass

    // ======================================================================
    // Sequences
    // ======================================================================
    class apb_rw_seq extends uvm_sequence #(apb_item);
        `uvm_object_utils(apb_rw_seq)
        bit [7:0] addr; bit [31:0] data; bit write; bit [31:0] rdata;
        function new(string name = "apb_rw_seq"); super.new(name); endfunction
        task body();
            req = apb_item::type_id::create("req");
            start_item(req);
            req.addr = addr; req.data = data; req.write = write;
            finish_item(req);
            rdata = req.rdata;
        endtask
    endclass

    // register-level API used by the tests (a light-weight "RAL")
    class can_reg_api extends uvm_object;
        `uvm_object_utils(can_reg_api)
        apb_sequencer sqr;
        function new(string name = "can_reg_api"); super.new(name); endfunction
        task wr(bit [7:0] a, bit [31:0] d);
            apb_rw_seq s = apb_rw_seq::type_id::create("wr");
            s.addr = a; s.data = d; s.write = 1; s.start(sqr);
        endtask
        task rd(bit [7:0] a, output bit [31:0] d);
            apb_rw_seq s = apb_rw_seq::type_id::create("rd");
            s.addr = a; s.write = 0; s.start(sqr); d = s.rdata;
        endtask
        task configure(bit [31:0] ctrl = 0, bit [30:0] acr = 0, bit [30:0] amr = '1);
            wr(R_CTRL, 1); wr(R_BTR, BTR_1M); wr(R_ACR, acr); wr(R_AMR, amr);
            wr(R_IEN, 32'h3FF); wr(R_ISTAT, 32'h3FF); wr(R_CTRL, ctrl & ~32'h1);
        endtask
        task send(can_frame f);
            bit [31:0] d0, d1, st;
            do rd(R_STATUS, st); while (!st[2]);          // wait TX buffer free
            for (int i = 0; i < 4; i++) begin d0[i*8 +: 8] = f.data[i]; d1[i*8 +: 8] = f.data[i+4]; end
            wr(R_TXID, {1'b0, f.ide, f.rtr, f.id}); wr(R_TXDLC, f.dlc); wr(R_TXD0, d0); wr(R_TXD1, d1);
            wr(R_CMD, 1);
        endtask
        task wait_tx_free();
            bit [31:0] st;
            do rd(R_STATUS, st); while (!st[2]);
        endtask
        // read every frame currently in the RX FIFO
        task drain_rx(output int n);
            bit [31:0] st, d;
            n = 0;
            rd(R_STATUS, st);
            while (st[0]) begin
                rd(R_RXID, d); rd(R_RXDLC, d); rd(R_RXD0, d); rd(R_RXD1, d);
                wr(R_CMD, 4); n++;
                rd(R_STATUS, st);
            end
        endtask
        task poll_status_regs();
            bit [31:0] d;
            rd(R_ISTAT, d); wr(R_ISTAT, d); rd(R_ECODE, d); rd(R_ECNT, d); rd(R_STATUS, d);
        endtask
    endclass

    class can_bfm_tx_seq extends uvm_sequence #(can_bfm_item);
        `uvm_object_utils(can_bfm_tx_seq)
        can_frame frame; int inject = -1;
        bit success;
        function new(string name = "can_bfm_tx_seq"); super.new(name); endfunction
        task body();
            can_bfm_item it = can_bfm_item::type_id::create("it");
            if (frame == null) begin
                frame = can_frame::type_id::create("f");
                if (!frame.randomize()) `uvm_fatal("RAND", "frame")
            end
            frame.inject_idx = inject;
            it.kind = can_bfm_item::TX_FRAME; it.frame = frame;
            start_item(it); finish_item(it);
            success = it.success;
        endtask
    endclass

    class can_bfm_simple_seq extends uvm_sequence #(can_bfm_item);
        `uvm_object_utils(can_bfm_simple_seq)
        can_bfm_item::kind_e kind; int n_bits = 20;
        function new(string name = "can_bfm_simple_seq"); super.new(name); endfunction
        task body();
            can_bfm_item it = can_bfm_item::type_id::create("it");
            it.kind = kind; it.n_bits = n_bits;
            start_item(it); finish_item(it);
        endtask
    endclass
endpackage
