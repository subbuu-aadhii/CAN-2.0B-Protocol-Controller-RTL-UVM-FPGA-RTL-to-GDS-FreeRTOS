"""
check_bitstream.py - cross-check the RTL bus bitstream against the golden model.

For every directed test the testbench logs each bus bit sampled by node2
(03_tb_directed/logs/bits_<test>.log : "<time> <bit> <bsp_state>").
This script:
  1. decodes the stream with the independent golden decoder,
  2. for every correctly received frame re-encodes it with the golden encoder
     and compares the RTL bitstream BIT-EXACTLY (stuffing, CRC, ACK, EOF),
  3. lists error frames (expected only in the error-injection tests).
"""
import glob, os, sys
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from can_golden import decode_stream, encode_frame, self_test

LOGDIR = os.path.join(os.path.dirname(__file__), "..", "03_tb_directed", "logs")
ERROR_TESTS = {"ack_error", "single_shot", "error_inject", "form_error", "bus_off",
               "loopback"}   # loopback: internal self-ACK, bus ACK slot stays recessive

self_test()
report, total_ok, total_mismatch, unexpected_err = [], 0, 0, 0
for path in sorted(glob.glob(os.path.join(LOGDIR, "bits_*.log"))):
    test = os.path.basename(path)[5:-4]
    bits = [int(l.split()[1]) for l in open(path) if l.strip()]
    if not bits:
        continue
    frames = decode_stream(bits)
    ok = [f for f in frames if f.ok]
    bad = [f for f in frames if not f.ok]
    mism = 0
    for f in ok:
        ref, _ = encode_frame(f.id, f.ide, f.rtr, f.dlc, bytes(f.data + [0] * 8))
        rtl = bits[f.start:f.start + len(ref)]
        if rtl != ref:
            mism += 1
    total_ok += len(ok); total_mismatch += mism
    if test not in ERROR_TESTS:
        unexpected_err += len(bad)
    report.append(f"\n=== {test}: {len(bits)} bus bits, {len(ok)} valid frames, "
                  f"{len(bad)} error/aborted frames, bit-exact mismatches vs golden = {mism}")
    for f in frames[:40]:
        report.append("   " + repr(f))
    if len(frames) > 40:
        report.append(f"   ... {len(frames) - 40} more")

summary = (f"\nGOLDEN-MODEL CROSS-CHECK: {total_ok} valid frames compared bit-exactly, "
           f"{total_mismatch} mismatches, {unexpected_err} unexpected error frames -> "
           f"{'PASS' if total_mismatch == 0 and unexpected_err == 0 else 'FAIL'}")
out = "\n".join(report) + "\n" + summary + "\n"
open(os.path.join(os.path.dirname(__file__), "golden_crosscheck_report.txt"), "w").write(out)
print(out[-6000:])
