#!/bin/bash
# Re-run selected tests with VCD dumping for GTKWave / waveform plots
cd "$(dirname "$0")"
for t in std_frame arbitration error_inject; do
  vvp -n sim.vvp +TEST=$t +VCD > waves/$t.log 2>&1
  mv -f wave_$t.vcd waves/; rm -f bits_$t.log
  echo "$t: $(grep -o 'TEST PASSED\|TEST FAILED' waves/$t.log) $(ls -la waves/wave_$t.vcd | awk '{print $5}') bytes"
done
