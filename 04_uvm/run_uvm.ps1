# run_uvm.ps1 - compile / elaborate / run the UVM regression on Vivado xsim
#   powershell -ExecutionPolicy Bypass -File run_uvm.ps1 [-Tests can_dut_tx_test,...] [-NoCompile]
param([string[]]$Tests = @("can_reg_test","can_dut_tx_test","can_dut_rx_test","can_arb_test","can_err_test","can_busoff_test","can_full_test"),
      [switch]$NoCompile)
$ErrorActionPreference = "Continue"
$Tests = @($Tests | ForEach-Object { $_ -split "," } | Where-Object { $_ })
$VIV = "D:\2025.2\Vivado\bin"
Set-Location $PSScriptRoot
New-Item -ItemType Directory -Force logs | Out-Null
$rtl = "..\01_rtl"
if (-not $NoCompile) {
  & "$VIV\xvlog.bat" -sv -L uvm --include tb `
     "$rtl\can_crc15.v" "$rtl\can_btl.v" "$rtl\can_bsp.v" "$rtl\can_acf.v" "$rtl\can_fifo.v" "$rtl\can_regs.v" "$rtl\can_top.v" `
     tb\can_ifs.sv tb\can_uvm_pkg.sv tb\can_sva.sv tb\tb_top.sv -log logs\xvlog.log | Out-Null
  if ($LASTEXITCODE -ne 0) { Get-Content logs\xvlog.log | Select-String "ERROR" ; exit 1 }
  & "$VIV\xelab.bat" tb_top -L uvm -timescale 1ns/1ps -s can_uvm_snap -debug typical `
     -log logs\xelab.log | Out-Null
  if ($LASTEXITCODE -ne 0) { Get-Content logs\xelab.log | Select-String "ERROR" ; exit 1 }
}
$summary = @()
foreach ($t in $Tests) {
  # .bat wrappers split arguments on '=', so pass the options through a file
  $opts = @("-testplusarg UVM_TESTNAME=$t", "-testplusarg UVM_VERBOSITY=UVM_LOW",
            "-sv_seed $(Get-Random -Maximum 99999)", "-cov_db_name $t", "-cov_db_dir ./func_cov_db")
  $opts | Set-Content -Encoding ascii "logs\$t.opts"
  & "$VIV\xsim.bat" can_uvm_snap -runall -f "logs\$t.opts" -log "logs\$t.log" | Out-Null
  $log = (Get-Content "logs\$t.log" -Raw -ErrorAction SilentlyContinue) + ""
  $res = if ($log -match "TEST PASSED") { "PASSED" } else { "FAILED" }
  $sb  = ([regex]::Match($log, "SCOREBOARD:[^\r\n]*")).Value
  $cov = ([regex]::Match($log, "FUNCTIONAL COVERAGE:[^\r\n]*")).Value
  $err = ([regex]::Match($log, "UVM_ERROR :\s*\d+")).Value
  $sva = ([regex]::Matches($log, "Assertion .* failed")).Count
  $line = "{0,-18} {1,-7} {2} | {3} | SVA fails: {4}`r`n                           {5}" -f $t, $res, $err, $sb, $sva, $cov
  $summary += $line; Write-Output $line
}
$summary | Set-Content logs\uvm_summary.txt
# functional coverage report (RTL code coverage: Verilator, see 08_freertos_cosim)
& "$VIV\xcrg.bat" -report_format html -dir .\func_cov_db\xsim.covdb -report_dir .\reports\functional_cov -log logs\xcrg_func.log | Out-Null
Write-Output "reports in 04_uvm\reports"
