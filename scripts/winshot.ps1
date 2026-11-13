# winshot.ps1 - bring a window (matched by title regex) to front, maximize it and
# save a screenshot of it.  Usage: winshot.ps1 -Title 'GTKWave' -Out file.png [-Wait 8]
param([string]$Title, [string]$Out, [int]$Wait = 6, [switch]$NoMax)
Add-Type -AssemblyName System.Windows.Forms, System.Drawing
Add-Type @'
using System; using System.Text; using System.Runtime.InteropServices; using System.Collections.Generic;
public class WinApi {
  public delegate bool EnumProc(IntPtr h, IntPtr l);
  [DllImport("user32.dll")] public static extern bool EnumWindows(EnumProc cb, IntPtr l);
  [DllImport("user32.dll")] public static extern int GetWindowText(IntPtr h, StringBuilder s, int n);
  [DllImport("user32.dll")] public static extern bool IsWindowVisible(IntPtr h);
  [DllImport("user32.dll")] public static extern bool SetForegroundWindow(IntPtr h);
  [DllImport("user32.dll")] public static extern bool ShowWindow(IntPtr h, int c);
  [DllImport("user32.dll")] public static extern bool GetWindowRect(IntPtr h, out RECT r);
  [DllImport("user32.dll")] public static extern bool SetProcessDPIAware();
  [DllImport("user32.dll")] public static extern void keybd_event(byte k, byte s, int f, int e);
  [StructLayout(LayoutKind.Sequential)] public struct RECT { public int L, T, R, B; }
  public static List<KeyValuePair<IntPtr,string>> All() {
    var res = new List<KeyValuePair<IntPtr,string>>();
    EnumWindows((h, l) => { if (IsWindowVisible(h)) { var sb = new StringBuilder(512); GetWindowText(h, sb, 512);
      if (sb.Length > 0) res.Add(new KeyValuePair<IntPtr,string>(h, sb.ToString())); } return true; }, IntPtr.Zero);
    return res; }
}
'@
[WinApi]::SetProcessDPIAware() | Out-Null
Start-Sleep $Wait
$w = [WinApi]::All() | Where-Object { $_.Value -match $Title } | Select-Object -First 1
if (-not $w) { "NOT FOUND. Visible windows:"; [WinApi]::All() | ForEach-Object { "  " + $_.Value }; exit 1 }
$h = $w.Key
if (-not $NoMax) { [WinApi]::ShowWindow($h, 3) | Out-Null }
[WinApi]::keybd_event(0x12, 0, 0, 0); [WinApi]::keybd_event(0x12, 0, 2, 0)   # ALT tap allows SetForegroundWindow
[WinApi]::SetForegroundWindow($h) | Out-Null
Start-Sleep 3
$r = New-Object WinApi+RECT; [WinApi]::GetWindowRect($h, [ref]$r) | Out-Null
$scr = [System.Windows.Forms.Screen]::PrimaryScreen.Bounds
$L = [Math]::Max($r.L, 0); $T = [Math]::Max($r.T, 0); $R = [Math]::Min($r.R, $scr.Width); $B = [Math]::Min($r.B, $scr.Height)
if (($R - $L) -lt 200 -or ($B - $T) -lt 200) { $L = 0; $T = 0; $R = $scr.Width; $B = $scr.Height }
$bmp = New-Object System.Drawing.Bitmap ($R - $L), ($B - $T)
$g = [System.Drawing.Graphics]::FromImage($bmp)
$g.CopyFromScreen($L, $T, 0, 0, $bmp.Size)
$bmp.Save($Out)
"saved '$($w.Value)' $($R-$L)x$($B-$T) -> $Out"
