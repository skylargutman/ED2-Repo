# Exports the FAU lab PCB submission package with the required file names.
# Usage: powershell -File tools\export_pcb_upload.ps1 [-Version 2] [-Term 26f]
param(
  [string]$Term = '26f',
  [string]$Class = 'EGN4952C',
  [string]$Group = 'G06',
  [string]$Project = 'Pendulum',
  [string]$Version = '2',
  [string]$Pcb = "$PSScriptRoot\..\KiCad\PendulumV2\26S_EGN4952C_G6_PENDULUM_V2.kicad_pcb"
)
$ErrorActionPreference = 'Stop'
$cli = 'C:\Program Files\KiCad\9.0\bin\kicad-cli.exe'
$Pcb = (Resolve-Path $Pcb).Path
$src = Split-Path $Pcb
$sch = [IO.Path]::ChangeExtension($Pcb, '.kicad_sch')
$stem = [IO.Path]::GetFileNameWithoutExtension($Pcb)
$base = "${Term}_${Class}_${Group}_${Project}"
$v = "_V$Version"
$out = Join-Path $src "Upload_${Term}$v"
$raw = Join-Path $out '_raw'
if (Test-Path $out) { Remove-Item $out -Recurse -Force }
New-Item -ItemType Directory -Force $raw | Out-Null

& $cli pcb export gerbers --layers 'F.Cu,B.Cu,Edge.Cuts,F.Silkscreen,B.Silkscreen' --output "$raw\" $Pcb | Out-Null
& $cli pcb export drill --format excellon --excellon-units in --excellon-separate-th --output "$raw\" $Pcb | Out-Null
& $cli sch export pdf --output "$out\$base$v.pdf" $sch | Out-Null
& $cli pcb drc --severity-error --output "$out\${base}-DRC_report$v.txt" $Pcb | Out-Null

$map = @{
  "$stem-B_Cu.gbl"          = "$base-B_Cu$v.gbl"
  "$stem-F_Cu.gtl"          = "$base-F_Cu$v.gtl"
  "$stem-PTH.drl"           = "$base-pth$v.drl"
  "$stem-NPTH.drl"          = "$base-npth$v.drl"
  "$stem-Edge_Cuts.gm1"     = "$base-edge_cuts$v.GKO"
  "$stem-F_Silkscreen.gto"  = "$base-F_Silkscreen$v.gbr"
  "$stem-B_Silkscreen.gbo"  = "$base-B_Silkscreen$v.gbr"
}
foreach ($k in $map.Keys) {
  if (Test-Path "$raw\$k") { Copy-Item "$raw\$k" "$out\$($map[$k])" } else { Write-Warning "missing export: $k" }
}
# 1:1 bottom-layer print for the paper-fit photo (print at 100% / "Actual size").
# Mirrored so it shows the board as seen from the bottom; holes plotted at actual size for pushing parts through.
& $cli pcb export pdf --mode-single --layers 'B.Cu,Edge.Cuts' --mirror --black-and-white --drill-shape-opt 2 `
  --output "$out\$base-Paper_Fit_Print_PRINT_AT_100pct$v.pdf" $Pcb | Out-Null

# Completion statement to print, sign, and photograph (exact wording required by the lab).
$names = @('Victor Borden','Peter Awonusonu','Dylan Florencio','Christopher Vega','Alex Carillo')
$rows = ($names | ForEach-Object { "<tr><td>$_</td><td></td><td></td></tr>" }) -join "`n"
$html = @"
<!doctype html><html><head><meta charset="utf-8"><title>Completion Statement</title>
<style>
 @page { size: letter; margin: 0.9in; }
 body { font-family: Arial, Helvetica, sans-serif; font-size: 13pt; color: #000; }
 h1 { font-size: 16pt; margin: 0 0 4pt; } .sub { font-size: 10.5pt; margin-bottom: 26pt; }
 .stmt p { margin: 0 0 9pt; font-size: 14pt; }
 table { width: 100%; border-collapse: collapse; margin-top: 30pt; }
 th { text-align: left; font-size: 11pt; border-bottom: 1.5px solid #000; padding: 4pt; }
 td { height: 40pt; border-bottom: 1px solid #000; padding: 4pt; vertical-align: bottom; }
 td:nth-child(1) { width: 34%; } td:nth-child(2) { width: 22%; }
</style></head><body>
<h1>PCB Completion Statement</h1>
<div class="sub">$Class &middot; Group $Group &middot; $Project &middot; Version $Version</div>
<div class="stmt">
<p>I have reviewed the Schematic, Netlist, Parts Fitment, &amp; Layout</p>
<p>This is my original work</p>
<p>I ran the DRC in KiCad</p>
<p>I read ALL the pages in the README PCB doc.</p>
</div>
<table><tr><th>Name</th><th>Date</th><th>Signature</th></tr>
$rows
</table></body></html>
"@
$htmlPath = Join-Path $env:TEMP "completion_statement_$Version.html"
Set-Content -Path $htmlPath -Value $html -Encoding utf8
$edge = 'C:\Program Files (x86)\Microsoft\Edge\Application\msedge.exe'
Start-Process -FilePath $edge -Wait -WindowStyle Hidden -RedirectStandardError (Join-Path $env:TEMP 'edge_pdf.log') -ArgumentList @(
  '--headless', '--disable-gpu', '--no-pdf-header-footer', '--print-to-pdf-no-header',
  "`"--print-to-pdf=$out\$base-Completion_Statement_TO_SIGN$v.pdf`"", "file:///$($htmlPath -replace '\\','/')")
Remove-Item $htmlPath -ErrorAction SilentlyContinue

Copy-Item $Pcb "$out\$base$v.kicad_pcb"
Copy-Item $sch "$out\$base$v.kicad_sch"
Remove-Item $raw -Recurse -Force

Get-ChildItem $out | Select-Object Name, Length | Format-Table -AutoSize
Select-String "$out\${base}-DRC_report$v.txt" -Pattern '^\*\* Found' | ForEach-Object { $_.Line }
