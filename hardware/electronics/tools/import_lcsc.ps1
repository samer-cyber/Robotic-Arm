<#
    import_lcsc.ps1 — one-command LCSC -> KiCad global library import.

    The lcsc library now uses easyeda2kicad's NATIVE layout, all in one folder:

        C:\Users\hadad\Documents\KiCad\9.0\lcsc\
            lcsc.kicad_sym
            lcsc.pretty\
            lcsc.3dshapes\

    Because that is exactly what easyeda2kicad's --output base path produces,
    nothing has to be moved. This script only:
      1. runs the tool with the right base path
      2. rewrites the hard-coded absolute 3D paths to ${LCSC_3DSHAPES}
      3. guarantees every .kicad_mod is written as UTF-8 with NO BOM

    (3) is not cosmetic. A UTF-8 BOM at the start of a .kicad_mod makes KiCad
    silently drop that footprint from the library — it just doesn't appear,
    with no error. PowerShell's `Set-Content -Encoding utf8` writes a BOM on
    Windows PowerShell 5.1, which is how three footprints went missing on
    2026-08-01. Always use [System.IO.File]::WriteAllText with UTF8Encoding($false).

    Usage:
        .\import_lcsc.ps1 C160355
        .\import_lcsc.ps1 C160355 C157929 C2934560

    Close KiCad before running; reopen afterwards.
#>

param(
    [Parameter(Mandatory = $true, ValueFromRemainingArguments = $true)]
    [string[]]$LcscIds
)

$ErrorActionPreference = 'Stop'

# --- config ------------------------------------------------------------
$LibRoot = "C:\Users\hadad\Documents\KiCad\9.0\lcsc"
$LibNick = "lcsc"
$Exe     = "C:\Python314\Scripts\easyeda2kicad.exe"
# -----------------------------------------------------------------------

$base   = Join-Path $LibRoot $LibNick     # <root>\lcsc  -> tool appends the suffixes
$pretty = "$base.pretty"
$utf8NoBom = New-Object System.Text.UTF8Encoding $false

if (-not (Test-Path $LibRoot)) { New-Item -ItemType Directory -Force -Path $LibRoot | Out-Null }

Write-Host "Importing: $($LcscIds -join ', ')" -ForegroundColor Cyan
# easyeda2kicad writes informational WARNINGs to stderr. Under PowerShell 5.1 with
# ErrorActionPreference=Stop those get promoted to terminating errors, so relax it
# around the native call and judge success by the exit code instead.
$prev = $ErrorActionPreference
$ErrorActionPreference = 'Continue'
& $Exe --full --overwrite --lcsc_id @LcscIds --output $base
$rc = $LASTEXITCODE
$ErrorActionPreference = $prev
if ($rc -ne 0) { throw "easyeda2kicad failed (exit $rc)" }

# --- normalise every footprint: ${LCSC_3DSHAPES} path + no BOM ----------
$fixedPath = 0; $fixedBom = 0
Get-ChildItem $pretty -Filter *.kicad_mod -File | ForEach-Object {
    $bytes = [System.IO.File]::ReadAllBytes($_.FullName)
    $hadBom = ($bytes.Length -ge 3 -and $bytes[0] -eq 0xEF -and $bytes[1] -eq 0xBB -and $bytes[2] -eq 0xBF)
    $text = if ($hadBom) { [System.Text.Encoding]::UTF8.GetString($bytes, 3, $bytes.Length - 3) }
            else         { [System.Text.Encoding]::UTF8.GetString($bytes) }

    $new = [regex]::Replace(
        $text,
        '\(model\s+"[^"]*[/\\]([^"/\\]+\.(?:wrl|step))"',
        '(model "${' + 'LCSC_3DSHAPES}/$1"')

    if ($hadBom -or $new -ne $text) {
        [System.IO.File]::WriteAllText($_.FullName, $new, $utf8NoBom)
        if ($hadBom)        { $fixedBom++ }
        if ($new -ne $text) { $fixedPath++ }
    }
}

# --- verify: no BOM survived -------------------------------------------
$bad = Get-ChildItem $pretty -Filter *.kicad_mod -File | Where-Object {
    $b = [System.IO.File]::ReadAllBytes($_.FullName)
    $b.Length -ge 3 -and $b[0] -eq 0xEF -and $b[1] -eq 0xBB -and $b[2] -eq 0xBF
}

Write-Host ""
Write-Host "Library: $LibRoot" -ForegroundColor Green
Write-Host "  footprints in library : $((Get-ChildItem $pretty -Filter *.kicad_mod).Count)"
Write-Host "  3D paths rewritten    : $fixedPath"
Write-Host "  BOMs stripped         : $fixedBom"
if ($bad) { Write-Host "  !! BOM STILL PRESENT: $($bad.Name -join ', ')" -ForegroundColor Red }
else      { Write-Host "  BOM check             : clean" -ForegroundColor Green }
Write-Host ""
Write-Host "Restart KiCad to pick up the new library entries." -ForegroundColor Yellow
