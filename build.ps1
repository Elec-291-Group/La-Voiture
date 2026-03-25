# =============================================================================
# build.ps1 - Flash script for STM32 projects
# Usage:
#   .\build.ps1 -Port COM3         # flash existing .bin
# =============================================================================

param(
    [string]$Port      = "",
    [string]$Baud      = "115200",
    [string]$FlashTool = ""
)

# --- Auto-detect stm32flash if not specified ---------------------------------
if (-not $FlashTool) {
    $candidates = @(
        "D:\stm32flash\stm32flash.exe",
        "C:\stm32flash\stm32flash.exe"
    )
    $FlashTool = $candidates | Where-Object { Test-Path $_ } | Select-Object -First 1
}

$ErrorActionPreference = "Stop"

# --- Auto-detect project name from CMakeLists.txt ----------------------------
$ProjectRoot = $PSScriptRoot
$CmakeLists  = Get-Content "$ProjectRoot\CMakeLists.txt" -Raw
if ($CmakeLists -match 'set\s*\(\s*CMAKE_PROJECT_NAME\s+([\w_]+)') {
    $ProjectName = $Matches[1]
} else {
    throw "Cannot find CMAKE_PROJECT_NAME in CMakeLists.txt"
}

# --- Derived paths -----------------------------------------------------------
$BuildDir = "$ProjectRoot\build"
$BinFile  = "$BuildDir\$ProjectName.bin"

# --- Flash -------------------------------------------------------------------
if (-not $Port) {
    Write-Host "[error] -Port is required." -ForegroundColor Red
    Write-Host "Available COM ports:" -ForegroundColor Yellow
    $ports = [System.IO.Ports.SerialPort]::GetPortNames() | Sort-Object
    if ($ports.Count -eq 0) {
        Write-Host "  (none detected)" -ForegroundColor DarkGray
    } else {
        foreach ($p in $ports) { Write-Host "  $p" -ForegroundColor White }
    }
    Write-Host "Usage:" -ForegroundColor Yellow
    Write-Host "  .\build.ps1 -Port COM3" -ForegroundColor White
    exit 1
}

$availablePorts = [System.IO.Ports.SerialPort]::GetPortNames()
if ($Port -notin $availablePorts) {
    Write-Host "[error] Port '$Port' not found." -ForegroundColor Red
    ($availablePorts | Sort-Object) | ForEach-Object { Write-Host "  $_" -ForegroundColor White }
    exit 1
}
if (-not (Test-Path $BinFile))   { throw "$ProjectName.bin not found - run cmake --build build first" }
if (-not (Test-Path $FlashTool)) { throw "stm32flash not found at: $FlashTool" }

Write-Host "[flash] Writing to $Port at $Baud baud..." -ForegroundColor Cyan
& $FlashTool -b $Baud -w $BinFile -v -g 0x08000000 $Port
if ($LASTEXITCODE -ne 0) { throw "Flash failed" }
Write-Host "[done] Flash successful" -ForegroundColor Green
