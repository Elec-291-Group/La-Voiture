# =============================================================================
# build.ps1 - Build & flash script for STM32CubeMX CMake projects
# Usage:
#   .\build.ps1 build                    # make + elf->bin
#   .\build.ps1 flash -Port COM3         # flash existing .bin
#   .\build.ps1 clean                    # clean build directory
#
# NOTE: Run cmake once manually from Git Bash before first build:
#   cmake -B build -DCMAKE_TOOLCHAIN_FILE=cmake/gcc-arm-none-eabi.cmake
# =============================================================================

param(
    [string]$Target    = "build",
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
$ElfFile  = "$BuildDir\$ProjectName.elf"
$BinFile  = "$BuildDir\$ProjectName.bin"

# --- Fix MinGW linker temp file issue ----------------------------------------
$env:TMP  = "$env:USERPROFILE\AppData\Local\Temp"
$env:TEMP = "$env:USERPROFILE\AppData\Local\Temp"

# --- Targets -----------------------------------------------------------------
switch ($Target) {

    "build" {
        if (-not (Test-Path $BuildDir)) {
            throw "build/ not found. Run cmake first from Git Bash:`n  cmake -B build -DCMAKE_TOOLCHAIN_FILE=cmake/gcc-arm-none-eabi.cmake"
        }
        Write-Host "[cmake] Building '$ProjectName'..." -ForegroundColor Cyan
        Push-Location $ProjectRoot
        cmake --build build
        $exitCode = $LASTEXITCODE
        Pop-Location
        if ($exitCode -ne 0) { throw "Build failed" }

        Write-Host "[objcopy] $ProjectName.elf -> $ProjectName.bin" -ForegroundColor Cyan
        arm-none-eabi-objcopy -O binary $ElfFile $BinFile
        Write-Host "[done] $BinFile" -ForegroundColor Green
    }

    "flash" {
        if (-not $Port) {
            Write-Host "[error] -Port is required for flash." -ForegroundColor Red
            Write-Host "Available COM ports:" -ForegroundColor Yellow
            $ports = [System.IO.Ports.SerialPort]::GetPortNames() | Sort-Object
            if ($ports.Count -eq 0) {
                Write-Host "  (none detected)" -ForegroundColor DarkGray
            } else {
                foreach ($p in $ports) {
                    Write-Host "  $p" -ForegroundColor White
                }
            }
            Write-Host "Usage:" -ForegroundColor Yellow
            Write-Host "  .\build.ps1 flash -Port COM3" -ForegroundColor White
            exit 1
        }
        $availablePorts = [System.IO.Ports.SerialPort]::GetPortNames()
        if ($Port -notin $availablePorts) {
            Write-Host "[error] Port '$Port' not found." -ForegroundColor Red
            Write-Host "Available COM ports:" -ForegroundColor Yellow
            if ($availablePorts.Count -eq 0) {
                Write-Host "  (none detected)" -ForegroundColor DarkGray
            } else {
                ($availablePorts | Sort-Object) | ForEach-Object { Write-Host "  $_" -ForegroundColor White }
            }
            exit 1
        }
        if (-not (Test-Path $BinFile))   { throw "$ProjectName.bin not found - run build first" }
        if (-not (Test-Path $FlashTool)) { throw "stm32flash not found at: $FlashTool" }
        Write-Host "[flash] Writing to $Port at $Baud baud..." -ForegroundColor Cyan
        & $FlashTool -b $Baud -w $BinFile -v -g 0x08000000 $Port
        if ($LASTEXITCODE -ne 0) { throw "Flash failed" }
        Write-Host "[done] Flash successful" -ForegroundColor Green
    }

    "clean" {
        if (Test-Path $BuildDir) {
            Remove-Item -Recurse -Force $BuildDir
            Write-Host "[done] build directory removed" -ForegroundColor Green
        } else {
            Write-Host "[skip] build directory does not exist" -ForegroundColor Yellow
        }
    }

    default {
        Write-Host "Unknown target: $Target"
        Write-Host "Valid targets: build (default), flash, clean"
    }
}