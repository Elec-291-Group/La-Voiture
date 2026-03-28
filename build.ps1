param(
    [string]$Target    = "build",
    [string]$Port      = "auto",
    [string]$Baud      = "115200",
    [string]$FlashTool = "C:\Users\Owner\Downloads\STM32L051 (1)\stm32flash\stm32flash.exe"
)

$ErrorActionPreference = "Stop"
$IsWindowsOS = [System.Runtime.InteropServices.RuntimeInformation]::IsOSPlatform([System.Runtime.InteropServices.OSPlatform]::Windows)

# --- Project root ---
$ProjectRoot = $PSScriptRoot

function Get-FlashTool {
    param(
        [string]$RequestedTool,
        [bool]$OnWindows,
        [string]$Root
    )

    if (-not [string]::IsNullOrWhiteSpace($RequestedTool)) {
        if (Test-Path $RequestedTool) { return $RequestedTool }
        $cmd = Get-Command $RequestedTool -ErrorAction SilentlyContinue
        if ($cmd) { return $cmd.Source }
    }

    $cmdCandidates = if ($OnWindows) { @("stm32flash.exe", "stm32flash") } else { @("stm32flash") }
    foreach ($name in $cmdCandidates) {
        $cmd = Get-Command $name -ErrorAction SilentlyContinue
        if ($cmd) { return $cmd.Source }
    }

    if ($OnWindows) {
        $candidates = @(
            "D:\stm32flash\stm32flash.exe",
            "C:\stm32flash\stm32flash.exe",
            "$Root\stm32flash.exe",
            "$env:USERPROFILE\Downloads\stm32flash.exe"
        )
    } else {
        $candidates = @(
            "$Root/stm32flash",
            "/opt/homebrew/bin/stm32flash",
            "/usr/local/bin/stm32flash",
            "/usr/bin/stm32flash"
        )
    }

    return $candidates | Where-Object { $_ -and (Test-Path $_) } | Select-Object -First 1
}

function Get-SerialPorts {
    param([bool]$OnWindows)

    if ($OnWindows) {
        return [System.IO.Ports.SerialPort]::GetPortNames() | Sort-Object
    }

    $patterns = @("/dev/cu.*", "/dev/tty.*", "/dev/ttyUSB*", "/dev/ttyACM*")
    $ports = @()
    foreach ($pattern in $patterns) {
        $ports += Get-ChildItem -Path $pattern -Name -ErrorAction SilentlyContinue
    }

    if (-not $ports -or $ports.Count -eq 0) {
        $ports = [System.IO.Ports.SerialPort]::GetPortNames()
    }

    return $ports | Sort-Object -Unique
}

$FlashTool = Get-FlashTool -RequestedTool $FlashTool -OnWindows $IsWindowsOS -Root $ProjectRoot

# --- Detect project name ---
$CmakeLists  = Get-Content "$ProjectRoot\CMakeLists.txt" -Raw
if ($CmakeLists -match 'set\s*\(\s*CMAKE_PROJECT_NAME\s+([\w_]+)') {
    $ProjectName = $Matches[1]
} else {
    throw "Cannot find CMAKE_PROJECT_NAME in CMakeLists.txt"
}

# --- Paths ---
$BuildDir = "$ProjectRoot\build"
$ElfFile  = "$BuildDir\$ProjectName.elf"
$BinFile  = "$BuildDir\$ProjectName.bin"

# --- Fix temp path (Windows only) ---
if ($IsWindowsOS) {
    $env:TMP  = "$env:USERPROFILE\AppData\Local\Temp"
    $env:TEMP = "$env:USERPROFILE\AppData\Local\Temp"
}

# --- Targets ---
switch ($Target) {

    "build" {
        if (-not (Test-Path $BuildDir)) {
            throw "build/ not found. Run cmake first."
        }

        Write-Host "[cmake] Building '$ProjectName'..." -ForegroundColor Cyan

        Push-Location $ProjectRoot
        cmake --build build
        $exitCode = $LASTEXITCODE
        Pop-Location

        if ($exitCode -ne 0) { throw "Build failed" }

        Write-Host "[objcopy] ELF -> BIN" -ForegroundColor Cyan
        arm-none-eabi-objcopy -O binary $ElfFile $BinFile

        if (-not (Test-Path $BinFile)) {
            throw "BIN generation failed"
        }

        Write-Host "[done] $BinFile" -ForegroundColor Green
    }

    "flash" {

        # --- Port check ---
        $availablePorts = Get-SerialPorts -OnWindows $IsWindowsOS

        if ([string]::IsNullOrWhiteSpace($Port) -or $Port -ieq "auto") {
            if ($availablePorts.Count -eq 0) {
                Write-Host "[error] No serial ports detected." -ForegroundColor Red
                exit 1
            }
            $Port = $availablePorts[0]
            Write-Host "[flash] Auto-selected port: $Port" -ForegroundColor Yellow
        }

        if ($Port -notin $availablePorts) {
            Write-Host "[error] Port '$Port' not found." -ForegroundColor Red
            Write-Host "Available serial ports:" -ForegroundColor Yellow

            if ($availablePorts.Count -eq 0) {
                Write-Host "  (none detected)" -ForegroundColor DarkGray
            } else {
                ($availablePorts | Sort-Object) | ForEach-Object {
                    Write-Host "  $_" -ForegroundColor White
                }
            }
            exit 1
        }

        # --- File checks ---
        if (-not (Test-Path $BinFile)) {
            throw "$ProjectName.bin not found - run build first"
        }

        if (-not $FlashTool) {
            throw "stm32flash not found. Install it (e.g. brew install stm32flash) or pass -FlashTool."
        }

        Write-Host "[flash] Using: $FlashTool" -ForegroundColor Cyan
        Write-Host "[flash] Writing to $Port @ $Baud..." -ForegroundColor Cyan

        & $FlashTool -b $Baud -w $BinFile -v -g 0x08000000 $Port

        if ($LASTEXITCODE -ne 0) {
            throw "Flash failed"
        }

        Write-Host "[done] Flash successful 🚀" -ForegroundColor Green
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
        Write-Host "Valid targets: build, flash, clean"
    }
}
