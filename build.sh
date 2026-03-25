#!/usr/bin/env bash
# =============================================================================
# build.sh - Build & flash script for STM32CubeMX CMake projects (macOS/Linux)
# Usage:
#   ./build.sh build                        # make + elf->bin
#   ./build.sh flash [-p /dev/tty.usbserial-*] [-b 115200]
#   ./build.sh clean                        # clean build directory
#
# NOTE: Run cmake once manually before first build:
#   cmake -B build -DCMAKE_TOOLCHAIN_FILE=cmake/gcc-arm-none-eabi.cmake
# =============================================================================

set -e

# --- Defaults ----------------------------------------------------------------
TARGET="${1:-build}"
PORT=""
BAUD="115200"

find_serial_ports() {
    ls /dev/cu.usbserial-* /dev/cu.usbmodem* /dev/cu.SLAB_USBtoUART \
       /dev/tty.usbserial-* /dev/tty.usbmodem* /dev/tty.SLAB_USBtoUART \
       /dev/ttyUSB* /dev/ttyACM* 2>/dev/null || true
}

# --- Parse optional flags ----------------------------------------------------
shift || true
while [[ $# -gt 0 ]]; do
    case "$1" in
        -p|--port) PORT="$2"; shift 2 ;;
        -b|--baud) BAUD="$2"; shift 2 ;;
        *) echo "Unknown argument: $1"; exit 1 ;;
    esac
done

# --- Auto-detect serial port if not specified --------------------------------
if [[ -z "$PORT" ]]; then
    PORT=$(find_serial_ports | head -1)
fi

# If caller passed /dev/tty.* on macOS, prefer the matching /dev/cu.* device.
if [[ "$PORT" == /dev/tty.* ]]; then
    ALT_PORT="/dev/cu.${PORT#/dev/tty.}"
    if [[ -e "$ALT_PORT" ]]; then
        echo "[flash] Switching to preferred callout port: $ALT_PORT"
        PORT="$ALT_PORT"
    fi
fi

# --- Auto-detect stm32flash --------------------------------------------------
FLASH_TOOL=$(command -v stm32flash 2>/dev/null || true)
if [[ -z "$FLASH_TOOL" ]]; then
    echo "[error] stm32flash not found. Install with: brew install stm32flash"
    exit 1
fi

# --- Auto-detect project name from CMakeLists.txt ----------------------------
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_NAME=$(grep -m1 'set\s*(CMAKE_PROJECT_NAME' "$SCRIPT_DIR/CMakeLists.txt" \
    | sed -E 's/.*CMAKE_PROJECT_NAME[[:space:]]+([A-Za-z0-9_]+).*/\1/')

if [[ -z "$PROJECT_NAME" ]]; then
    echo "[error] Cannot find CMAKE_PROJECT_NAME in CMakeLists.txt"
    exit 1
fi

# --- Derived paths -----------------------------------------------------------
BUILD_DIR="$SCRIPT_DIR/build"
ELF_FILE="$BUILD_DIR/$PROJECT_NAME.elf"
BIN_FILE="$BUILD_DIR/$PROJECT_NAME.bin"

# --- Targets -----------------------------------------------------------------
case "$TARGET" in

    build)
        if [[ ! -d "$BUILD_DIR" ]]; then
            echo "[error] build/ not found. Run cmake first:"
            echo "  cmake -B build -DCMAKE_TOOLCHAIN_FILE=cmake/gcc-arm-none-eabi.cmake"
            exit 1
        fi
        echo "[cmake] Building '$PROJECT_NAME'..."
        cmake --build "$BUILD_DIR"

        echo "[objcopy] $PROJECT_NAME.elf -> $PROJECT_NAME.bin"
        arm-none-eabi-objcopy -O binary "$ELF_FILE" "$BIN_FILE"
        echo "[done] $BIN_FILE"
        ;;

    flash)
        if [[ ! -f "$BIN_FILE" ]]; then
            echo "[error] $PROJECT_NAME.bin not found - run build first"
            exit 1
        fi
        if [[ -z "$PORT" ]]; then
            echo "[error] No serial port found. Specify with: ./build.sh flash -p /dev/cu.usbserial-XXXX"
            exit 1
        fi
        if [[ ! -e "$PORT" ]]; then
            echo "[error] Port '$PORT' not found."
            echo "Available ports:"
            AVAILABLE=$(find_serial_ports)
            if [[ -z "$AVAILABLE" ]]; then
                echo "  (none detected)"
            else
                echo "$AVAILABLE" | sed 's/^/  /'
            fi
            exit 1
        fi
        echo "[flash] Writing to $PORT at $BAUD baud..."
        "$FLASH_TOOL" -b "$BAUD" -w "$BIN_FILE" -v -g 0x08000000 "$PORT"
        echo "[done] Flash successful"
        ;;

    clean)
        if [[ -d "$BUILD_DIR" ]]; then
            rm -rf "$BUILD_DIR"
            echo "[done] build directory removed"
        else
            echo "[skip] build directory does not exist"
        fi
        ;;

    *)
        echo "Unknown target: $TARGET"
        echo "Valid targets: build (default), flash, clean"
        exit 1
        ;;
esac
