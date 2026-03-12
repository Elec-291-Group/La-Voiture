# Build & Flash

## First-time setup (all platforms)

Run cmake once to configure the build directory:
```bash
mkdir build && cd build && cmake .. -DCMAKE_TOOLCHAIN_FILE=../cmake/gcc-arm-none-eabi.cmake
```

---

## Usage

### Cross-platform (Python)
```bash
python build.py build
python build.py flash -Port COM3          # Windows
python build.py flash -p /dev/tty.usbserial-1410  # macOS
python build.py clean
```

---

## Flashing via UART bootloader

Before flashing, put the MCU into bootloader mode:
same as FPGA 8052 re-boot process
1. long push boot0 button and push the reset button
