# Build & Flash

## Windows PATH requirements

Add the following to your Windows Environment Variables → `Path`:

| Tool | Typical path | Purpose |
|---|---|---|
| ARM GCC toolchain | `C:\Program Files (x86)\Arm GNU Toolchain\...\bin` | `arm-none-eabi-gcc`, `objcopy`, etc. |
| CMake | `C:\Program Files\CMake\bin` | `cmake` command |
| Ninja | `C:\msys64\usr\bin` or standalone install | Build system (`-G Ninja`) |
| Python | `C:\Users\<you>\AppData\Local\Programs\Python\Python3x` | `python build.py` |
| stm32flash | `D:\stm32flash` or `C:\stm32flash` | Flashing via UART |

> To verify each tool is on PATH, run in PowerShell:
> ```powershell
> arm-none-eabi-gcc --version
> cmake --version
> ninja --version
> python --version
> ```

---

## First-time configure

Run once to set up the build directory:
```powershell
cmake -B build -G Ninja
```

---

## Build

```powershell
cmake --build build
```

This also automatically converts the output `.elf` to `.bin`.

---

## Flash

Before flashing, put the MCU into bootloader mode (same as FPGA 8052 re-boot process):
1. Long push BOOT0 button and push the reset button

```powershell
python build.py flash -Port COM3          # Windows
python build.py flash -p /dev/tty.xxx     # macOS/Linux  (use ls /dev/tty.* to find port)
```

---

## Clean

```powershell
Remove-Item -Recurse -Force build
```
