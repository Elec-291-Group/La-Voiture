#!/usr/bin/env python3
"""
Cross-platform build entry point.
Automatically calls build.ps1 on Windows or build.sh on macOS/Linux.

Usage:
  python build.py build
  python build.py flash -Port COM3        (Windows)
  python build.py flash -p /dev/tty.xxx  (macOS/Linux)
  python build.py clean
"""

import sys
import os
import subprocess
import platform

script_dir = os.path.dirname(os.path.abspath(__file__))
args = sys.argv[1:]

if platform.system() == "Windows":
    cmd = ["powershell", "-ExecutionPolicy", "Bypass", "-File",
           os.path.join(script_dir, "build.ps1")] + args
else:
    cmd = [os.path.join(script_dir, "build.sh")] + args

result = subprocess.run(cmd)
sys.exit(result.returncode)
