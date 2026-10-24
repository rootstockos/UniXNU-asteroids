#!/usr/bin/env python3
"""Build and run the GPU power board host test (package/GpuPowerBoardTests.c, stages 1-2). Host only, no device."""
import subprocess
from pathlib import Path
ROOT = Path(__file__).resolve().parents[1]
out = ROOT / 'build/gpu-power'
out.mkdir(parents=True, exist_ok=True)
binary = out / 'board-test'
subprocess.run(['clang', '-std=c11', '-O1', '-Wall', '-Wextra', '-Werror', '-I', str(ROOT / 'package'),
                str(ROOT / 'package/GpuPowerBoardTests.c'), '-o', str(binary)], check=True)
subprocess.run([str(binary)], check=True)
