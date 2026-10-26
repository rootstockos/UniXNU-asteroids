#!/usr/bin/env python3
"""Build and run the GPU power board host test (package/GpuPowerBoardTests.c, stages 1-2). Host only, no device."""
import subprocess
from pathlib import Path
ROOT = Path(__file__).resolve().parents[1]
out = ROOT / 'build/gpu-power'
out.mkdir(parents=True, exist_ok=True)
binary = out / 'board-test'
# With the generated GMU header present, mode 8's offsets are also checked against it.
sheng = ROOT / 'build/sheng-gmu/ShengGmuSource.h'
extra = ['-DGPB_HAVE_SHENG_SOURCE', '-I', str(sheng.parent)] if sheng.exists() else []
subprocess.run(['clang', '-std=c11', '-O1', '-Wall', '-Wextra', '-Werror', '-I', str(ROOT / 'package'), '-I', str(ROOT / 'drivers/Adreno740'),
                *extra,
                str(ROOT / 'package/GpuPowerBoardTests.c'), '-o', str(binary)], check=True)
print('mode 8 offsets checked against ' + str(sheng) if extra else 'mode 8 offsets: no generated ShengGmuSource.h here')
subprocess.run([str(binary)], check=True)
