# SPDX-License-Identifier: MIT
"""glslang and SPIRV-Tools for the 3a's GPU builds, from the Khronos sources at one Vulkan SDK tag, built once into the UniXNU
tree's ignored .cache/shader-tools (cmake and ninja from PyPI in a venv there). Mesa's Vulkan runtime needs glslangValidator;
build-asteroids-gpu-user.py compiles and validates the Turnip smoke shaders with glslang and spirv-val."""
import os
from pathlib import Path
import subprocess
import sys

TAG = 'vulkan-sdk-1.4.357.0'


def run(command, **kw):
    print('+', ' '.join(str(c) for c in command), flush=True)
    return subprocess.run([str(c) for c in command], check=True, **kw)


def ensure(root):
    """Builds the tools when missing, puts their bin directory first on PATH and returns it."""
    tools = Path(root) / '.cache/shader-tools'
    prefix = tools / 'prefix'
    if not all((prefix / 'bin' / name).exists() for name in ('glslang', 'glslangValidator', 'spirv-val')):
        tools.mkdir(parents=True, exist_ok=True)
        if not (tools / 'venv/bin/cmake').exists():
            run([sys.executable, '-m', 'venv', tools / 'venv'])
            run([tools / 'venv/bin/pip', 'install', '-q', 'cmake', 'ninja'])
        os.environ['PATH'] = str(tools / 'venv/bin') + ':' + os.environ['PATH']
        for name in ('SPIRV-Headers', 'SPIRV-Tools', 'glslang'):
            if not (tools / name).is_dir():
                run(['git', 'clone', '-q', '--depth', '1', '--branch', TAG, 'https://github.com/KhronosGroup/' + name, tools / name])
        for source, build, extra in (('SPIRV-Headers', 'build-headers', []),
                                     ('SPIRV-Tools', 'build-tools', ['-DSPIRV-Headers_SOURCE_DIR=' + str(tools / 'SPIRV-Headers'),
                                                                     '-DSPIRV_SKIP_TESTS=ON', '-DSPIRV_WERROR=OFF']),
                                     ('glslang', 'build-glslang', ['-DALLOW_EXTERNAL_SPIRV_TOOLS=ON', '-DCMAKE_PREFIX_PATH=' + str(prefix),
                                                                   '-DGLSLANG_TESTS=OFF'])):
            run(['cmake', '-S', tools / source, '-B', tools / build, '-G', 'Ninja', '-DCMAKE_BUILD_TYPE=Release',
                 '-DCMAKE_INSTALL_PREFIX=' + str(prefix), *extra], stdout=subprocess.DEVNULL)
            run(['cmake', '--build', tools / build], stdout=subprocess.DEVNULL)
            run(['cmake', '--install', tools / build], stdout=subprocess.DEVNULL)
    os.environ['PATH'] = str(prefix / 'bin') + ':' + os.environ['PATH']
    return prefix / 'bin'
