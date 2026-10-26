#!/usr/bin/env python3
"""Prepare and build Turnip (Mesa freedreno Vulkan with UniXNU's Darwin kernel backend) for the 3a's Adreno 810.

  prepare-asteroids-turnip.py --unixnu PATH [--fetch-only]

Runs on macOS with Xcode's iPhoneOS SDK. Everything goes under the UniXNU tree's ignored .cache/ and build/:
  .cache/mesa-work   Mesa at the commit scripts/data/adreno-compiler-inputs.json pins, sparse, with the recorded patches applied
                     in their recorded order (the build script checks that sequence before it builds)
  .cache/mesa-venv   meson, mako, PyYAML, packaging and ninja for the build
  .cache/mesa-tools  bison at the pinned version and checksum (macOS ships 2.3; Mesa needs 3)
  .cache/shader-tools glslangValidator for Mesa's Vulkan runtime (shader_tools.py)
Then UniXNU's scripts/build-turnip-darwin.py --target ios builds build/mesa-tu-ios/src/freedreno/vulkan/libvulkan_freedreno.dylib
against this repository's NativeAdreno ABI (drivers/Adreno740/NativeAdrenoABI.h), the one the 3a's kernel serves.

The pinned Mesa already lists the Adreno 810 (freedreno_devices.py: chip id 0xffff44010000, a8xx_gen1, one CCU, one slice), so the
chip id the kernel reports (0x44010000, GPB_A810_CHIP_ID) selects it through Mesa's wildcard fuse-id match; nothing here changes Mesa
beyond the recorded patches. Fetches are from the pinned sources only; checksums are checked before anything is unpacked."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
import tarfile

import shader_tools

DEVICE = Path(__file__).resolve().parents[1]
parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
parser.add_argument('--unixnu', type=Path, required=True, help='the UniXNU tree (rootstockos/UniXNU asteroids-dev)')
parser.add_argument('--fetch-only', action='store_true', help='prepare the source and tools, do not build')
args = parser.parse_args()
ROOT = args.unixnu.resolve()
CACHE = ROOT / '.cache'
pins = json.loads((ROOT / 'scripts/data/adreno-compiler-inputs.json').read_text())
mesa = pins['mesa']
SOURCE = CACHE / 'mesa-work'
VENV = CACHE / 'mesa-venv'
TOOLS = CACHE / 'mesa-tools'


def run(command, **kw):
    print('+', ' '.join(str(c) for c in command), flush=True)
    return subprocess.run([str(c) for c in command], check=True, **kw)


def git(*command, **kw):
    return run(['git', '-C', SOURCE, *command], **kw)


def head():
    p = subprocess.run(['git', '-C', str(SOURCE), 'rev-parse', 'HEAD'], capture_output=True, text=True)
    return p.stdout.strip() if p.returncode == 0 else None


# 1. Mesa at the pin, sparse. A tree that is already at the pin with the patches applied is kept (the build script's own check
#    rejects anything else); a tree at another commit is replaced.
CACHE.mkdir(exist_ok=True)
if head() != mesa['commit']:
    shutil.rmtree(SOURCE, ignore_errors=True)
    SOURCE.mkdir(parents=True)
    git('init', '-q')
    git('remote', 'add', 'origin', mesa['repository'])
    git('sparse-checkout', 'set', '--cone', *mesa['sparse_directories'])
    git('fetch', '-q', '--depth', '1', '--filter=blob:none', 'origin', mesa['commit'])
    git('checkout', '-q', '--detach', 'FETCH_HEAD')
    if head() != mesa['commit']:
        sys.exit('Mesa checkout is not at the pinned commit')
    tree = subprocess.check_output(['git', '-C', str(SOURCE), 'rev-parse', 'HEAD^{tree}'], text=True).strip()
    if tree != mesa['tree']:
        sys.exit('Mesa tree differs from the pinned tree: ' + tree)
    for patch in pins['patches']:
        git('apply', '--whitespace=nowarn', ROOT / patch)
    print('Mesa', mesa['commit'][:12], 'with', len(pins['patches']), 'recorded patches', flush=True)
else:
    print('Mesa already at', mesa['commit'][:12], flush=True)

# 2. The build's Python tools.
if not (VENV / 'bin/meson').exists():
    run([sys.executable, '-m', 'venv', VENV])
    run([VENV / 'bin/pip', 'install', '-q', 'meson', 'mako', 'PyYAML', 'packaging', 'ninja'])
os.environ['PATH'] = str(VENV / 'bin') + ':' + os.environ['PATH']

# 3. bison at the pinned version, from the pinned tarball and checksum.
bison = pins['bison']
if not (TOOLS / 'bin/bison').exists():
    work = CACHE / 'bison-build'
    shutil.rmtree(work, ignore_errors=True)
    work.mkdir()
    archive = work / Path(bison['url']).name
    # curl, not urllib: it uses the system's trust store (python.org's Python has none until its certificate script runs).
    run(['curl', '-sfL', '--max-filesize', str(bison['bytes']), '-o', archive, bison['url']])
    data = archive.read_bytes()
    if len(data) != bison['bytes'] or hashlib.sha256(data).hexdigest() != bison['sha256']:
        sys.exit('bison archive differs from its pin')
    with tarfile.open(archive) as tar:
        tar.extractall(work, filter='data')
    source = work / ('bison-' + bison['version'])
    run(['./configure', '--prefix=' + str(TOOLS), '--disable-nls'], cwd=source, stdout=subprocess.DEVNULL)
    run(['make', '-j6'], cwd=source, stdout=subprocess.DEVNULL)
    run(['make', 'install'], cwd=source, stdout=subprocess.DEVNULL)
    shutil.rmtree(work)
version = subprocess.check_output([str(TOOLS / 'bin/bison'), '--version'], text=True).splitlines()[0]
if not version.endswith(' ' + bison['version']):
    sys.exit('bison is not the pinned version: ' + version)
if args.fetch_only:
    print('PASS: Mesa, its Python tools and', version, 'ready')
    sys.exit(0)

# 4. Turnip for iOS against this board's ABI. Mesa's Vulkan runtime compiles its internal shaders with glslangValidator.
shader_tools.ensure(ROOT)
run([sys.executable, ROOT / 'scripts/build-turnip-darwin.py', '--target', 'ios', '--abi-dir', DEVICE / 'drivers/Adreno740'], cwd=ROOT)
