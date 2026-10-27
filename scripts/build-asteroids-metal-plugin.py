#!/usr/bin/env python3
"""Build the 3a's Metal plugin, test-only (T17) or published (T18, --published), into a GPU addon and an owner trust bundle for the boot payload.

  build-asteroids-metal-plugin.py --unixnu PATH --trust-cache ORIGINAL.bundle [--published]

Runs on macOS with Xcode's iPhoneOS SDK, after scripts/prepare-asteroids-turnip.py (Turnip for iOS against this repository's ABI).
--published also needs UniXNU's metal2vulkan bridge for iOS (scripts/build-adreno-metal-bridge.py --target ios, build/adreno-metal-bridge/ios).
Output in build/asteroids-metal-plugin, or build/asteroids-metal-plugin-published (ignored):
  NativeTurnipShaders.h      the Turnip smoke shaders (UniXNU drivers/AdrenoMetal/shaders) as SPIR-V words: glslang, then spirv-val
  NativeAdrenoMetal.bundle   package/AsteroidsMetalPlugin.mm (principal class NativeAdrenoMetalDevice) with Turnip beside it, each signed
                             ad hoc with 4 KiB pages
  asteroids-metal.addon      the NGPUADD2 file for `unixnu build payload --gpu-addon` (scripts/make-gpu-addon.py's format, paths below
                             /private/var/ngpu): addons/NativeAdrenoMetal.bundle/*. Test-only: no NativeAdrenoMetal.publish beside it, so the
                             plugin only tests. Published: the bundle also holds UniXNU's native Metal frontend (NativeBridgeMetalFrontend.mm)
                             and the bridge, built with ASTEROIDS_METAL_PUBLISH, and addons/NativeAdrenoMetal.publish is beside it. No daemon,
                             no launchd job either way. T41: also audio/ShengAudio.driver/*, the audio server plug-in (--no-audio leaves it out).
  owner-trust.bundle         for `--owner-trust-cache`: one owner module with the CDHashes (plugin, Turnip, audio plug-in), then the original modules of
                             ORIGINAL.bundle byte for byte
The kernel's addon mount presents addons/ over /System/Library/Extensions, where the original discovery loads <MetalPluginName>.bundle;
GPU power mode 19 publishes the accelerator that names it. shader_tools.py builds glslang and SPIRV-Tools when missing."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import plistlib
import re
import shutil
import struct
import subprocess
import sys
import uuid
import zlib

import shader_tools

DEVICE = Path(__file__).resolve().parents[1]
OUT = DEVICE / 'build/asteroids-metal-plugin'
ORIGINAL_TRUST_SHA256 = '7416ca4f8377146626bb284e2133c76bee6cb4e3ee0af5d648b186235a7323a7'
parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
parser.add_argument('--unixnu', type=Path, required=True)
parser.add_argument('--trust-cache', type=Path, required=True, help='the unchanged original static/System trust bundle')
parser.add_argument('--published', action='store_true', help='T18: the published stage with the metal2vulkan bridge')
parser.add_argument('--clients', default='CollectionsPoster PosterBoard iconservicesagent',
                    help='T31: the Metal clients beside backboardd (NativeAdrenoMetal.clients); empty for none')
parser.add_argument('--sample-only', action='store_true', help='T22, with --published: no Metal device, only the stack samples')
parser.add_argument('--no-audio', action='store_true', help='T41: leave out the audio server plug-in (audio/ShengAudio.driver)')
args = parser.parse_args()
ROOT = args.unixnu.resolve()
if args.published:
    OUT = OUT.with_name(OUT.name + ('-sample' if args.sample_only else '-published'))
TURNIP = ROOT / 'build/mesa-tu-ios/src/freedreno/vulkan/libvulkan_freedreno.dylib'
VULKAN_INCLUDE = ROOT / '.cache/mesa-work/include'
METAL = ROOT / 'drivers/AdrenoMetal'
BRIDGE = ROOT / 'build/adreno-metal-bridge/ios/libadreno_metal_bridge.a'


def run(command, **kw):
    print('+', ' '.join(str(c) for c in command), flush=True)
    return subprocess.run([str(c) for c in command], check=True, **kw)


def cdhash(path):
    detail = subprocess.run(['codesign', '--display', '--verbose=4', str(path)], capture_output=True, text=True, check=True).stderr
    if not re.search(r'^Page size=4096$', detail, re.M):
        sys.exit(f'{path} is not signed with 4 KiB pages')
    return re.search(r'^CDHash=([a-f0-9]{40})$', detail, re.M)[1]


if not TURNIP.is_file():
    sys.exit('Build Turnip first: scripts/prepare-asteroids-turnip.py --unixnu ' + str(ROOT))
if args.published and not BRIDGE.is_file():
    sys.exit('Build the bridge first: python3 scripts/build-adreno-metal-bridge.py --target ios (in ' + str(ROOT) + ')')
original = args.trust_cache.read_bytes()
if hashlib.sha256(original).hexdigest() != ORIGINAL_TRUST_SHA256:
    sys.exit('The trust bundle is not the unchanged original static/System bundle')
shutil.rmtree(OUT, ignore_errors=True)
OUT.mkdir(parents=True)

# 1. The smoke shaders (shader_tools.py builds glslang and spirv-val when missing).
bin_dir = shader_tools.ensure(ROOT)
glslang, spirv_val = bin_dir / 'glslang', bin_dir / 'spirv-val'
lines = ['// Generated by UniXNU-asteroids scripts/build-asteroids-metal-plugin.py from UniXNU drivers/AdrenoMetal/shaders; do not edit.',
         '#pragma once', '#include <stdint.h>']
for source in sorted((METAL / 'shaders').glob('smoke_*')):
    spv = OUT / (source.name + '.spv')
    run([glslang, '-V', '--target-env', 'vulkan1.1', '-o', spv, source], stdout=subprocess.DEVNULL)
    run([spirv_val, '--target-env', 'vulkan1.1', spv])
    data = spv.read_bytes()
    words = struct.unpack('<%dI' % (len(data) // 4), data)
    lines.append('static const uint32_t kSpv_%s[] = {' % source.name.replace('.', '_'))
    lines += ['    ' + ', '.join('0x%08x' % w for w in words[i:i + 8]) + ',' for i in range(0, len(words), 8)]
    lines.append('};')
    spv.unlink()
(OUT / 'NativeTurnipShaders.h').write_text('\n'.join(lines) + '\n')

# 2. The plugin executable. TurnipReplay.h is on no include path here (AsteroidsMetalPlugin.mm refuses to build with it).
sdk = subprocess.check_output(['xcrun', '--sdk', 'iphoneos', '--show-sdk-path'], text=True).strip()
base = ['-target', 'arm64-apple-ios17.0', '-isysroot', sdk, '-O2', '-Wall', '-Wextra', '-Werror']
includes = ['-I', DEVICE / 'drivers/Adreno740', '-I', METAL, '-I', ROOT / 'kits/AdrenoPM4', '-I', VULKAN_INCLUDE, '-I', OUT]
pm4 = OUT / 'AdrenoPM4.o'
run(['xcrun', 'clang', *base, '-std=c11', '-c', ROOT / 'kits/AdrenoPM4/AdrenoPM4.c', '-o', pm4])
bundle = OUT / 'NativeAdrenoMetal.bundle'
bundle.mkdir()
exe = bundle / 'NativeAdrenoMetal'
# Published: the native frontend and the bridge, as UniXNU's build-adreno-metal-plugin.py links them (without its TurnipReplay and prime list).
published = (['-DASTEROIDS_METAL_PUBLISH=1', '-x', 'objective-c++', METAL / 'NativeBridgeMetalFrontend.mm'] if args.published else [])
run(['xcrun', 'clang++', *base, '-std=c++17', '-fobjc-arc', '-Wno-deprecated-declarations', '-bundle', *includes,
     '-x', 'objective-c++', DEVICE / 'package/AsteroidsMetalPlugin.mm', *published, '-x', 'none', pm4,
     *([BRIDGE, '-Wl,-dead_strip', '-framework', 'IOSurface'] if args.published else []),
     '-framework', 'Foundation', '-framework', 'Metal', '-o', exe])
pm4.unlink()
# The symbols for the T20 stack samples (offsets in the plugin go back to functions with atos or nm).
shutil.copyfile(exe, OUT / 'NativeAdrenoMetal.unstripped')
run(['strip', '-x', exe])
header = exe.read_bytes()[:12]
if struct.unpack('<3I', header)[:2] != (0xfeedfacf, 0x100000c) or struct.unpack('<3I', header)[2] & 0xffffff:
    sys.exit('Expected an ordinary ARM64 (subtype 0) Mach-O')

# 3. Turnip inside the bundle (NativeTurnipSmoke.h's libraryPath looks next to the executable), signed on its own before the bundle so
#    the bundle's resource seal covers its final bytes.
turnip = bundle / 'libvulkan_freedreno.dylib'
shutil.copyfile(TURNIP, turnip)
run(['strip', '-x', turnip])
run(['codesign', '--force', '--sign', '-', '--pagesize', '4096', '--identifier', 'org.rootstockos.turnip', turnip])
(bundle / 'Info.plist').write_bytes(plistlib.dumps(dict(
    CFBundleIdentifier='org.rootstockos.NativeAdrenoMetal', CFBundleExecutable='NativeAdrenoMetal', CFBundlePackageType='BNDL',
    CFBundleVersion='1', CFBundleShortVersionString='1', NSPrincipalClass='NativeAdrenoMetalDevice', MinimumOSVersion='17.0'),
    fmt=plistlib.FMT_XML))
run(['codesign', '--force', '--sign', '-', '--pagesize', '4096', '--identifier', 'org.rootstockos.NativeAdrenoMetal', bundle])
for path in (bundle, turnip):
    run(['codesign', '--verify', '--strict', path])
hashes = {'plugin_cdhash': cdhash(bundle), 'turnip_cdhash': cdhash(turnip)}
# T41: sheng's audio server plug-in (hal/ShengAudioPlugIn.c, scripts/build-sheng-audio-hal.py), which opens the kernel's audio service
# (package/AsteroidsAudioProbe.cpp, sheng-audio). The addon carries it as audio/ShengAudio.driver; the kernel mounts audio/ over
# /System/Library/Audio/Plug-Ins/HAL (patches/xnu/zzyd-board-audio-addon-mount.patch), and its CDHash joins the owner trust module.
audio = None
if not args.no_audio:
    run([sys.executable, DEVICE / 'scripts/build-sheng-audio-hal.py', OUT / 'audio-hal'])
    audio = OUT / 'audio-hal/ShengAudio.driver'
    hashes['audio_cdhash'] = cdhash(audio)

# 4. The addon (make-gpu-addon.py's NGPUADD2 layout): every file of the bundle, executables 0755.
files = []
for path in sorted(p for p in bundle.rglob('*') if p.is_file()):
    mode = 0o755 if path in (exe, turnip) else 0o644
    files.append(('addons/NativeAdrenoMetal.bundle/' + path.relative_to(bundle).as_posix(), mode, path.read_bytes()))
if audio:
    for path in sorted(p for p in audio.rglob('*') if p.is_file()):
        files.append(('audio/ShengAudio.driver/' + path.relative_to(audio).as_posix(), 0o755 if path.name == 'ShengAudio' else 0o644,
                      path.read_bytes()))
if args.published:
    # The marker the plugin looks for beside the bundle; outside the bundle, so its seal and CDHash are the same either way.
    files.append(('addons/NativeAdrenoMetal.sample', 0o644, b'sample\n') if args.sample_only else
                 ('addons/NativeAdrenoMetal.publish', 0o644, b'published\n'))
    if not args.sample_only:
        # T31: the Metal clients beside backboardd (the kernel's NativeSandboxGrantProcess grants the same three; NativeAdreno has four slots):
        # by default the active wallpaper (WallpaperKit's CollectionsPoster), PosterBoard and iconservicesagent, after UniXNU-sheng's list.
        # No clients is a file with no names (one new line: the kernel's addon check refuses empty entries), not a missing file: the
        # addon's files stay on Data, so an earlier list would otherwise stay in force.
        files.append(('addons/NativeAdrenoMetal.clients', 0o644, (''.join(name + '\n' for name in args.clients.split()) or '\n').encode()))
pad = lambda data: data + bytes(-len(data) % 8)
body = b''
for name, mode, data in files:
    encoded = name.encode() + b'\0'
    assert len(encoded) <= 160 and '..' not in name.split('/') and not name.startswith('/')
    packed = zlib.compress(data, 9)
    body += struct.pack('<IIQQ', len(encoded), mode, len(data), len(packed)) + pad(encoded) + pad(packed)
addon = OUT / 'asteroids-metal.addon'
addon.write_bytes(b'NGPUADD2' + struct.pack('<IIQ', len(files), 0, 24 + len(body)) + body)

# 5. The owner trust bundle (make-owner-trust-bundle.py's layout; DarwinBoot wants a module's hashes strictly ascending).
count = struct.unpack_from('<I', original)[0]
assert count == 2
offsets = struct.unpack_from('<2I', original, 4)
originals = [original[offsets[0]:offsets[1]], original[offsets[1]:]]
ordered = sorted(bytes.fromhex(h) for h in hashes.values())
if len(set(ordered)) != len(ordered):
    sys.exit('Duplicate CDHash')
identifier = uuid.uuid5(uuid.NAMESPACE_URL, 'org.rootstockos.asteroids-metal-plugin:' + ''.join(h.hex() for h in ordered))
own = struct.pack('<I16sI', 2, identifier.bytes, len(ordered)) + b''.join(h + bytes([2, 0, 0, 0]) for h in ordered)
modules = [own, *originals]
starts, cursor = [], 16
for module in modules:
    starts.append(cursor)
    cursor += len(module)
trust = struct.pack('<4I', 3, *starts) + b''.join(modules)
assert trust[starts[1]:] == original[offsets[0]:]
(OUT / 'owner-trust.bundle').write_bytes(trust)
(OUT / 'trust.json').write_text(json.dumps(dict(hashes, addon_sha256=hashlib.sha256(addon.read_bytes()).hexdigest(),
    owner_trust_sha256=hashlib.sha256(trust).hexdigest(), shader_tools=shader_tools.TAG,
    purpose='owner-built %s Metal plugin and Turnip; trusted only through an extra owner trust module' % ('published' if args.published else 'test-only')), indent=2) + '\n')
for name, mode, data in files:
    print(f'{mode:o} {len(data):>10} {name}')
print('PASS: %s addon' % ('T18 published' if args.published else 'T17'), addon, f'({addon.stat().st_size} bytes), owner trust bundle with', ', '.join(hashes.values()))
