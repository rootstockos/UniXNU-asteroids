#!/usr/bin/env python3
"""Build the boot progress kit (XNU_BOOT_PROGRESS) against this source kernel's actual ABI."""
# asteroids: UniXNU-sheng 0e1f6e5 scripts/build-boot-progress.py (BSD-2-Clause-Patent) with the kit's sources in
# kits/asteroids (package/, copied from sheng 0e1f6e5) and these changes: the font comes from package/Font.h (Mu-Silicium's
# FrameBufferSerialPortLib copy, AOSP licence), the watchdog policy is ours (uke's, with the Gunyah watchdog pet thread),
# the UI trace and process probes join this archive (sheng builds them with its display), BootProgressStandins.c gives the
# Virtio and owner-root status words this board does not have, and the audio DSP firmware reader is left out.
import hashlib
import json
from pathlib import Path
import subprocess

ROOT = Path(__file__).resolve().parents[1]
KIT = ROOT / 'kits/asteroids'
OUT = ROOT / 'build/boot-progress'
OUT.mkdir(parents=True, exist_ok=True)


def compile_command(record_path, source, obj, extra, own_define):
    record = json.loads((ROOT / record_path).read_text().strip().rstrip(','))
    command, args, i = [], record['arguments'], 0
    while i < len(args):
        arg = args[i]
        if arg == record['file']:
            command.append(str(source))
        elif arg in ('-o', '-MF', '-MT', '-MJ'):
            if arg == '-o':
                command += ['-o', str(obj)]
            i += 1
        elif arg == '-Xclang' and args[i+1].startswith('-W'):
            i += 1
        elif own_define and arg == '-D' and args[i+1].startswith('XNU_BOOT_PROGRESS='):
            i += 1
        elif not arg.startswith('-W') and not (own_define and arg.startswith('-DXNU_BOOT_PROGRESS=')):
            command.append(arg)
        i += 1
    subprocess.run(command + extra, cwd=record['directory'], check=True, timeout=90)


osfmk = 'build/xnu/ios-obj/DEVELOPMENT_ARM64_BCM2837/osfmk/DEVELOPMENT/startup.o.json'
bsd = 'build/xnu/ios-obj/DEVELOPMENT_ARM64_BCM2837/bsd/DEVELOPMENT/kern_exec.o.json'
strict = ['-mgeneral-regs-only', '-Wall', '-Wextra', '-Werror']
osfmk_flags = ['-DXNU_BOOT_PROGRESS=1'] + strict + ['-I', str(ROOT / '.cache/xnu-work/osfmk'), '-I', str(KIT)]
objects = []
for name, record, flags in [
        ('BootProgress.c', osfmk, osfmk_flags),
        ('BootFaultProbe.c', osfmk, osfmk_flags),
        ('BootUiTraceProbe.c', osfmk, osfmk_flags),
        ('BootProcessProbe.c', osfmk, osfmk_flags),
        ('BootProgressStandins.c', osfmk, osfmk_flags),
        # T37: sheng's UFS diagnostic log (sheng-log=1, through NativeUfs's log window) and its weak stand-ins for the probes this
        # board does not build.
        ('BootLog.c', osfmk, osfmk_flags),
        ('BootLogStandins.c', osfmk, osfmk_flags),
        ('BootWatchdogPolicy.c', bsd, strict),
        ('BootDiagExport.c', bsd, strict)]:
    obj = OUT / (Path(name).stem + '.o')
    compile_command(record, KIT / name, obj, flags, record == osfmk)
    objects.append(obj)

archive = OUT / 'libBootProgress.a'
archive.unlink(missing_ok=True)
subprocess.run(['xcrun', 'ar', 'rcs', str(archive), *map(str, objects)], check=True, timeout=30)
(OUT / 'build-report.json').write_text(json.dumps({
    **{Path(o).stem + '_sha256': hashlib.sha256((KIT / (Path(o).stem + ('.c'))).read_bytes()).hexdigest() for o in objects},
    'archive_sha256': hashlib.sha256(archive.read_bytes()).hexdigest(),
    'hardware_execution': False,
}, indent=2) + '\n')
print('PASS: boot progress kit for asteroids')
