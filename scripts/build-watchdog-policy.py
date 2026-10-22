#!/usr/bin/env python3
"""Build the launchd watchdog policy (package/BootWatchdogPolicy.c) against this kernel's kern_exec compile record.

The policy half of Allenwdk/UniXNU-uke scripts/build-boot-log.py (BSD-2-Clause-Patent), without uke's pram log
writer. XNU_WATCHDOG_POLICY (our UniXNU fork) links build/watchdog-policy/libWatchdogPolicy.a.
"""
import hashlib
import json
from pathlib import Path
import subprocess

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / 'build/watchdog-policy'
OUT.mkdir(parents=True, exist_ok=True)
record = json.loads((ROOT / 'build/xnu/ios-obj/DEVELOPMENT_ARM64_BCM2837/bsd/DEVELOPMENT/kern_exec.o.json').read_text().strip().rstrip(','))
source = ROOT / 'kits/asteroids/BootWatchdogPolicy.c'
obj = OUT / 'BootWatchdogPolicy.o'
command = []
args = record['arguments']
i = 0
while i < len(args):
    arg = args[i]
    if arg == record['file']:
        command.append(str(source))
    elif arg in ('-o', '-MF', '-MT', '-MJ'):
        if arg == '-o':
            command += ['-o', str(obj)]
        i += 1
    elif arg == '-Xclang' and args[i + 1].startswith('-W'):
        i += 1
    elif not arg.startswith('-W'):
        command.append(arg)
    i += 1
command += ['-mgeneral-regs-only', '-Wall', '-Wextra', '-Werror']
subprocess.run(command, cwd=record['directory'], check=True, timeout=120)
archive = OUT / 'libWatchdogPolicy.a'
archive.unlink(missing_ok=True)
subprocess.run(['xcrun', 'ar', 'rcs', str(archive), str(obj)], check=True, timeout=30)
(OUT / 'build-report.json').write_text(json.dumps({
    'source_sha256': hashlib.sha256(source.read_bytes()).hexdigest(),
    'archive_sha256': hashlib.sha256(archive.read_bytes()).hexdigest(),
    'hardware_execution': False,
}, indent=2) + '\n')
print('PASS: watchdog policy built against the kern_exec compile record')
