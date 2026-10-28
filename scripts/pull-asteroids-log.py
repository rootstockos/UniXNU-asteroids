#!/usr/bin/env python3
"""Read the UniXNU diagnostic log (package/BootLog.c, sheng-log=1) of the last boot from the phone and decode it.

The phone runs LineageOS Recovery with ADB (root shell). The log window is the last 32 MiB of userdata (NativeUfs kLogReserveBlocks,
8192 blocks of 4096 bytes); it is copied with one dd into
the phone's /tmp (RAM) and fetched with adb pull, nothing on the phone's storage is written, and sheng's decoder (scripts/read-sheng-log.py
--decode, UniXNU-sheng 08cde20) writes header.txt, msgbuf.txt, slots.txt and latest.txt. When the header names a diagnostic export
(sheng-diag-export, or sheng-gpu-trace-export's rounds), it is read from the export region below the window the same way and unpacked
into diag/; a GPU trace in it (Turnip's TU_DW_TRACE) is decoded by scripts/gpu-trace.py into gpu-trace.txt.

    python3 scripts/pull-asteroids-log.py --out <dir>
"""
import argparse
import importlib.util
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
BLOCK = 4096
LOG_BLOCKS = 8192
# board.toml [userdata]: the partition in 4096-byte LBAs.
USERDATA_FIRST, USERDATA_LAST = 0x256248, 0x3a8bffa


def adb(*args, check=True):
    return subprocess.run(['adb', *args], check=check, capture_output=True).stdout


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--out', required=True, type=Path)
    parser.add_argument('--device', default='/dev/block/by-name/userdata')
    args = parser.parse_args()
    args.out.mkdir(parents=True, exist_ok=True)
    if adb('shell', 'id -u').decode().strip() != '0':
        sys.exit('adb shell is not root: enable ADB in LineageOS Recovery')
    partition = USERDATA_LAST - USERDATA_FIRST + 1
    # The window goes to the phone's /tmp first and comes over with adb pull: a 32 MiB exec-out stream broke partway on this host's USB link.
    remote = '/tmp/asteroids-log.bin'
    # The shell can end with status 255 after dd has finished (the link drops as the session closes), so the copy is checked by its size.
    adb('shell', f'dd if={args.device} of={remote} bs={BLOCK} skip={partition - LOG_BLOCKS} count={LOG_BLOCKS} 2>/dev/null', check=False)
    copied = adb('shell', f'stat -c %s {remote}', check=False).decode().strip()
    if copied != str(LOG_BLOCKS * BLOCK):
        sys.exit(f'the copy in {remote} has {copied or "no"} bytes, not {LOG_BLOCKS * BLOCK}')
    log = args.out / 'log.bin'
    adb('pull', remote, str(log))
    adb('shell', f'rm -f {remote}', check=False)
    if log.stat().st_size != LOG_BLOCKS * BLOCK:
        sys.exit(f'short read: {log.stat().st_size} of {LOG_BLOCKS * BLOCK} bytes')
    subprocess.run([sys.executable, ROOT / 'scripts/read-sheng-log.py', '--decode', str(log), '--out', str(args.out)], check=True)
    for run in sorted(p for p in args.out.iterdir() if p.is_dir() and (p / 'header.txt').exists()):
        pull_diag(args, run, partition)


def pull_diag(args, run, partition):
    h = dict(line.split('=', 1) for line in (run / 'header.txt').read_text().splitlines() if '=' in line)
    size, first, below = (int(h.get(k, 0)) for k in ('diag_bytes', 'diag_first', 'export_below'))
    if not size or not below:
        return
    blocks = (size + BLOCK - 1) // BLOCK
    remote = '/tmp/asteroids-diag.bin'
    adb('shell', f'dd if={args.device} of={remote} bs={BLOCK} skip={partition - LOG_BLOCKS - below + first} count={blocks} 2>/dev/null',
        check=False)
    blob = run / 'diag-region.bin'
    adb('pull', remote, str(blob))
    adb('shell', f'rm -f {remote}', check=False)
    if blob.stat().st_size != blocks * BLOCK:
        sys.exit(f'diag export: short read, {blob.stat().st_size} of {blocks * BLOCK} bytes')
    spec = importlib.util.spec_from_file_location('read_sheng_log', ROOT / 'scripts/read-sheng-log.py')
    reader = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(reader)
    reader.unpack_export(blob.read_bytes()[:size], run, 'diag')
    blob.unlink()
    for trace in sorted((run / 'diag/files').glob('private/var/*/Library/Caches/com.apple.backboardd/m2v/gpu/trace.bin')):
        with open(run / 'gpu-trace.txt', 'w') as out:
            subprocess.run([sys.executable, ROOT / 'scripts/gpu-trace.py', str(trace)], check=False, stdout=out)
        print(f'{run / "gpu-trace.txt"}: from {trace}')


if __name__ == '__main__':
    main()
