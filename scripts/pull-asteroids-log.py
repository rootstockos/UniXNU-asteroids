#!/usr/bin/env python3
"""Read the UniXNU diagnostic log (package/BootLog.c, sheng-log=1) of the last boot from the phone and decode it.

The phone runs LineageOS Recovery with ADB (root shell). The log window is the last 32 MiB of userdata (NativeUfs kLogReserveBlocks,
8192 blocks of 4096 bytes); it is copied with one dd into
the phone's /tmp (RAM) and fetched with adb pull, nothing on the phone's storage is written, and sheng's decoder (scripts/read-sheng-log.py
--decode, UniXNU-sheng 08cde20) writes header.txt, msgbuf.txt, slots.txt and latest.txt.

    python3 scripts/pull-asteroids-log.py --out <dir>
"""
import argparse
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


if __name__ == '__main__':
    main()
