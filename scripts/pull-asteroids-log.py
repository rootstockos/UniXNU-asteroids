#!/usr/bin/env python3
"""Read the UniXNU diagnostic log (package/BootLog.c, sheng-log=1) of the last boot from the phone and decode it.

The phone runs LineageOS Recovery with ADB (root shell). The log window is the last 32 MiB of userdata (NativeUfs kLogReserveBlocks,
8192 blocks of 4096 bytes); it is copied with one dd, nothing on the phone is written, and sheng's decoder (scripts/read-sheng-log.py
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


def adb(*args):
    return subprocess.run(['adb', *args], check=True, capture_output=True).stdout


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--out', required=True, type=Path)
    parser.add_argument('--device', default='/dev/block/by-name/userdata')
    args = parser.parse_args()
    args.out.mkdir(parents=True, exist_ok=True)
    if adb('shell', 'id -u').decode().strip() != '0':
        sys.exit('adb shell is not root: enable ADB in LineageOS Recovery')
    partition = USERDATA_LAST - USERDATA_FIRST + 1
    window = adb('exec-out', f'dd if={args.device} bs={BLOCK} skip={partition - LOG_BLOCKS} count={LOG_BLOCKS} 2>/dev/null')
    if len(window) != LOG_BLOCKS * BLOCK:
        sys.exit(f'short read: {len(window)} of {LOG_BLOCKS * BLOCK} bytes')
    log = args.out / 'log.bin'
    log.write_bytes(window)
    subprocess.run([sys.executable, ROOT / 'scripts/read-sheng-log.py', '--decode', str(log), '--out', str(args.out)], check=True)


if __name__ == '__main__':
    main()
