#!/usr/bin/env python3
"""Copy the persistent Data volume of the last boot from the phone into a disk image on this Mac.

The phone runs LineageOS Recovery with ADB enabled (root shell), or Android with a root adb shell (Rooted debugging and
`adb root`), or Android with `su` (--su). Nothing on the phone is written: the script only reads the
userdata partition with dd. The Data disk is the packed seed (the payload's Data ramdisk) with the pages the running system
changed on top; those pages live in the persistent store window at the end of userdata (drivers/RamBlock/PersistOverlay.h:
header block "SHNGDATA", presence bitmap, page i at 1 + bitmap blocks + i), placed by NativeUfs (native_ufs_store_open: the
window ends where the 1024 MiB log reserve begins).

    python3 scripts/pull-asteroids-data.py --seed <owner-data.dbr> --out <dir>

writes <dir>/data.img (sparse, the full Data disk), <dir>/pages.bin and <dir>/store.json, then attaches the image read-only.
"""
import argparse
import json
import shlex
import struct
import subprocess
import sys
from pathlib import Path

BLOCK = 4096
# board.toml [userdata]: the partition in 4096-byte LBAs.
USERDATA_FIRST, USERDATA_LAST = 0x256248, 0x3a8bffa
RESERVE_BLOCKS = 1024 * 256              # sheng-log-reserve default (1024 MiB)


SU = False


def adb(*args, data=False):
    # --su: run the shell commands through su on a non-root adb shell (Android with a su binary).
    if SU and args[0] in ('shell', 'exec-out'):
        args = (args[0], 'su -c ' + shlex.quote(' '.join(args[1:])))
    out = subprocess.run(['adb', *args], check=True, capture_output=True)
    return out.stdout if data else out.stdout.decode().strip()


def dd(device, skip, count):
    command = f'dd if={shlex.quote(device)} bs={BLOCK} skip={skip} count={count} 2>/dev/null'
    blob = adb('exec-out', command, data=True)
    if len(blob) != count * BLOCK:
        sys.exit(f'short read at block {skip}: {len(blob)} of {count * BLOCK} bytes')
    return blob


def seed_regions(blob):
    magic, version, page, logical, count = struct.unpack_from('<8sIIQQ', blob)
    if magic != b'DBRDISK1' or version != 1 or page != BLOCK:
        sys.exit('not a DBRDISK1 seed')
    cursor, extents = 32 + 16 * count, []
    for index in range(count):
        offset, length = struct.unpack_from('<QQ', blob, 32 + 16 * index)
        extents.append((offset, blob[cursor:cursor + length]))
        cursor += length
    return logical, extents


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--seed', required=True, type=Path, help='the Data seed the boot image was packed with')
    parser.add_argument('--out', required=True, type=Path)
    parser.add_argument('--device', default='/dev/block/by-name/userdata')
    parser.add_argument('--no-attach', action='store_true')
    parser.add_argument('--su', action='store_true', help='Android with su: run the reads through su -c')
    args = parser.parse_args()
    global SU
    SU = args.su
    args.out.mkdir(parents=True, exist_ok=True)

    logical, extents = seed_regions(args.seed.read_bytes())
    pages = logical // BLOCK
    bitmap_blocks = (pages + 8 * BLOCK - 1) // (8 * BLOCK)
    window = 1 + bitmap_blocks + pages
    partition = USERDATA_LAST - USERDATA_FIRST + 1
    start = partition - RESERVE_BLOCKS - window   # partition-relative

    if adb('shell', 'id -u') != '0':
        sys.exit('adb shell is not root: enable ADB in LineageOS Recovery, use adb root (Rooted debugging) or --su')
    device = adb('shell', f'readlink -f {shlex.quote(args.device)}')
    size = int(adb('shell', f'blockdev --getsize64 {shlex.quote(device)}'))
    if size != partition * BLOCK:
        sys.exit(f'{device} is {size} bytes, board.toml says {partition * BLOCK}')

    header = dd(device, start, 1)
    if header[:8] != b'SHNGDATA':
        sys.exit(f'no SHNGDATA header at partition block {start:#x}: no Data store, or another seed size')
    count, generation = struct.unpack_from('<Q', header, 16)[0], struct.unpack_from('<I', header, 56)[0]
    if count != pages:
        sys.exit(f'store has {count} pages, the seed {pages}: the boot used another seed')

    bitmap = dd(device, start + 1, bitmap_blocks)
    present = [byte * 8 + bit for byte, value in enumerate(bitmap) if value
               for bit in range(8) if value >> bit & 1 and byte * 8 + bit < pages]
    runs = []
    for index in present:
        if runs and runs[-1][0] + runs[-1][1] == index:
            runs[-1][1] += 1
        else:
            runs.append([index, 1])
    print(f'store at partition block {start:#x}, generation {generation}, {len(present)} changed pages in {len(runs)} runs')

    script = args.out / 'pull.sh'
    base = start + 1 + bitmap_blocks
    script.write_text(''.join(f'dd if={device} bs={BLOCK} skip={base + i} count={n} 2>/dev/null\n' for i, n in runs))
    remote = '/data/local/tmp/pull.sh' if adb('shell', 'test -d /data/local/tmp && echo y || true') == 'y' else '/tmp/pull.sh'
    adb('push', str(script), remote)
    pulled = adb('exec-out', f'sh {remote}', data=True)
    if len(pulled) != len(present) * BLOCK:
        sys.exit(f'pulled {len(pulled)} bytes, expected {len(present) * BLOCK}')
    (args.out / 'pages.bin').write_bytes(pulled)

    image = args.out / 'data.img'
    with open(image, 'wb') as f:
        f.truncate(logical)
        for offset, data in extents:
            f.seek(offset)
            f.write(data)
        cursor = 0
        for index, n in runs:
            f.seek(index * BLOCK)
            f.write(pulled[cursor:cursor + n * BLOCK])
            cursor += n * BLOCK
    (args.out / 'store.json').write_text(json.dumps({
        'device': device, 'store_block': start, 'generation': generation, 'pages': pages,
        'changed_pages': len(present), 'runs': len(runs), 'seed': str(args.seed)}, indent=1) + '\n')
    print(f'Data disk: {image}')
    if not args.no_attach:
        print(subprocess.run(['hdiutil', 'attach', '-readonly', '-owners', 'off', '-imagekey',
                              'diskimage-class=CRawDiskImage', str(image)], check=True, capture_output=True, text=True).stdout)


if __name__ == '__main__':
    main()
