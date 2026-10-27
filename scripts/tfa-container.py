#!/usr/bin/env python3
"""List the contents of an NXP TFA98xx settings container (tfa98xx.cnt): its devices, profiles and each list item.

    tfa-container.py tfa98xx.cnt

Written from the container's layout (header "PM", 24-bit offset + 8-bit type descriptors), not from NXP's code. The Nothing Phone (3a)'s
container (proprietary-files: vendor/firmware/tfa98xx.cnt) holds two TFA9873 devices and their profiles; this shows which registers,
bitfields and files each device and profile writes, which is what a driver has to replay."""
import struct
import sys

TYPES = ['device', 'profile', 'register', 'string', 'file', 'patch', 'marker', 'mode', 'inputselect', 'outputselect',
         'programconfig', 'lagw', 'gains', 'vbatfactors', 'sensescal', 'sensesdelay', 'bitfield', 'default', 'livedata',
         'livedatastring', 'group', 'cmd', 'mbdrc', 'filter', 'noinit', 'features', 'cfmem', 'fwkusecase', 'vddpconfig']


def desc(word):
    return word & 0xffffff, word >> 24


def string_at(data, offset):
    end = data.index(b'\0', offset)
    return data[offset:end].decode('ascii', 'replace')


def item(data, word):
    offset, kind = desc(word)
    name = TYPES[kind] if kind < len(TYPES) else f'type{kind}'
    if name == 'string':
        return f'string "{string_at(data, offset)}"'
    if name == 'register':
        address, value, mask = struct.unpack_from('<BHH', data, offset)
        return f'register 0x{address:02x} = 0x{value:04x} mask 0x{mask:04x}'
    if name == 'bitfield':
        value, field = struct.unpack_from('<HH', data, offset)
        return f'bitfield 0x{field:04x} = 0x{value:04x}'
    if name in ('file', 'patch'):
        named, = struct.unpack_from('<I', data, offset)
        size, = struct.unpack_from('<I', data, offset + 4)
        head = data[offset + 8:offset + 8 + 2]
        return f'{name} "{string_at(data, desc(named)[0])}" {size} bytes (header {head!r})'
    if name in ('mode', 'noinit', 'features', 'default', 'marker', 'group', 'cmd'):
        return f'{name} at 0x{offset:x}: {data[offset:offset + 8].hex()}'
    return f'{name} at 0x{offset:x}'


def main():
    data = open(sys.argv[1], 'rb').read()
    ident, version, subversion = data[0:2], data[2:4], data[4:6]
    size, crc, rev = struct.unpack_from('<IIH', data, 6)
    customer, application, kind = (data[o:o + 8].split(b'\0')[0].decode() for o in (16, 24, 32))
    ndev, nprof, nlive = struct.unpack_from('<HHH', data, 40)
    print(f'{ident.decode()} {version.decode()} {subversion.decode()} size {size} crc {crc:08x} rev {rev} '
          f'customer {customer} application {application} type {kind}: {ndev} devices, {nprof} profiles, {nlive} live data')
    # The top index lists the devices; each device's list names its profiles (shared between devices by offset).
    profiles = []
    for i in range(ndev):
        offset, typ = desc(struct.unpack_from('<I', data, 46 + 4 * i)[0])
        length, bus, dev, func, devid, name = struct.unpack_from('<BBBBII', data, offset)
        print(f'\ndevice "{string_at(data, desc(name)[0])}" bus {bus} address 0x{dev:02x} func {func} devid 0x{devid:08x}')
        for j in range(length):
            word = struct.unpack_from('<I', data, offset + 12 + 4 * j)[0]
            if TYPES[desc(word)[1]] == 'profile':
                if desc(word)[0] not in profiles:
                    profiles.append(desc(word)[0])
                continue
            print('   ', item(data, word))
    for offset in profiles:
        head, name = struct.unpack_from('<II', data, offset)
        length, group, ident = head & 0xff, (head >> 8) & 0xff, head >> 16
        print(f'\nprofile at 0x{offset:x} "{string_at(data, desc(name)[0])}" group {group} id {ident}')
        for j in range(length):
            print('   ', item(data, struct.unpack_from('<I', data, offset + 8 + 4 * j)[0]))


if __name__ == '__main__':
    main()
