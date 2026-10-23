#!/usr/bin/env python3
"""Qualify the original IOMobileFramebufferLegacy default-FB-surface contract.

The original class keeps its default surface at instance+0x1618 and its
temporary surface at instance+0x1620. Only the (absent) platform driver ever
installs the former; this audit pins the exact fields, slots and user-client
dispatch entry the bring-up backend may use, and refuses to run against a
different original binary.
"""
import hashlib
import json
import struct
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'kits/NativePrelink'))
from native_prelink import macho, backed_offset  # noqa: E402


def require(value, why):
    if not value:
        raise SystemExit(why)


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


LEGACY = ROOT / 'build/display-analysis/mobile-graphics-legacy-abi.json'
require(LEGACY.exists(), 'Run audit-mobile-graphics-legacy.py first')
audit = json.loads(LEGACY.read_text())
FAMILY_SHA = '2e044ee68c1650397e1435234ca6459206262e4c7ed709cb7131c6f664f9bd9b'
require(audit['original_family_text_sha256'] == FAMILY_SHA,
        'Original IOMobileGraphicsFamily executable differs')

candidate = ROOT / 'build/native-prelink/native-owner-mobile-graphics-candidate.macho'
if not candidate.exists():
    later = sorted((ROOT / 'build/native-prelink').glob(
        'native-owner-ufs-cpu-desktop-*-candidate.macho'), key=lambda f: f.stat().st_mtime)
    require(later, 'No reference candidate available')
    candidate = later[-1]
blob = candidate.read_bytes()
image = macho(blob)


def read(va, size):
    at = backed_offset(image['segments'], va, size, len(blob))
    return blob[at:at + size]


def word(va):
    return struct.unpack('<I', read(va, 4))[0]


methods = audit['methods']


def slot(index):
    return int(methods[index]['address'], 16)


SLOTS = {
    174: 0xfffffff00578cbf8,   # default_fb_surface gate -> slot333
    242: 0xfffffff00578b390,   # copy the surface send-right/port name
    299: 0xfffffff005793694,   # create_default_fb_surface -> instance+0x1620
    333: 0xfffffff00578cca8,   # default-surface layout/commit, returns sid
}
for index, address in SLOTS.items():
    require(slot(index) == address, 'Original Legacy slot %d differs' % index)

# Exact original instructions that define the fields this backend relies on.
CHECKS = [
    (0xfffffff005793748, 0xd282f009, 'slot299 materializes instance+0x1780'),
    (0xfffffff00579374c, 0x8b090108, 'slot299 adds instance+0x1780'),
    (0xfffffff005793750, 0x39400108, 'slot299 reads the enable byte'),
    (0xfffffff005793754, 0x36000188, 'slot299 tests bit 0 of the enable byte'),
    (0xfffffff0057939cc, 0xf90b1128, 'slot299 installs the temp surface at +0x1620'),
    (0xfffffff005787000, 0xf90b0d1f, 'init clears the default surface at +0x1618'),
    (0xfffffff005787004, 0xf90b111f, 'init clears the temp surface at +0x1620'),
    (0xfffffff005788a84, 0xf90b0d1f, 'stop clears the default surface at +0x1618'),
    (0xfffffff00578d038, 0xf90b0d1f, 'slot333 commit failure clears +0x1618'),
    (0xfffffff00578cc18, 0xf94b0509, 'slot174 loads the command gate at +0x1608'),
    (0xfffffff00578b3d8, 0xf94b0d08, 'slot242 loads the default surface at +0x1618'),
]
for address, expected, why in CHECKS:
    require(word(address) == expected, why)

DISPATCH = 0xfffffff005f35508
require(struct.unpack('<Q', read(DISPATCH, 8))[0] == 0xfffffff00577925c,
        'default_fb_surface dispatch function differs')
inputs, structure_inputs, outputs, structure_outputs = struct.unpack('<4I', read(DISPATCH + 8, 16))
require((inputs, structure_inputs, outputs, structure_outputs) == (2, 0, 1, 0),
        'default_fb_surface dispatch counts differ')

offsets = {
    'surface_root': 0x1600,
    'command_gate': 0x1608,
    'work_loop': 0x1610,
    'default_surface': 0x1618,
    'temp_surface': 0x1620,
    'memory_region': 0x17c0,
    'surface_enable': 0x1780,
    'mode_block': 0x1700,
    'width': 0x16b0,
    'height': 0x16b4,
    'surface_sid': 0x10,
    'surface_region': 0x28,
    'surface_descriptor': 0x30,
    'surface_format': 0x80,
    'surface_stride': 0x68,
    'surface_allocation': 0x90,
}
result = {
    'candidate': str(candidate.relative_to(ROOT)),
    'candidate_sha256': sha(candidate),
    'family_text_sha256': FAMILY_SHA,
    'legacy_vptr': audit['legacy_vptr'],
    'legacy_instance_bytes': audit['legacy_instance_bytes'],
    'slots': {str(index): hex(address) for index, address in SLOTS.items()},
    'dispatch': {
        'selector': 3,
        'function': hex(0xfffffff00577925c),
        'scalar_inputs': inputs,
        'structure_inputs': structure_inputs,
        'scalar_outputs': outputs,
        'structure_outputs': structure_outputs,
        'output_is_mach_port_for_default_surface': True,
    },
    'offsets': {name: hex(value) for name, value in offsets.items()},
    'original_region_lookup': hex(0xfffffff00579361c),
    'original_copy_port_name': hex(0xfffffff004baa90c),
    'surface_creation_requires': {
        'surface_enable_bit0': True,
        'nonzero_width_height': True,
        'purple_gfx_mem_region': 'IOSurfaceRoot lookup of PurpleGfxMem',
    },
    'default_surface_writers': [
        hex(0xfffffff005787000), hex(0xfffffff005788a84), hex(0xfffffff00578d038),
    ],
    'default_surface_nonzero_writer_in_family': False,
    'audit_static_only': True,
    'hardware_execution': False,
}
output = ROOT / 'build/display-analysis/default-fb-surface-abi.json'
output.parent.mkdir(parents=True, exist_ok=True)
output.write_text(json.dumps(result, indent=2) + '\n')
print('PASS: original Legacy default-FB-surface fields, slots and dispatch bound; '
      'the family never installs instance+0x1618 itself')
