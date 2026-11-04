#!/usr/bin/env python3
"""remap-24A446-display-fixture.py IPAD_KERNELCACHE IPHONE_KERNELCACHE SRC_FIXTURE OUT: the IOMobileFramebufferLegacy ABI fixture of iPad17,3 24A446 rewritten for iPhone17,4 24A446.
Slots are the iPhone's own vtable pointers (PAC fields equal to the iPad's, targets verified by offset or identical code), bindings
and checks are moved by fileset entry, segment and offset and verified (checks byte for byte), offsets and slot indices are kept."""
import hashlib, json, sys
sys.path.insert(0, str(__import__('pathlib').Path(__file__).resolve().parent))
from kcremap_24A446 import Mapper, slot
IPAD, IPHONE, src, out = sys.argv[1:5]
d = json.load(open(src)); m = Mapper(IPAD, IPHONE)
assert d['kernelcache_sha256'] == hashlib.sha256(m.s.data).hexdigest()
r = dict(d); r['kernelcache_sha256'] = hashlib.sha256(m.d.data).hexdigest()
r['collection_slide'] = hex(int(d['collection_slide'], 16) + m.s.image.base - m.d.image.base)
vt = int(d['vtable'], 16)
new_vt, _ = m.raw(vt); r['vtable'] = hex(new_vt)
slots = []
for i, (t, div) in enumerate(d['slots']):
    site, b, how = slot(m, vt + 8 * i)
    if not site: sys.exit(f'slot {i}: {how}')
    slots.append([hex(b[0]), div])   # the diversity column keeps the fixture's (pure slots carry their callers' diversity)
r['slots'] = slots
def code_or_data(va):
    loc = m.s.locate(va)
    if loc and loc[1].startswith('__TEXT'):
        new, how = m.code(va, 3)
        if not new: sys.exit(f'{va:#x}: {how}')
        return new
    new, where = m.raw(va)
    if not new: sys.exit(f'{va:#x}: {where}')
    return new
r['bindings'] = {k: hex(code_or_data(int(v, 16))) for k, v in d['bindings'].items()}
checks = []
for va, hexbytes in d['checks']:
    new = code_or_data(int(va, 16)); data = bytes.fromhex(hexbytes)
    if m.d.data[m.d.image.offset(new, len(data)):][:len(data)] != data: sys.exit(f'check {va} bytes differ at {new:#x}')
    checks.append([hex(new), hexbytes])
r['checks'] = checks
r['source'] = f'remapped from {src.split("/")[-1]} (iPad17,3) by scripts/remap-24A446-display-fixture.py'
json.dump(r, open(out, 'w'), indent=1); open(out, 'a').write('\n')
print('wrote', out, 'vtable', r['vtable'], 'slide', r['collection_slide'])
