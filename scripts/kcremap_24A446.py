#!/usr/bin/env python3
"""kcremap-24A446.py: map addresses of one fileset kernelcache to another of the same build (iPad17,3 -> iPhone17,4, 24A446).

An address is located by its fileset entry, segment and offset in the source cache and moved to the same entry, segment and
offset in the destination. Every mapped address is verified: code by its instructions with address-dependent fields masked,
data by its chained fixups (each pointer's target maps to the destination's pointer target, with the same PAC diversity).
Where the offset does not verify (the kernel itself differs per SoC), a unique masked-instruction match in the destination
segment is searched instead."""
import struct, sys
sys.path.insert(0, str(__import__('pathlib').Path(__file__).resolve().parents[2] / 'UniXNU/kits/XnuPrelink'))
from kc_image import Image

def mask(x):
    if (x & 0x7c000000) == 0x14000000: return x & 0xfc000000            # b, bl
    if (x & 0x9f000000) == 0x90000000: return x & 0x9f00001f            # adrp
    if (x & 0x9f000000) == 0x10000000: return x & 0x9f00001f            # adr
    if (x & 0xff000010) == 0x54000000: return x & 0xff00001f            # b.cond
    if (x & 0x7e000000) == 0x34000000: return x & 0xff00001f            # cbz, cbnz
    if (x & 0x7e000000) == 0x36000000: return x & 0xfff8001f            # tbz, tbnz
    if (x & 0x7f800000) == 0x11000000: return x & 0xffc003ff            # add imm (the low half of an adrp pair)
    if (x & 0x3b000000) == 0x39000000: return x & 0xffc003ff            # ldr/str unsigned imm
    if (x & 0x3b000000) == 0x18000000: return x & 0xff00001f            # ldr literal
    return x

class Cache:
    def __init__(self, path):
        self.data = open(path, 'rb').read()
        self.image = Image(self.data)
        self.fix = self.image.fixups()
        self.segs = []   # (va, end, entry, segname)
        for name, (va, fileoff) in self.image.entries.items():
            sub = Image(self.data, fileoff)
            for s in sub.segments:
                if s['size'] and s['name'] != '__LINKEDIT':
                    self.segs.append((s['va'], s['va'] + s['size'], name, s['name']))
        self.segs.sort()
        self.by = {(e, n): (va, end) for va, end, e, n in self.segs}
    def locate(self, va):
        for start, end, e, n in self.segs:
            if start <= va < end: return e, n, va - start
        return None
    def word(self, va):
        return struct.unpack_from('<I', self.data, self.image.offset(va, 4))[0]
    def words(self, va, n):
        try: return [mask(self.word(va + 4 * i)) for i in range(n)]
        except ValueError: return None

class Mapper:
    def __init__(self, src, dst):
        self.s, self.d = Cache(src), Cache(dst)
    def raw(self, va):
        where = self.s.locate(va)
        if not where: return None, 'unlocated'
        e, n, off = where
        if (e, n) not in self.d.by: return None, f'{e} {n} missing'
        return self.d.by[(e, n)][0] + off, (e, n)
    def code(self, va, n=12):
        new, where = self.raw(va)
        want = self.s.words(va, n)
        if new is not None and want and self.d.words(new, n) == want: return new, 'offset'
        if not want: return None, 'unreadable'
        e, segname = where if isinstance(where, tuple) else (None, None)
        hits = []
        for start, end, de, dn in self.d.segs:
            if dn != (segname or '__TEXT_EXEC') or (e and de != e and e != 'com.apple.kernel'): continue
            if e and de != e: continue
            for at in range(start, end - 4 * n, 4):
                if mask(self.d.word(at)) == want[0] and self.d.words(at, n) == want: hits.append(at)
        return (hits[0], 'search') if len(hits) == 1 else (None, f'{len(hits)} matches')
    def pointer(self, site):
        """A data site holding a fixed-up pointer: the mapped site, verified by its target and diversity."""
        new, _ = self.raw(site)
        a, b = self.s.fix.get(site), self.d.fix.get(new) if new else None
        if not a or not b: return None, 'no fixup'
        ta, _ = self.raw(a[0])
        if ta != b[0] or a[1:4] != b[1:4]: return None, f'pointer differs {a} {b}'
        return new, 'fixup'

def slot(m, site):
    """A vtable slot: the destination's own pointer at the mapped site, accepted when its PAC fields equal the source's and its
    target is the source target moved by offset or, failing that, code identical to the source target's."""
    new, _ = m.raw(site)
    a, b = m.s.fix.get(site), m.d.fix.get(new) if new else None
    if not a or not b: return None, None, 'no fixup'
    if a[1:5] != b[1:5]: return None, None, f'pac differs {a[1:5]} {b[1:5]}'
    moved, _ = m.raw(a[0])
    if moved == b[0]: return new, b, 'offset'
    sa, sb = m.s.words(a[0], 12), m.d.words(b[0], 12)
    if sa and sa == sb: return new, b, 'code'
    # The kernel is built per SoC: a kernel target the destination's own vtable names stands, PAC fields being equal.
    if m.s.locate(a[0])[:2] == m.d.locate(b[0])[:2] == ('com.apple.kernel', '__TEXT_EXEC'): return new, b, 'kernel'
    return None, None, f'target differs {a[0]:#x} -> {b[0]:#x} (moved {moved:#x})'
