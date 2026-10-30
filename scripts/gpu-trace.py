#!/usr/bin/env python3
"""Read the render server's GPU trace (Turnip's TU_DW_TRACE, turnip-asteroids-timing.patch) and report on it.

The trace is a file of binary records that Turnip writes in the render server's cache directory (m2v/gpu/trace.bin) when the boot
arguments carry asteroids-tu-trace=<letters>; sheng-gpu-trace-export=<seconds> exports it with the boot log, and
scripts/pull-asteroids-log.py unpacks it and runs this script. Every pass, draw and shader is in it, so a new question is a change to
this script, not a new build.

    python3 scripts/gpu-trace.py <trace.bin>                     the report
    python3 scripts/gpu-trace.py <trace.bin> --csv passes|draws|shaders|frags > table.csv
    python3 scripts/gpu-trace.py <trace.bin> --binaries <dir>    each shader's binary as <dir>/<stage>-<id>.bin (ir3 disassembler input)

Records (little endian, packed; the C structs are in tu_cmd_buffer.cc, tu_dw_trace_*): u16 type, u16 0, u32 bytes, then the fields.
The file starts with "TUDWTRC1", u32 counter frequency, u32 pid, u64 the CPU's monotonic ns at open.
"""
import argparse
import collections
import csv
import struct
import sys
from pathlib import Path

HEAD = struct.Struct('<HHI')
FILE_HEAD = struct.Struct('<8sIIQ')
PASS = struct.Struct('<IHBBQQI5H4BI5Q' + 'IBB2x' * 4)
DRAW = struct.Struct('<IHHQQIBBBBB6BBB3x4HQ')
SHADER = struct.Struct('<QBBBBbbbbII16H8HI')
FRAG = struct.Struct('<IHBBIIQ')
NOTE = struct.Struct('<I')
SUBMIT = struct.Struct('<QQII')
TYPES = {1: 'pass', 3: 'draw', 4: 'shader', 5: 'frag', 6: 'note', 7: 'submit'}
KINDS = ['gmem', 'sysmem', 'blit', 'copy', 'resolve', 'clear']
STAGES = {0: 'vs', 1: 'tcs', 2: 'tes', 3: 'gs', 4: 'fs', 5: 'cs'}
LARGE_PX = 1 << 20
BLEND_FACTORS = ['0', '1', 'Sc', '1-Sc', 'Dc', '1-Dc', 'Sa', '1-Sa', 'Da', '1-Da', 'Cc', '1-Cc', 'Ca', '1-Ca', 'Sa_sat', 'S1c',
                 '1-S1c', 'S1a', '1-S1a']
BLEND_OPS = ['add', 'sub', 'rsub', 'min', 'max']
FORMATS = {9: 'R8', 16: 'RG8', 37: 'RGBA8', 43: 'RGBA8_SRGB', 44: 'BGRA8', 50: 'BGRA8_SRGB', 58: 'A2RGB10', 64: 'A2BGR10',
           76: 'R16F', 83: 'RG16F', 97: 'RGBA16F', 100: 'R32F', 109: 'RGBA32F', 124: 'D16', 126: 'D32F', 127: 'S8', 129: 'D24S8',
           130: 'D32FS8'}


def read(path):
    data = Path(path).read_bytes()
    magic, freq, pid, ns = FILE_HEAD.unpack_from(data)
    if magic != b'TUDWTRC1':
        sys.exit(f'{path}: not a TU_DW_TRACE file')
    t = dict(freq=freq, pid=pid, ns=ns, pass_=[], draw=[], shader={}, frag=[], submit=[], lost=0, torn=0, binaries={})
    at = FILE_HEAD.size
    while at + HEAD.size <= len(data):
        kind, _, size = HEAD.unpack_from(data, at)
        if size < HEAD.size or at + size > len(data):
            t['torn'] = len(data) - at   # the last write was cut (the export copied the file while Turnip was writing)
            break
        body = at + HEAD.size
        if kind == 1:
            v = PASS.unpack_from(data, body)
            p = dict(zip(('serial', 'pass_no', 'kind', 'flags', 'start', 'end', 'area', 'width', 'height', 'tiles', 'tile_w',
                          'tile_h', 'loads', 'stores', 'subpasses', 'attachments', 'draws', 'st_begin', 'st_last', 'sum_loads',
                          'sum_draws', 'sum_stores'), v[:22]))
            p['att'] = [dict(format=v[22 + 3 * i], samples=v[23 + 3 * i], flags=v[24 + 3 * i]) for i in range(4)]
            t['pass_'].append(p)
        elif kind == 3:
            v = DRAW.unpack_from(data, body)
            t['draw'].append(dict(zip(('serial', 'pass_no', 'draw_no', 'vs', 'fs', 'count', 'flags', 'att_count', 'blend_mask',
                                       'write_mask', 'write_enables', 'src_color', 'dst_color', 'color_op', 'src_alpha',
                                       'dst_alpha', 'alpha_op', 'ds', 'samples_per_px', 'sx', 'sy', 'sw', 'sh', 'samples'), v)))
        elif kind == 4:
            v = SHADER.unpack_from(data, body)
            s = dict(zip(('id', 'stage', 'double_threadsize', 'subgroup_size', 'early_preamble', 'max_reg', 'max_half_reg',
                          'max_waves', '_pad', 'constlen', 'size', 'sizedwords', 'instrs', 'preamble_instrs', 'nops', 'movs',
                          'covs', 'loops', 'stp', 'ldp', 'ss', 'sy', 'sstall', 'systall', 'last_baryf', 'last_helper', '_pad2'),
                         v[:27]))
            s['cat'] = list(v[27:35])
            bin_bytes = v[35]
            start = body + SHADER.size
            if bin_bytes:
                t['binaries'][s['id']] = data[start:start + bin_bytes]
            t['shader'][s['id']] = s
        elif kind == 5:
            v = FRAG.unpack_from(data, body)
            t['frag'].append(dict(zip(('serial', 'pass_no', 'sysmem', 'has_samples', 'draws', 'area', 'samples'), v)))
        elif kind == 6:
            t['lost'] += NOTE.unpack_from(data, body)[0]
        elif kind == 7:
            # T78 (letter q): a kernel submission, CLOCK_UPTIME_RAW ns before and after it (the frames trace's clock, in us there),
            # the queue's fence, then its command buffers' serials.
            before, after, fence, count = SUBMIT.unpack_from(data, body)
            serials = struct.unpack_from(f'<{count}I', data, body + SUBMIT.size)
            t['submit'].append(dict(before=before, after=after, fence=fence, serials=serials))
        at += size
    if not t['torn'] and at < len(data):
        t['torn'] = len(data) - at
    return t


def link_secondaries(t):
    """The Metal bridge records a pass's draws in a secondary command buffer, so the draws carry the secondary's serial and pass 0 while
    the pass carries the primary's (T61). A recording takes its serial at its first record: a secondary's draws come just before the
    primary's pass, so each secondary goes to the first primary serial above it, and to that primary's pass whose autotune draw count
    equals the secondary's draws (else its first pass). A join counts as verified where the draws add up to the autotune's count."""
    import bisect
    pass_keys = sorted({(p['serial'], p['pass_no']) for p in t['pass_'] if p['kind'] < 2})
    if not t['draw'] or {(d['serial'], d['pass_no']) for d in t['draw']} & set(pass_keys):
        return 0, 0
    frag = {(f['serial'], f['pass_no']): f['draws'] for f in t['frag']}
    by_serial = collections.defaultdict(list)
    for d in t['draw']:
        by_serial[d['serial']].append(d)
    primaries = sorted({k[0] for k in pass_keys})
    passes_of = collections.defaultdict(list)
    for s_, n in pass_keys:
        passes_of[s_].append(n)
    joined = collections.Counter()
    for sec in sorted(by_serial):
        i = bisect.bisect_right(primaries, sec)
        if i == len(primaries):
            continue
        prim = primaries[i]
        draws = by_serial[sec]
        target = next((n for n in passes_of[prim] if frag.get((prim, n)) == len(draws)), passes_of[prim][0])
        for d in draws:
            d['secondary'] = sec
            d['serial'], d['pass_no'] = prim, target
        joined[prim, target] += len(draws)
    verified = sum(1 for k, n in joined.items() if frag.get(k) == n)
    return verified, sum(1 for k in joined if k in frag)


def us(ticks, freq):
    return ticks * 1e6 / freq


def ok_time(p):
    return p['start'] and p['end'] > p['start'] and p['end'] - p['start'] < 19200000


def blend_name(d):
    if not (d['blend_mask'] & 1):
        return 'opaque' if d['write_mask'] else 'no color write'
    f = lambda i: BLEND_FACTORS[i] if i < len(BLEND_FACTORS) else str(i)
    op = BLEND_OPS[d['color_op']] if d['color_op'] < len(BLEND_OPS) else str(d['color_op'])
    return f'{f(d["src_color"])}*S {op} {f(d["dst_color"])}*D'


def report(t, out):
    freq = t['freq']
    verified, checkable = link_secondaries(t)
    passes = [p for p in t['pass_'] if p['kind'] < 2]
    timed = [p for p in passes if ok_time(p)]
    draws_by_pass = collections.defaultdict(list)
    for d in t['draw']:
        draws_by_pass[d['serial'], d['pass_no']].append(d)
    frag_by_pass = {(f['serial'], f['pass_no']): f for f in t['frag']}
    have_samples = any(d['samples'] != 0xffffffffffffffff for d in t['draw'])
    w = out.write
    span = (max(p['end'] for p in timed) - min(p['start'] for p in timed)) / freq if timed else 0
    w(f'trace: pid {t["pid"]}, {len(t["pass_"])} pass records ({len(timed)} timed), {len(t["draw"])} draws, {len(t["shader"])} shaders, '
      f'{len(t["frag"])} autotune entries over {span:.1f} s; lost {t["lost"]}, torn tail {t["torn"]} bytes; per-draw samples '
      f'{"yes" if have_samples else "no"}\n')
    if checkable:
        w(f'draws joined to passes from secondary command buffers: {verified} of {checkable} passes match the autotune\'s draw count\n')
    w('\n')

    w('== passes by size (large: render area of 1 MP or more)\n')
    for name, sel in (('large', lambda p: p['area'] >= LARGE_PX), ('small', lambda p: p['area'] < LARGE_PX)):
        ps = [p for p in timed if sel(p)]
        if not ps:
            continue
        total = sum(p['end'] - p['start'] for p in ps)
        gm = [p for p in ps if p['kind'] == 0]
        fr = [frag_by_pass[p['serial'], p['pass_no']] for p in ps if (p['serial'], p['pass_no']) in frag_by_pass]
        fr = [f for f in fr if f['has_samples']]
        over = sum(f['samples'] for f in fr) / max(1, sum(f['area'] for f in fr)) if fr else 0
        light = sum(1 for p in ps if p['flags'] & 4)
        w(f'{name}: {len(ps)} passes, {us(total, freq) / 1e3:.0f} ms ({us(total, freq) / len(ps):.0f} us each), '
          f'{sum(p["area"] for p in ps) / len(ps) / 1e3:.0f} K px, '
          f'{sum(p["draws"] or len(draws_by_pass[p["serial"], p["pass_no"]]) for p in ps) / len(ps):.1f} draws; '
          f'GMEM {len(gm)} ({sum(p["tiles"] for p in gm) / max(1, len(gm)):.0f} tiles); overdraw {over:.2f} over {len(fr)}'
          + (f'; {light} in system memory by coverage (TU_DW_LIGHT)' if light else '') + '\n')
    w('\n')

    large = [p for p in timed if p['area'] >= LARGE_PX]
    if large and t['draw']:
        w('== draws in large passes, by blend state\n')
        by = collections.defaultdict(lambda: [0, 0, 0, 0])   # draws, samples, full-scissor draws, samples known
        for p in large:
            for d in draws_by_pass[p['serial'], p['pass_no']]:
                b = by[blend_name(d)]
                b[0] += 1
                b[2] += d['sw'] * d['sh'] >= 0.9 * p['area']
                if d['samples'] != 0xffffffffffffffff:
                    b[1] += d['samples']
                    b[3] += 1
        total_draws = sum(b[0] for b in by.values()) or 1
        total_samples = sum(b[1] for b in by.values()) or 1
        area = sum(p['area'] for p in large)
        for name, b in sorted(by.items(), key=lambda kv: -kv[1][0]):
            extra = f', {100 * b[1] / total_samples:.0f} % of samples ({b[1] / area:.2f} per px)' if b[3] else ''
            w(f'{name:30} {b[0]:7} draws ({100 * b[0] / total_draws:.0f} %), {b[2]} with a full-screen scissor{extra}\n')
        w('\n')

        w('== fragment shaders in large passes (by draws, then samples)\n')
        fs = collections.defaultdict(lambda: [0, 0])
        for p in large:
            for d in draws_by_pass[p['serial'], p['pass_no']]:
                fs[d['fs']][0] += 1
                if d['samples'] != 0xffffffffffffffff:
                    fs[d['fs']][1] += d['samples']
        w(f'{"fs id":18} {"draws":>7} {"Msamples":>9} {"instrs":>6} {"nops":>5} {"tex":>4} {"sfu":>4} {"alu":>5} {"regs":>4} '
          f'{"waves":>5} {"sstall":>6} {"systall":>7}\n')
        for sid, (n, smp) in sorted(fs.items(), key=lambda kv: (-kv[1][0], -kv[1][1]))[:25]:
            s = t['shader'].get(sid)
            if not s:
                w(f'{sid:016x} {n:7} {smp / 1e6:9.1f} (no shader record)\n')
                continue
            alu = s['cat'][2] + s['cat'][3]
            w(f'{sid:016x} {n:7} {smp / 1e6:9.1f} {s["instrs"]:6} {s["nops"]:5} {s["cat"][5]:4} {s["cat"][4]:4} {alu:5} '
              f'{s["max_reg"] + 1:4} {s["max_waves"]:5} {s["sstall"]:6} {s["systall"]:7}\n')
        w('\n')

        w('== render targets of large passes\n')
        targets = collections.Counter()
        for p in large:
            for a in p['att'][:min(p['attachments'], 4)]:
                fl = a['flags']
                targets[(FORMATS.get(a['format'], str(a['format'])), a['samples'],
                         ' '.join(n for bit, n in ((1, 'load'), (2, 'store'), (4, 'clear'), (8, 'ubwc'), (16, 'tiled')) if fl & bit))] += 1
        for (fmt, samples, flags), n in targets.most_common(10):
            w(f'{fmt:12} x{samples} {flags:28} {n} passes\n')
        w('\n')

    staged = [p for p in timed if p['flags'] & 2 and p['st_begin'] and p['area'] >= LARGE_PX]
    if staged:
        # By how much of the render area the pass draws (the autotune's samples over its area), where that is known.
        w('== large GMEM passes by stage (TU_DW_TRACE t), per pass in us, by coverage (samples / render area)\n')
        w(f'{"coverage":12} {"passes":>6} {"total":>7} {"begin":>6} {"loads":>6} {"draws":>6} {"stores":>6} {"tiles":>6} {"end":>5} '
          f'{"n tiles":>7} {"draws":>6}\n')
        buckets = (('< 0.25', 0, 0.25), ('0.25-1', 0.25, 1), ('1-3', 1, 3), ('3-8', 3, 8), ('>= 8', 8, 1e9), ('unknown', -1, -1))
        for name, lo, hi in buckets:
            ps = []
            for p in staged:
                f = frag_by_pass.get((p['serial'], p['pass_no']))
                cov = f['samples'] / f['area'] if f and f['has_samples'] and f['area'] else -1
                if (lo <= cov < hi) or (lo == -1 and cov == -1):
                    ps.append(p)
            if not ps:
                continue
            n = len(ps)
            m = lambda v: us(sum(v(p) for p in ps), freq) / n
            between = lambda p: (p['st_last'] - p['st_begin']) - (p['sum_loads'] + p['sum_draws'] + p['sum_stores'])
            w(f'{name:12} {n:6} {m(lambda p: p["end"] - p["start"]):7.0f} {m(lambda p: p["st_begin"] - p["start"]):6.0f} '
              f'{m(lambda p: p["sum_loads"]):6.0f} {m(lambda p: p["sum_draws"]):6.0f} {m(lambda p: p["sum_stores"]):6.0f} '
              f'{m(between):6.0f} {m(lambda p: p["end"] - p["st_last"]):5.0f} {sum(p["tiles"] for p in ps) / n:7.1f} '
              f'{sum(len(draws_by_pass[p["serial"], p["pass_no"]]) for p in ps) / n:6.1f}\n')
        w('\n')
    sys_large = [p for p in timed if p['kind'] == 1 and p['area'] >= LARGE_PX]
    if sys_large:
        w('== large system-memory passes by coverage, per pass in us\n')
        for name, lo, hi in (('< 0.25', 0, 0.25), ('0.25-1', 0.25, 1), ('1-3', 1, 3), ('>= 3', 3, 1e9)):
            ps = []
            for p in sys_large:
                f = frag_by_pass.get((p['serial'], p['pass_no']))
                if f and f['has_samples'] and f['area'] and lo <= f['samples'] / f['area'] < hi:
                    ps.append(p)
            if ps:
                w(f'{name:12} {len(ps):6} {us(sum(p["end"] - p["start"] for p in ps), freq) / len(ps):7.0f} us, '
                  f'{sum(len(draws_by_pass[p["serial"], p["pass_no"]]) for p in ps) / len(ps):5.1f} draws\n')
        w('\n')

    other = collections.Counter()
    for p in t['pass_']:
        if p['kind'] >= 2 and ok_time(p):
            other[KINDS[p['kind']] if p['kind'] < len(KINDS) else str(p['kind'])] += p['end'] - p['start']
    if other:
        w('== other work (TU_DW_TRACE a)\n')
        for k, v in other.most_common():
            w(f'{k:8} {us(v, freq) / 1e3:8.1f} ms\n')


def write_csv(t, table, out):
    link_secondaries(t)
    rows = {'passes': t['pass_'], 'draws': t['draw'], 'frags': t['frag'], 'shaders': list(t['shader'].values())}[table]
    if not rows:
        return
    flat = []
    for r in rows:
        r = dict(r)
        for key in ('att', 'cat'):
            if key in r:
                for i, v in enumerate(r.pop(key)):
                    if isinstance(v, dict):
                        for k2, v2 in v.items():
                            r[f'{key}{i}_{k2}'] = v2
                    else:
                        r[f'{key}{i}'] = v
        flat.append(r)
    writer = csv.DictWriter(out, fieldnames=list(flat[0].keys()))
    writer.writeheader()
    writer.writerows(flat)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('trace', type=Path)
    ap.add_argument('--csv', choices=('passes', 'draws', 'shaders', 'frags'))
    ap.add_argument('--binaries', type=Path)
    a = ap.parse_args()
    t = read(a.trace)
    if a.binaries:
        a.binaries.mkdir(parents=True, exist_ok=True)
        for sid, blob in t['binaries'].items():
            stage = STAGES.get(t['shader'][sid]['stage'], 'x')
            (a.binaries / f'{stage}-{sid:016x}.bin').write_bytes(blob)
        print(f'{len(t["binaries"])} binaries in {a.binaries}')
    elif a.csv:
        write_csv(t, a.csv, sys.stdout)
    else:
        report(t, sys.stdout)


if __name__ == '__main__':
    main()
