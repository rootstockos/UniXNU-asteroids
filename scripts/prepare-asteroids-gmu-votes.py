#!/usr/bin/env python3
"""Generate build/sheng-gmu/ShengGmuVotesSource.h and ShengGmuCommandDbSource.h for the Adreno 810; no device I/O.

The asteroids counterpart of UniXNU-sheng's scripts/prepare-sheng-gmu-votes.py (BSD-2-Clause-Patent). Inputs, all
pinned in scripts/data/a810-gmu-inputs.json:
  - the 3a's own device tree (.cache/asteroids/live.dtb, sha256): its GPU levels are KGSL power-level bins, not an
    OPP table. Each bin (qcom,gpu-pwrlevels-N) lists the SKU codes it applies to (qcom,sku-codes) and per level the
    frequency (qcom,gpu-freq), the RPMh corner (qcom,level, mainline's opp-level) and a DDR bus index
    (qcom,bus-freq-ddr7 into qcom,bus-table-ddr7, in kB/s, mainline's opp-peak-kBps). The 3a is LPDDR4X, DDR type 7
    (board.toml [gpu] highest_bank_bit). The GMU levels are qcom,gmu-freq-table {Hz, corner} pairs; the cmd-db
    reservation is /reserved-memory/...cmd-db;
  - torvalds/linux: the A810 catalog BCM policy (SH0, MC0, ACV) and the A8xx fixed-BCM rule of
    a6xx_gmu_rpmh_bw_votes_init (perfmode on every valid bandwidth for any non-A7xx GPU).
Both bins are generated in device-tree order, because the builder selects one as KGSL's adreno_of_get_pwrlevels does
(the 3a's own graphics-kernel adreno.c): the first bin, in that order, listing the SoC code or 0, and 0 matches any
code. On the 3a qcom,gpu-pwrlevels-1 (codes 0x2, 0x0) comes first, so every 3a takes it. No command-db value is in the output: rails and BCM aux data are read from the reservation at boot.
Two steps, as uke's prepare-uke-gmu.py: with the pinned DTB and Linux sources this derives the facts into
scripts/data/a810-gmu-votes.json (committed; the DTB itself is not) and writes the headers; --from-data writes the same
headers from that file alone, which is what the build tree and CI run. --check derives again and compares both.
Writes the headers and votes-source-report.json."""
import hashlib, json, re, struct, sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
PINS = json.loads((ROOT / 'scripts/data/a810-gmu-inputs.json').read_text())
DTB = ROOT / '.cache/asteroids/live.dtb'
DATA = ROOT / 'scripts/data/a810-gmu-votes.json'
MSM = ROOT / '.cache/linux-a8xx/drivers/gpu/drm/msm'
OUT = ROOT / 'build/sheng-gmu'
GPU = '/soc/qcom,kgsl-3d0@3d00000'


def fail(message):
    raise SystemExit('prepare-asteroids-gmu-votes: ' + message)


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def parse_fdt(blob):
    """Flattened device tree -> {path: {property: bytes}} (FDT v17 structure block)."""
    magic, total, off_struct, off_strings = struct.unpack_from('>4I', blob, 0)
    if magic != 0xd00dfeed or total != len(blob):
        fail('live.dtb is not a complete flattened device tree')
    nodes, stack, at = {}, [], off_struct
    while True:
        token = struct.unpack_from('>I', blob, at)[0]; at += 4
        if token == 1:
            end = blob.index(b'\0', at); name = blob[at:end].decode(); at = (end + 4) & ~3
            stack.append(name)
            path = '/' + '/'.join(s for s in stack if s)
            nodes.setdefault(path or '/', {})
        elif token == 2:
            stack.pop()
        elif token == 3:
            length, name_off = struct.unpack_from('>II', blob, at); at += 8
            name = blob[off_strings + name_off:blob.index(b'\0', off_strings + name_off)].decode()
            path = '/' + '/'.join(s for s in stack if s)
            nodes[path or '/'][name] = blob[at:at + length]; at = (at + length + 3) & ~3
        elif token == 4:
            continue
        elif token == 9:
            return nodes
        else:
            fail('bad FDT token')


def cells(value):
    if len(value) % 4:
        fail('property is not a cell array')
    return list(struct.unpack('>%dI' % (len(value) // 4), value))


def one(node, name):
    value = cells(node[name])
    if len(value) != 1:
        fail('expected one cell in ' + name)
    return value[0]


def derive():
    if not DTB.exists() or sha(DTB) != PINS['device_tree']['sha256']:
        fail('.cache/asteroids/live.dtb is missing or differs from its pin')
    for name in ['drivers/gpu/drm/msm/adreno/a6xx_catalog.c', 'drivers/gpu/drm/msm/adreno/a6xx_gmu.c']:
        if sha(ROOT / '.cache/linux-a8xx' / name) != PINS['linux']['files'][name]:
            fail(name + ' does not match its pin')
    nodes = parse_fdt(DTB.read_bytes())
    gpu = nodes.get(GPU)
    if gpu is None or b'qcom,adreno-gpu-gen8-3-0\0' not in gpu.get('compatible', b''):
        fail('no gen8-3-0 GPU node at ' + GPU)
    table = cells(gpu['qcom,bus-table-ddr7'])
    if table[0] != 0 or any(table[i] <= table[i - 1] for i in range(1, len(table))):
        fail('qcom,bus-table-ddr7 is not off then ascending')

    bins = []
    root = GPU + '/qcom,gpu-pwrlevel-bins'
    # Dict order is FDT order (parse_fdt inserts nodes as it meets them); KGSL walks the bins in it.
    for path in [p for p in nodes if re.fullmatch(re.escape(root) + r'/qcom,gpu-pwrlevels-\d+', p)]:
        if 'qcom,speed-bin' in nodes[path]:
            fail('bin with qcom,speed-bin; the builder handles SoC codes only')
        skus = cells(nodes[path]['qcom,sku-codes'])
        levels = []
        for child in sorted(p for p in nodes if re.fullmatch(re.escape(path) + r'/qcom,gpu-pwrlevel@[0-9a-f]+', p)):
            node = nodes[child]
            known = {'reg', 'qcom,gpu-freq', 'qcom,level', 'qcom,acd-level', 'phandle'} | {
                'qcom,bus-%s-ddr%d' % (k, t) for k in ('freq', 'min', 'max') for t in (7, 8)}
            if set(node) - known:
                fail('unhandled power-level property in %s: %s' % (child, sorted(set(node) - known)))
            hz, level, bus = one(node, 'qcom,gpu-freq'), one(node, 'qcom,level'), one(node, 'qcom,bus-freq-ddr7')
            if not hz or hz % 1000 or not 0 < level <= 0xffff or bus >= len(table) or not table[bus]:
                fail('unrepresentable level in ' + child)
            levels.append(dict(hz=hz, level=level, kbps=table[bus], acd_present=0, acd=0))
        levels.sort(key=lambda e: e['hz'])
        if not levels or len(levels) + 1 > 16 or len({e['hz'] for e in levels}) != len(levels):
            fail('invalid level count or duplicate frequency in ' + path)
        if any(levels[i]['level'] < levels[i - 1]['level'] for i in range(1, len(levels))):
            fail('nonmonotonic corners in ' + path)
        levels = [dict(hz=0, level=0, kbps=0, acd_present=0, acd=0)] + levels
        bins.append(dict(node=path, skus=skus, levels=levels, bandwidths=sorted({e['kbps'] for e in levels})))
    if not bins:
        fail('no power-level bins')

    gmu_nodes = [p for p, n in nodes.items() if b'qcom,gen8-gmu\0' in n.get('compatible', b'')]
    if len(gmu_nodes) != 1:
        fail('expected one qcom,gen8-gmu node')
    pairs = cells(nodes[gmu_nodes[0]]['qcom,gmu-freq-table'])
    gmu = [dict(hz=0, level=0, kbps=0, acd_present=0, acd=0)] + [
        dict(hz=pairs[i], level=pairs[i + 1], kbps=0, acd_present=0, acd=0) for i in range(0, len(pairs), 2)]
    if len(gmu) > 4 or any(gmu[i]['hz'] <= gmu[i - 1]['hz'] or gmu[i]['level'] < gmu[i - 1]['level']
                           for i in range(1, len(gmu))):
        fail('invalid qcom,gmu-freq-table')

    # A810 catalog BCMs and the non-A7xx fixed-BCM rule.
    catalog = (MSM / 'adreno/a6xx_catalog.c').read_text()
    entry = re.search(r'ADRENO_CHIP_IDS\(0x44010000\)(.*?)\n\t\}', catalog, re.S).group(1)
    if 'ADRENO_QUIRK_ACD' in entry:
        fail('the A810 entry gained ACD; extend the builder')
    found = re.findall(r'\{\s*\.name = "([A-Z0-9]+)",([^{}]+)\}', entry)
    if [n for n, _ in found] != ['SH0', 'MC0', 'ACV']:
        fail('A810 BCM order differs')
    gmu_c = (MSM / 'adreno/a6xx_gmu.c').read_text()
    if ('\t\t\t\t/* GMU on A6xx votes perfmode on all valid bandwidth */\n'
            '\t\t\t\tif (!adreno_is_a7xx(adreno_gpu) ||\n'
            '\t\t\t\t    (bcm->perfmode_bw && bw >= bcm->perfmode_bw))\n') not in gmu_c:
        fail('the fixed-BCM perfmode rule changed')
    policies = []
    for name, body in found:
        if name != 'ACV':
            m = re.fullmatch(r'\s*\.buswidth = (\d+)\s*', body)
            if not m:
                fail('unrecognised proportional BCM ' + name)
            policies.append(dict(name=name, buswidth=int(m.group(1)), fixed=0, perfmode=0, perfmode_kbps=0, all=0))
        else:
            m = re.fullmatch(r'\s*\.fixed = true,\s*\.perfmode = BIT\((\d+)\),\s*\.perfmode_bw = (\d+),\s*', body)
            if not m:
                fail('unrecognised fixed BCM ' + name)
            policies.append(dict(name=name, buswidth=0, fixed=1, perfmode=1 << int(m.group(1)),
                                 perfmode_kbps=int(m.group(2)), all=1))

    db = [p for p, n in nodes.items() if p.startswith('/reserved-memory/') and n.get('compatible') == b'qcom,cmd-db\0']
    if len(db) != 1 or 'no-map' not in nodes[db[0]]:
        fail('expected one no-map qcom,cmd-db reservation')
    reg = cells(nodes[db[0]]['reg'])
    db_base, db_bytes = (reg[0] << 32) | reg[1], (reg[2] << 32) | reg[3]
    if not 144 <= db_bytes <= 1048576:
        fail('unsupported command-db reservation size')

    return dict(dtb_sha256=sha(DTB), linux_revision=PINS['linux']['commit'], bins=bins, gmu_opps=gmu, bcms=policies,
                command_db=dict(node=db[0], base=hex(db_base), bytes=db_bytes))


def render(facts):
    bins, gmu, policies = facts['bins'], facts['gmu_opps'], facts['bcms']
    db = [facts['command_db']['node']]
    db_base, db_bytes = int(facts['command_db']['base'], 16), facts['command_db']['bytes']
    fields = ['hz', 'level', 'kbps', 'acd_present', 'acd']
    lines = ['/* Generated by scripts/prepare-asteroids-gmu-votes.py from the pinned 3a device tree and torvalds/linux %s;'
             % facts['linux_revision'][:12], ' * contains no command-db values. Do not commit; '
             'scripts/data/a810-gmu-votes.json holds its inputs. */',
             '#include <stdint.h>', '#define SHENG_GMU_VOTES_CHIP_ID 0x44010000u',
             'typedef struct {uint64_t hz;uint32_t level,kbps,acd_present,acd;} ShengGmuOpp;',
             'typedef struct {unsigned sku_count;uint32_t skus[16];unsigned opp_count;ShengGmuOpp opps[16];',
             '    unsigned bandwidth_count;uint32_t bandwidths[16];} ShengGmuGpuBin;',
             '/* qcom,gpu-pwrlevel-bins in device-tree order: the first listing the SoC code or 0 (any) is used. */',
             '#define SHENG_GPU_BINS %du' % len(bins), 'static const ShengGmuGpuBin sheng_gpu_bins[%d]={' % len(bins)]
    for b in bins:
        lines.append('    {%d,{%s},%d,{%s},%d,{%s}},   /* %s */' % (
            len(b['skus']), ','.join(hex(s) for s in b['skus']), len(b['levels']),
            ','.join('{' + ','.join(str(e[k]) for k in fields) + '}' for e in b['levels']),
            len(b['bandwidths']), ','.join(map(str, b['bandwidths'])), b['node'].rsplit('/', 1)[1]))
    lines += ['};', '#define SHENG_GMU_OPPS %du' % len(gmu), 'static const ShengGmuOpp sheng_gmu_opps[]={',
              '\n'.join('{' + ','.join(str(e[k]) for k in fields) + '},' for e in gmu), '};',
              '/* A810 catalog SH0, MC0, ACV; ACV votes its perfmode on every valid bandwidth on A8xx. */',
              'static const AdrenoHfiBcm sheng_bcm_policy[3]={']
    lines += ['{.buswidth=%d,.fixed=%d,.perfmode=%d,.perfmode_kbps=%d,.perfmode_all=%d},' % (
        p['buswidth'], p['fixed'], p['perfmode'], p['perfmode_kbps'], p['all']) for p in policies]
    lines.append('};')
    votes = '\n'.join(lines) + '\n'
    db_text = ('/* Pinned 3a DT reservation (%s), not observed hardware memory. */\n'
               '#define SHENG_GMU_COMMAND_DB_BASE UINT64_C(%s)\n#define SHENG_GMU_COMMAND_DB_BYTES %du\n'
               % (db[0], hex(db_base), db_bytes))
    return {OUT / 'ShengGmuVotesSource.h': votes, OUT / 'ShengGmuCommandDbSource.h': db_text}


def main():
    args = sys.argv[1:]
    if '--from-data' in args:
        facts = json.loads(DATA.read_text())
    else:
        facts = derive()
        data = json.dumps(facts, indent=2) + '\n'
        if '--check' in args:
            if not DATA.exists() or DATA.read_text() != data:
                fail(DATA.name + ' is stale')
        else:
            DATA.write_text(data)
    targets = render(facts)
    if '--check' in args:
        for path, text in targets.items():
            if not path.exists() or path.read_text() != text:
                fail(path.name + ' is stale')
        print('PASS: A810 vote inputs match the pinned sources')
        return
    OUT.mkdir(parents=True, exist_ok=True)
    for path, text in targets.items():
        path.write_text(text)
    bins, gmu = facts['bins'], facts['gmu_opps']
    db_base, db_bytes = int(facts['command_db']['base'], 16), facts['command_db']['bytes']
    report = dict(facts, ddr_type=7, selection='first bin in DT order listing the SoC code or 0 (KGSL)',
                  selected_for_any_soc_code=next((b['node'] for b in bins if 0 in b['skus']), None),
                  actual_command_db_available=False, hardware_ready=False)
    (OUT / 'votes-source-report.json').write_text(json.dumps(report, indent=2) + '\n')
    print('PASS: %d GPU bins (%s), %d GMU levels, A810 BCM policy, cmd-db 0x%x/0x%x; rails and BCM aux read at boot'
          % (len(bins), ', '.join('%d levels' % len(b['levels']) for b in bins), len(gmu), db_base, db_bytes))


main()
