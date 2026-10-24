#!/usr/bin/env python3
"""Generate build/sheng-gmu/ShengGmuSource.h for the Adreno 810 (gen8) GMU cold start; no device I/O.

The asteroids counterpart of UniXNU-sheng's scripts/prepare-sheng-gmu.py (BSD-2-Clause-Patent), which derives the A740
inputs. Every value comes from a pinned input:
  - torvalds/linux (scripts/data/a810-gmu-inputs.json, sha256 per file): register offsets from a6xx_gmu.xml/a6xx.xml
    with the variant the kernel's header generator gives the C name the gen8 branch of a6xx_gmu.c uses (REG_A8XX_X:
    variants starting at A8XX; REG_A6XX_X: no variants or starting at A6XX), each required to cover A8XX; the gen8
    sequence facts from the a8xx branches of a6xx_gmu.c, each bound to its exact source line; the GBIF CX list and the
    GMU chip ID from the A810 entry of a6xx_catalog.c;
  - board.toml [gpu]: the 3a's DT windows (gmu, rscc, cx_misc, kgsl_3d0_reg_memory) and the firmware pin;
  - the pinned gen80300_gmu.bin (sha256 and size checked), whose block records are counted.
The 3a has no gmu_pdc window: on A8xx the RPMh/PDC setup lives in AOP (a6xx_gmu_rpmh_init returns early), so the PDC
window is absent (size 0) and the RSC setup list is empty.

--fetch makes the sparse checkout in .cache/linux-a8xx. Writes the header, ShengGmuFirmware.h and ShengSqeFirmware.h (the pinned blobs as C
arrays, for the kernel build only: build/ is ignored, never commit it) and source-report.json; --check compares.
Inside the UniXNU build tree (kits/asteroids links to the device package) board.toml and .cache are read from the
device repository, where the CI bootstrap fetched them; the headers still go to the tree's build/sheng-gmu."""
import hashlib, json, re, struct, subprocess, sys, tomllib
import xml.etree.ElementTree as ET
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
# The device repository: ROOT itself, or the target of the build tree's kits/asteroids link.
DEVICE = ROOT if (ROOT / 'board.toml').exists() else (ROOT / 'kits/asteroids').resolve().parent
PIN = json.loads((ROOT / 'scripts/data/a810-gmu-inputs.json').read_text())['linux']
SRC = DEVICE / '.cache/linux-a8xx'
MSM = SRC / 'drivers/gpu/drm/msm'
FIRMWARE = DEVICE / '.cache/firmware/asteroids-gpu/gen80300_gmu.bin'
SQE = DEVICE / '.cache/firmware/asteroids-gpu/gen80300_sqe.fw'
ZAP = DEVICE / '.cache/firmware/asteroids-gpu/gen80300_zap.mbn'
OUT = ROOT / 'build/sheng-gmu'
NS = '{http://nouveau.freedesktop.org/}'
ORDER = ['A2XX', 'A3XX', 'A4XX', 'A5XX', 'A6XX', 'A7XX', 'A8XX']


def fail(message):
    raise SystemExit('prepare-asteroids-gmu: ' + message)


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def fetch():
    if not (SRC / '.git').exists():
        SRC.mkdir(parents=True, exist_ok=True)
        subprocess.run(['git', 'init', '-q', str(SRC)], check=True)
        subprocess.run(['git', '-C', str(SRC), 'remote', 'add', 'origin', PIN['repository']], check=True)
    subprocess.run(['git', '-C', str(SRC), 'sparse-checkout', 'set', '--no-cone',
                    '/drivers/gpu/drm/msm/adreno/', '/drivers/gpu/drm/msm/registers/adreno/'], check=True)
    # The fetch takes all the commit's trees (minutes on a hosted runner) and the checkout the sparse paths' blobs.
    print('fetching', PIN['repository'], PIN['commit'], 'trees only', flush=True)
    subprocess.run(['git', '-C', str(SRC), 'fetch', '-q', '--depth', '1', '--filter=blob:none', 'origin',
                    PIN['commit']], check=True)
    print('checking out the adreno sources', flush=True)
    subprocess.run(['git', '-C', str(SRC), 'checkout', '-q', 'FETCH_HEAD'], check=True)


def covers(variants):
    if variants is None:
        return True
    for part in variants.split(','):
        part = part.strip()
        if '-' in part:
            left, right = part.split('-', 1)
            low = ORDER.index(left) if left else 0
            high = ORDER.index(right) if right else len(ORDER) - 1
            if low <= ORDER.index('A8XX') <= high:
                return True
        elif part == 'A8XX':
            return True
    return False


class Registers:
    def __init__(self):
        self.regs = {}
        self.bitsets = {}
        for name in ['a6xx.xml', 'a6xx_gmu.xml']:
            root = ET.parse(MSM / 'registers/adreno' / name).getroot()
            for bitset in root.iter(NS + 'bitset'):
                self.bitsets.setdefault(bitset.get('name'), bitset)
            for domain in root.findall(NS + 'domain'):
                for element in domain:
                    if element.tag.replace(NS, '') not in ('reg32', 'reg64'):
                        continue
                    variants = element.get('variants')
                    prefix = re.split(r'[-,]', variants)[0] if variants else domain.get('name')
                    self.regs.setdefault(element.get('name'), []).append(
                        (prefix, variants, int(element.get('offset'), 0), element))

    def offset(self, token):
        """REG_A6XX_X / REG_A8XX_X as the header generator names them, covering A8XX."""
        m = re.fullmatch(r'REG_(A[678]XX)_([A-Z0-9_]+)', token)
        hits = [r for r in self.regs.get(m.group(2), []) if r[0] == m.group(1)] if m else []
        if len({r[2] for r in hits}) != 1:
            fail(token + ' is not one definition')
        if not covers(hits[0][1]):
            fail(token + ' does not cover A8XX')
        return hits[0][2], hits[0][3]

    def bit(self, token, field):
        _, element = self.offset(token)
        sources = [element] + ([self.bitsets[element.get('type')]] if element.get('type') in self.bitsets else [])
        for source in sources:
            for f in source.findall(NS + 'bitfield'):
                if f.get('name') == field and covers(f.get('variants')) and f.get('pos') is not None:
                    return 1 << int(f.get('pos'))
        fail('no bit %s in %s' % (field, token))


def function(source, name):
    start = source.index('\n' + name)
    depth, i = 0, source.index('{', start)
    while True:
        depth += {'{': 1, '}': -1}.get(source[i], 0)
        i += 1
        if not depth:
            return source[start:i]


def require(text, needle, what):
    if needle not in text:
        fail('the pinned source no longer has ' + what + ': ' + needle)


def blob_header(name, digest, symbol, blob):
    return ('/* %s (sha256 %s), from board.toml [gpu] firmware_repository at firmware_commit.\n'
            ' * Generated by scripts/prepare-asteroids-gmu.py for the kernel build. Qualcomm firmware: never commit. */\n'
            '#include <stdint.h>\n'
            'static const uint8_t %s[%d] __attribute__((aligned(16)))={\n' % (name, digest, symbol, len(blob)) +
            '\n'.join(','.join(str(b) for b in blob[i:i + 32]) + ',' for i in range(0, len(blob), 32)) + '\n};\n')


def main():
    if '--fetch' in sys.argv[1:]:
        fetch()
    for name, digest in PIN['files'].items():
        if not (SRC / name).exists():
            fail('missing ' + name + '; run with --fetch')
        if sha(SRC / name) != digest:
            fail(name + ' does not match its pin')
    board = tomllib.loads((DEVICE / 'board.toml').read_text())['gpu']
    pin = board['firmware_pins']['gen80300_gmu.bin']
    if not FIRMWARE.exists() or FIRMWARE.stat().st_size != pin['bytes'] or sha(FIRMWARE) != pin['sha256']:
        fail('gen80300_gmu.bin is missing or differs from its board.toml pin')
    sqe_pin = board['firmware_pins']['gen80300_sqe.fw']
    if not SQE.exists() or SQE.stat().st_size != sqe_pin['bytes'] or sha(SQE) != sqe_pin['sha256']:
        fail('gen80300_sqe.fw is missing or differs from its board.toml pin')
    zap_pin = board['firmware_pins']['gen80300_zap.mbn']
    if not ZAP.exists() or ZAP.stat().st_size != zap_pin['bytes'] or sha(ZAP) != zap_pin['sha256']:
        fail('gen80300_zap.mbn is missing or differs from its board.toml pin')
    gmu = (MSM / 'adreno/a6xx_gmu.c').read_text()
    catalog = (MSM / 'adreno/a6xx_catalog.c').read_text()
    r = Registers()

    # The gen8 sequence facts, each from its a8xx branch.
    rpmh_init = function(gmu, 'static void a6xx_gmu_rpmh_init(')
    require(rpmh_init, '/* On A8x and above, RPMH/PDC configurations are entirely configured in AOP */\n'
            '\tif (adreno_is_a8xx(adreno_gpu))\n\t\treturn;', 'the A8xx RPMh/PDC early return')
    fw_start = function(gmu, 'static int a6xx_gmu_fw_start(')
    require(fw_start, '\t} else if (adreno_is_a8xx(adreno_gpu)) {\n'
            '\t\tgmu_write(gmu, REG_A8XX_GPU_GMU_CX_GMU_CX_FALNEXT_INTF, 1);\n'
            '\t\tgmu_write(gmu, REG_A8XX_GPU_GMU_CX_GMU_CX_FAL_INTF, 1);', 'the A8xx FAL10 veto')
    require(fw_start, '\tif (adreno_is_a7xx(adreno_gpu))\n'
            '\t\ta6xx_cx_misc_write(a6xx_gpu, REG_A7XX_CX_MISC_TCM_RET_CNTL, 1);\n'
            '\telse if (!adreno_is_a8xx(adreno_gpu))', 'the TCM retention skip on A8xx')
    fence = re.search(r'if \(adreno_is_a8xx\(adreno_gpu\)\) \{\n\t\tfence_range_upper = (0x[0-9a-f]+);\n'
                      r'\t\tfence_range_lower = (0x[0-9a-f]+);', fw_start)
    if not fence:
        fail('the A8xx fence range is gone')
    upper, lower = int(fence.group(1), 16), int(fence.group(2), 16)
    require(fw_start, 'BIT(31) |\n\t\t  FIELD_PREP(GENMASK(30, 18), fence_range_upper) |\n'
            '\t\t  FIELD_PREP(GENMASK(17, 0), fence_range_lower));', 'the fence range encoding')
    fence_word = (1 << 31) | ((upper & 0x1fff) << 18) | (lower & 0x3ffff)
    require(fw_start, '\tif (adreno_is_a8xx(adreno_gpu)) {\n\t\tgmu_write(gmu, REG_A8XX_GMU_GENERAL_10, chipid);\n'
            '\t\tgmu_write(gmu, REG_A8XX_GMU_GENERAL_8,\n\t\t\t  (gmu->log.iova & GENMASK(31, 12)) |\n'
            '\t\t\t  ((gmu->log.size / SZ_4K - 1) & GENMASK(7, 0)));', 'the A8xx chip ID and log registers')
    gbif_tail = re.search(r'\tif \(adreno_is_a8xx\(adreno_gpu\)\) \{\n\t\tgpu_write\(gpu, (REG_A6XX_GBIF_CX_CONFIG), '
                          r'(0x[0-9a-f]+)\);\n\t\tgmu_write\(gmu, (REG_A6XX_GMU_MRC_GBIF_QOS_CTRL), (0x[0-9a-f]+)\);',
                          fw_start)
    if not gbif_tail:
        fail('the A8xx GBIF CX / MRC QoS writes are gone')
    require(fw_start, 'for (int i = 0; (gbif_cx && gbif_cx[i].offset); i++)\n'
            '\t\tgpu_write(gpu, gbif_cx[i].offset, gbif_cx[i].value);', 'the GBIF CX list write')
    gmu_start = function(gmu, 'static int a6xx_gmu_start(')
    require(gmu_start, 'if (adreno_is_a8xx(adreno_gpu))\n\t\tgmu_write(gmu, REG_A8XX_GMU_GENERAL_9, 0);',
            'the A8xx log write index')
    require(gmu_start, 'if (val <= 0x20010004) {', 'the firmware version ready rule')
    for token in ['REG_A6XX_GMU_CM3_FW_INIT_RESULT, 0', 'REG_A6XX_GMU_CM3_BOOT_CONFIG, 0x02',
                  'REG_A6XX_GMU_HFI_QTBL_INFO, 1', 'REG_A6XX_GMU_CM3_CFG, 0x4052']:
        require(fw_start, token, 'the shared CM3/HFI configuration')
    alloc = gmu[gmu.index('gmu->dummy.size = SZ_4K;'):gmu.index('"hfi");')]
    for needle in ['adreno_is_a8xx(adreno_gpu)) {\n\t\tret = a6xx_gmu_memory_alloc(gmu, &gmu->debug, SZ_4K * 7,\n'
                   '\t\t\t\t\t    0x60400000, "debug");', '0x60000000, "dummy");',
                   'adreno_is_a8xx(adreno_gpu)) {\n\t\tret = a6xx_gmu_memory_alloc(gmu, &gmu->icache,\n'
                   '\t\t\tSZ_16M - SZ_16K, 0x04000, "icache");']:
        require(alloc, needle, 'the A8xx GMU memory layout')
    require(function(gmu, 'static int a6xx_gmu_fw_load('), 'adreno_is_a8xx(adreno_gpu))\n\t\tdtcm_base = 0x10004000;',
            'the A8xx DTCM base')

    # A810 catalog entry: GMU chip ID and the GBIF CX list.
    entry = re.search(r'ADRENO_CHIP_IDS\(0x44010000\)(.*?)\n\t\}', catalog, re.S).group(1)
    fw_chip = int(re.search(r'\.gmu_chipid = (0x[0-9a-f]+)', entry).group(1), 16)
    gbif_name = re.search(r'\.gbif_cx = ([a-z0-9_]+),', entry).group(1)
    block = re.search(gbif_name + r'\[\] = \{(.*?)\n\};', catalog, re.S).group(1)
    gbif = [(t, int(v, 16)) for t, v in re.findall(r'\{\s*(REG_[A-Z0-9_]+),\s*(0x[0-9a-f]+)\s*\}', block)]
    if not gbif or len(gbif) != block.count('REG_'):
        fail('unparsed ' + gbif_name)
    gbif.append((gbif_tail.group(1), int(gbif_tail.group(2), 16)))

    # Registers, by the C names the gen8 branches use.
    tokens = {
        'REG_GMU_CM3_ITCM_START': 'REG_A6XX_GMU_CM3_ITCM_START', 'REG_GMU_CM3_DTCM_START': 'REG_A6XX_GMU_CM3_DTCM_START',
        'REG_GMU_CM3_SYSRESET': 'REG_A6XX_GMU_CM3_SYSRESET', 'REG_GMU_CM3_BOOT_CONFIG': 'REG_A6XX_GMU_CM3_BOOT_CONFIG',
        'REG_GMU_CM3_FW_INIT_RESULT': 'REG_A6XX_GMU_CM3_FW_INIT_RESULT', 'REG_GMU_CM3_CFG': 'REG_A6XX_GMU_CM3_CFG',
        'REG_GMU_HFI_QTBL_ADDR': 'REG_A6XX_GMU_HFI_QTBL_ADDR', 'REG_GMU_HFI_QTBL_INFO': 'REG_A6XX_GMU_HFI_QTBL_INFO',
        'REG_GMU_HFI_CTRL_INIT': 'REG_A6XX_GMU_HFI_CTRL_INIT', 'REG_GMU_HFI_CTRL_STATUS': 'REG_A6XX_GMU_HFI_CTRL_STATUS',
        'REG_GMU_HOST2GMU_INTR_SET': 'REG_A6XX_GMU_HOST2GMU_INTR_SET',
        'REG_GMU_SYS_BUS_CONFIG': 'REG_A6XX_GMU_SYS_BUS_CONFIG', 'REG_GMU_ICACHE_CONFIG': 'REG_A6XX_GMU_ICACHE_CONFIG',
        'REG_GMU_DCACHE_CONFIG': 'REG_A6XX_GMU_DCACHE_CONFIG', 'REG_GMU_AHB_FENCE_RANGE_0': 'REG_A6XX_GMU_AHB_FENCE_RANGE_0',
        'REG_GMU_GMU2HOST_INTR_INFO': 'REG_A6XX_GMU_GMU2HOST_INTR_INFO',
        'REG_GMU_AO_HOST_INTERRUPT_STATUS': 'REG_A6XX_GMU_AO_HOST_INTERRUPT_STATUS',
        'REG_GPU_GMU_CX_GMU_CX_FALNEXT_INTF': 'REG_A8XX_GPU_GMU_CX_GMU_CX_FALNEXT_INTF',
        'REG_GPU_GMU_CX_GMU_CX_FAL_INTF': 'REG_A8XX_GPU_GMU_CX_GMU_CX_FAL_INTF',
        'REG_GMU_GENERAL_8': 'REG_A8XX_GMU_GENERAL_8', 'REG_GMU_GENERAL_9': 'REG_A8XX_GMU_GENERAL_9',
        'REG_GMU_GENERAL_10': 'REG_A8XX_GMU_GENERAL_10', 'REG_GMU_MRC_GBIF_QOS_CTRL': gbif_tail.group(3),
        'REG_RSCC_SEQ_BUSY_DRV0': 'REG_A6XX_RSCC_SEQ_BUSY_DRV0',
    }
    for symbol, token in tokens.items():
        if token.startswith('REG_A8XX') and token not in gmu:
            fail(token + ' is not used by the pinned a6xx_gmu.c')
    regs = {symbol: r.offset(token)[0] for symbol, token in tokens.items()}
    fatal = sum(r.bit('REG_A6XX_GMU_AO_HOST_INTERRUPT_STATUS', b) for b in ['WDOG_BITE', 'FENCE_ERR', 'HOST_AHB_BUS_ERROR'])
    cm3 = r.bit('REG_A6XX_GMU_GMU2HOST_INTR_INFO', 'CM3_FAULT')
    gbif_writes = [(r.offset(t)[0] * 4, v) for t, v in gbif]

    # Windows in ShengGmu order: core, RSCC, PDC (absent on A8xx), GPU, CX misc.
    number = lambda key: int(board[key], 0)
    gpu_base, gpu_bytes = number('gpu_base'), number('gpu_size')
    windows = [(number('gmu_base'), number('gmu_size')), (number('rscc_base'), number('rscc_size')), (0, 0),
               (gpu_base, gpu_bytes), (number('cx_misc_base'), number('cx_misc_size'))]
    # a6xx_gmu_init on A8xx: "RSCC lives at GPU base + 0x50000".
    require(gmu, 'gmu->rscc = gpu->mmio + 0x50000;', 'the A8xx RSCC location')
    if windows[1][0] != gpu_base + 0x50000:
        fail('the board RSCC window is not at GPU base + 0x50000')
    delta = windows[0][0] - gpu_base
    for symbol, word in regs.items():
        if symbol.startswith('REG_RSCC'):
            at, size = word * 4, windows[1][1]
        else:
            at, size = word * 4 - delta, windows[0][1]
        if not 0 <= at <= size - 4:
            fail('%s lands outside its window' % symbol)
    for at, _ in gbif_writes:
        if not 0 <= at <= gpu_bytes - 4:
            fail('a GBIF CX write lands outside the GPU window')

    # Firmware block records: {addr, size, type, value, data[size]} (block_header in a6xx_gmu.c).
    require(gmu, 'struct block_header {\n\tu32 addr;\n\tu32 size;\n\tu32 type;\n\tu32 value;\n\tu32 data[];\n};',
            'the firmware block layout')
    data, at, blocks = FIRMWARE.read_bytes(), 0, 0
    while at < len(data):
        if at + 16 > len(data):
            fail('truncated firmware record')
        _, size, _, _ = struct.unpack_from('<4I', data, at)
        at += 16 + size
        blocks += 1
    if at != len(data):
        fail('firmware records overrun the image')

    lines = ['/* Generated by scripts/prepare-asteroids-gmu.py from torvalds/linux %s, board.toml [gpu] and the pinned'
             % PIN['commit'][:12],
             ' * gen80300_gmu.bin. Do not commit. */', '#include <stdint.h>', '#include <stddef.h>',
             '#define SHENG_GMU_GENERATION 8u']
    lines += ['#define %s UINT32_C(%s)' % (symbol, hex(word)) for symbol, word in regs.items()]
    lines += ['#define SHENG_GMU_CHIP_ID UINT32_C(0x44010000)', '#define SHENG_GMU_FW_CHIP_ID UINT32_C(%s)' % hex(fw_chip),
              '#define SHENG_GMU_CORE_GPU_DELTA %su' % hex(delta),
              '#define SHENG_GMU_FIRMWARE_BYTES %du' % pin['bytes'], '#define SHENG_GMU_FIRMWARE_BLOCKS %du' % blocks,
              '#define SHENG_GMU_AO_FATAL_MASK %su' % hex(fatal), '#define SHENG_GMU_CM3_FATAL_MASK %su' % hex(cm3),
              '#define SHENG_GMU_AHB_FENCE_RANGE UINT32_C(%s)' % hex(fence_word),
              '#define SHENG_GMU_MRC_GBIF_QOS UINT32_C(%s)' % gbif_tail.group(4),
              'static const uint8_t sheng_gmu_firmware_sha256[32]={%s};' %
              ','.join('0x' + pin['sha256'][i:i + 2] for i in range(0, 64, 2)),
              'static const uint64_t sheng_gmu_window_physical[5]={%s};' % ','.join(hex(b) for b, _ in windows),
              'static const size_t sheng_gmu_window_bytes[5]={%s};' % ','.join(hex(s) for _, s in windows),
              'typedef struct {unsigned window;size_t byte_offset;uint32_t value;} ShengGmuSetupWrite;',
              '/* A8xx: RPMh/PDC setup is done by AOP, so no RSC/PDC setup writes. */',
              '#define SHENG_GMU_RSC_SETUP_COUNT 0u',
              '/* %s then GBIF_CX_CONFIG again (a6xx_gmu_fw_start): {GPU-window byte offset, value}. */' % gbif_name,
              '#define SHENG_GMU_GBIF_CX_COUNT %du' % len(gbif_writes),
              'static const ShengGmuSetupWrite sheng_gmu_gbif_cx[%d]={' % len(gbif_writes)]
    lines += ['{3,%s,%s},' % (hex(at), hex(v)) for at, v in gbif_writes]
    lines.append('};')
    text = '\n'.join(lines) + '\n'
    header = OUT / 'ShengGmuSource.h'
    firmware_text = blob_header('gen80300_gmu.bin', pin['sha256'], 'sheng_gmu_firmware', FIRMWARE.read_bytes())
    # The SQE microcode for modes 12-14: the whole file; the probe copies it without its first dword into the GPU's SQE
    # buffer, as adreno_fw_create_bo does.
    sqe_text = blob_header('gen80300_sqe.fw', sqe_pin['sha256'], 'sheng_sqe_firmware', SQE.read_bytes())
    # The zap shader for mode 15: the whole signed image; the probe splits it into metadata and the load segment.
    zap_text = blob_header('gen80300_zap.mbn', zap_pin['sha256'], 'sheng_zap_firmware', ZAP.read_bytes())
    firmware_header = OUT / 'ShengGmuFirmware.h'
    sqe_header = OUT / 'ShengSqeFirmware.h'
    zap_header = OUT / 'ShengZapFirmware.h'
    if '--check' in sys.argv[1:]:
        if not header.exists() or header.read_text() != text:
            fail('build/sheng-gmu/ShengGmuSource.h is stale')
        if not firmware_header.exists() or firmware_header.read_text() != firmware_text:
            fail('build/sheng-gmu/ShengGmuFirmware.h is stale')
        if not sqe_header.exists() or sqe_header.read_text() != sqe_text:
            fail('build/sheng-gmu/ShengSqeFirmware.h is stale')
        if not zap_header.exists() or zap_header.read_text() != zap_text:
            fail('build/sheng-gmu/ShengZapFirmware.h is stale')
        print('PASS: A810 GMU inputs match the pinned sources')
        return
    OUT.mkdir(parents=True, exist_ok=True)
    header.write_text(text)
    firmware_header.write_text(firmware_text)
    sqe_header.write_text(sqe_text)
    zap_header.write_text(zap_text)
    report = dict(linux_revision=PIN['commit'], source_sha256=PIN['files'], registers=regs,
                  windows=[dict(base=hex(b), bytes=hex(s)) for b, s in windows],
                  window_names=['gmu', 'rscc', '(no gmu_pdc on A8xx)', 'kgsl_3d0_reg_memory', 'cx_misc'],
                  fence_range=hex(fence_word), gbif_cx=[[hex(a), hex(v)] for a, v in gbif_writes],
                  gmu_firmware_chip_id=hex(fw_chip), firmware=dict(pin, blocks=blocks),
                  generated_header_sha256=sha(header), source_analysis_only=True, hardware_registers_observed=False)
    (OUT / 'source-report.json').write_text(json.dumps(report, indent=2) + '\n')
    print('PASS: %d registers, fence 0x%x, %d GBIF CX writes, %d firmware blocks, no RSC/PDC setup (AOP on A8xx)'
          % (len(regs), fence_word, len(gbif_writes), blocks))


main()
