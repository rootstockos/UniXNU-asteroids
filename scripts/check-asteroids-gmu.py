#!/usr/bin/env python3
"""Build and run the gen8 GMU cold-start host test (package/ShengGmuGen8HostTests.c) against the generated
build/sheng-gmu headers, the kits/AdrenoFirmware submodule and the pinned gen80300_gmu.bin. Host only."""
import argparse, subprocess
from pathlib import Path
ROOT = Path(__file__).resolve().parents[1]
parser = argparse.ArgumentParser(description=__doc__)
args = parser.parse_args()
kit = ROOT / 'kits/AdrenoFirmware'   # rootstockos/AdrenoFirmware asteroids-a8xx (submodule)
subprocess.run(['python3', str(ROOT / 'scripts/prepare-asteroids-gmu.py'), '--check'], check=True)
subprocess.run(['python3', str(ROOT / 'scripts/prepare-asteroids-gmu-votes.py'), '--check'], check=True)
out = ROOT / 'build/sheng-gmu'
binary = out / 'gen8-host-test'
# The kit's HFI transport is AArch64-only (its barriers are dmb/dsb). On another host, compile a copy whose only
# change is those barriers as compiler fences, so the real queue logic still runs; each substitution must match once.
hfi = kit / 'AdrenoHfi.c'
if subprocess.run(['uname', '-m'], capture_output=True, text=True).stdout.strip() != 'arm64':
    text = hfi.read_text()
    swaps = [('#if !defined(__aarch64__) || __BYTE_ORDER__ != __ORDER_LITTLE_ENDIAN__\n',
              '#if __BYTE_ORDER__ != __ORDER_LITTLE_ENDIAN__\n'),
             ('__asm__ volatile("dmb oshld":::"memory");', '__atomic_thread_fence(__ATOMIC_ACQUIRE);'),
             ('__asm__ volatile("dsb oshst":::"memory");', '__atomic_thread_fence(__ATOMIC_RELEASE);'),
             ('__asm__ volatile("dmb osh":::"memory");', '__atomic_thread_fence(__ATOMIC_SEQ_CST);')]
    for old, new in swaps:
        if text.count(old) != 1:
            raise SystemExit('AdrenoHfi.c changed; the host copy no longer applies')
        text = text.replace(old, new)
    if '__asm__' in text:
        raise SystemExit('AdrenoHfi.c has another barrier')
    hfi = out / 'host-AdrenoHfi.c'
    hfi.write_text(text)
# The GMU cold start and command DB are the SoC profile's kit: the build tree's kits/Sm7635Platform, or a UniXNU-sm7635-common
# checkout: the one local.toml names, or one next to this repository.
import tomllib
candidates = [ROOT / 'kits/Sm7635Platform']
if (ROOT / 'local.toml').is_file():
    candidates += [(ROOT / e['local']).expanduser() / 'kits/Sm7635Platform'
                   for e in tomllib.loads((ROOT / 'local.toml').read_text()).get('source', []) if e.get('kind') == 'soc' and e.get('local')]
candidates.append(ROOT.parent / 'UniXNU-sm7635-common/kits/Sm7635Platform')
soc_kit = next((p for p in candidates if (p / 'ShengGmu.c').is_file()), None)
if soc_kit is None:
    raise SystemExit('kits/Sm7635Platform not found (UniXNU-sm7635-common next to this repository)')
subprocess.run(['clang', '-std=c11', '-O1', '-Wall', '-Wextra', '-Werror', '-DSHENG_GMU_HOST_TEST',
                '-I', str(out), '-I', str(kit), '-I', str(soc_kit), '-I', str(ROOT / 'package'),
                str(ROOT / 'package/ShengGmuGen8HostTests.c'), str(soc_kit / 'ShengGmu.c'),
                str(ROOT / 'package/ShengGmuVotes.c'), str(soc_kit / 'ShengGmuCommandDb.c'), str(kit / 'AdrenoHfiVotes.c'),
                str(kit / 'AdrenoFirmware.c'), str(hfi), str(kit / 'AdrenoHfiStartup.c'),
                '-o', str(binary)], check=True)
subprocess.run([str(binary), str(ROOT / '.cache/firmware/gen80300_gmu.bin')], check=True)
