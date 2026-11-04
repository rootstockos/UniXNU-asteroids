#!/usr/bin/env python3
"""sign-root-policies.py USERDATA_IMAGE OUT_DIR KEY_DIR: sign the OS and SystemOS windows of a UniXNU userdata image with the local owner key
(KEY_DIR/owner.pem and owner-public.pem, never in a repository; UniXNU kits/VerifiedRoot/tools/manifest.py), one policy directory per window: manifest.bin, manifest.header.bin (the 544-byte signed
header the boot image carries), trusted-key.bin, manifest.json."""
import importlib.util, io, struct, sys
from pathlib import Path
U = Path(__file__).resolve().parents[2] / 'UniXNU'
UUIDS = {'OS': '634783FE-1961-47A7-959E-C2676F56F57A', 'SystemOS': 'FA95C6F7-CDBC-46A3-A7E2-39E32EA5D265'}
spec = importlib.util.spec_from_file_location('manifest', U / 'kits/VerifiedRoot/tools/manifest.py')
manifest = importlib.util.module_from_spec(spec); spec.loader.exec_module(manifest)
image, out, KEYS = Path(sys.argv[1]), Path(sys.argv[2]), Path(sys.argv[3])
with image.open('rb') as f:
    head = f.read(4096)
    magic, ver, count, total = struct.unpack_from('<8sIIQ', head, 0)
    assert magic == b'SHNGROOT', magic
    for i in range(count):
        e = head[24 + 128 * i:24 + 128 * (i + 1)]
        name = e[:32].split(b'\0')[0].decode(); offset, size = struct.unpack_from('<QQ', e, 32)
        if name not in UUIDS: continue
        f.seek(offset)
        class Window(io.RawIOBase):
            left = size
            def read(self, n=-1):
                n = self.left if n < 0 else min(n, self.left); data = f.read(n); self.left -= len(data); return data
        record = manifest.make(Window(), size, KEYS / 'owner.pem', KEYS / 'owner-public.pem', UUIDS[name], 1, out / name)
        (out / name / 'manifest.header.bin').write_bytes((out / name / 'manifest.bin').read_bytes()[:544])
        (out / name / 'hashes.bin').unlink(); (out / name / 'header.bin').unlink(); (out / name / 'signature.bin').unlink()
        print(name, record['image_bytes'], record['block_count'], record['image_sha256'], record['trusted_key_sha256'][:16])
