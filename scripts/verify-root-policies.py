#!/usr/bin/env python3
"""verify-root-policies.py USERDATA_IMAGE POLICY_DIR LIBRARY: authenticate each window's policy with UniXNU's VerifiedRoot C code (LIBRARY: cc -shared -fPIC
-I include src/verified_root.c tests/openssl_provider.c -lcrypto): the full envelope and the 544-byte header alone against trusted-key.bin, then every 4 KiB
block of the window, and the digest of the whole. Writes verification.json beside each policy, in the form sheng's policies use."""
import ctypes, hashlib, json, struct, sys, uuid
from pathlib import Path
class View(ctypes.Structure):
    _fields_ = [('hashes', ctypes.c_void_p), ('image_bytes', ctypes.c_uint64), ('block_count', ctypes.c_uint64),
                ('generation', ctypes.c_uint64), ('volume_uuid', ctypes.c_uint8 * 16), ('image_sha256', ctypes.c_uint8 * 32)]
native = ctypes.CDLL(sys.argv[3])
for f in ('vr_host_open', 'vr_host_open_header'):
    getattr(native, f).argtypes = [ctypes.c_void_p, ctypes.c_size_t, ctypes.c_void_p, ctypes.c_uint64, ctypes.POINTER(View)]
    getattr(native, f).restype = ctypes.c_int
native.vr_host_block.argtypes = [ctypes.POINTER(View), ctypes.c_uint64, ctypes.c_void_p, ctypes.c_size_t]
native.vr_host_block.restype = ctypes.c_int
image, policies = Path(sys.argv[1]), Path(sys.argv[2])
with image.open('rb') as f:
    head = f.read(4096); _, _, count, _ = struct.unpack_from('<8sIIQ', head, 0)
    windows = {}
    for i in range(count):
        e = head[24 + 128 * i:24 + 128 * (i + 1)]; windows[e[:32].split(b'\0')[0].decode()] = struct.unpack_from('<QQ', e, 32)
    for name in ('OS', 'SystemOS'):
        d = policies / name; offset, size = windows[name]
        envelope = ctypes.create_string_buffer((d / 'manifest.bin').read_bytes())
        header = ctypes.create_string_buffer((d / 'manifest.header.bin').read_bytes())
        key = (d / 'trusted-key.bin').read_bytes(); anchor = ctypes.create_string_buffer(key)
        view, hview = View(), View()
        assert native.vr_host_open(envelope, len(envelope) - 1, anchor, 1, ctypes.byref(view)) == 0, name + ' envelope'
        assert native.vr_host_open_header(header, len(header) - 1, anchor, 1, ctypes.byref(hview)) == 0, name + ' header'
        assert bytes(hview.image_sha256) == bytes(view.image_sha256) and bytes(hview.volume_uuid) == bytes(view.volume_uuid)
        f.seek(offset); whole = hashlib.sha256()
        for index in range(view.block_count):
            block = f.read(min(4096, view.image_bytes - index * 4096))
            assert native.vr_host_block(ctypes.byref(view), index, block, len(block)) == 0, f'{name} block {index}'
            whole.update(block)
        assert view.image_bytes == size and whole.digest() == bytes(view.image_sha256)
        report = dict(image_bytes=view.image_bytes, blocks_verified=view.block_count, image_sha256=whole.hexdigest(),
                      volume_uuid=str(uuid.UUID(bytes=bytes(view.volume_uuid))).upper(), generation=view.generation,
                      trusted_key_sha256=hashlib.sha256(key).hexdigest(), real_c_signature_and_block_verification=True,
                      header_alone_authenticated=True, original_image_modified=False, host_only=True,
                      boot_root_authenticated=False, qemu_tested=False, hardware_tested=False)
        (d / 'verification.json').write_text(json.dumps(report, indent=2) + '\n')
        print(name, 'PASS', view.block_count, 'blocks', whole.hexdigest()[:16], report['volume_uuid'])
