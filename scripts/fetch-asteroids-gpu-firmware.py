#!/usr/bin/env python3
"""Fetch the A810 firmware (board.toml [gpu] firmware_sqe/gmu/zap) into .cache/firmware/asteroids-gpu.

The one source is board.toml's firmware_repository at firmware_commit (NullDebris's vendor repository on GitLab); there is
no fallback. Each file must match its [gpu.firmware_pins] size and sha256 or nothing is written. A file already in place
with the right pin is kept. --check only verifies what is there. The blobs are Qualcomm's: never commit them; CI removes
.cache/firmware after the build."""
import hashlib, subprocess, sys, tomllib
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / '.cache/firmware/asteroids-gpu'


def fail(message):
    raise SystemExit('fetch-asteroids-gpu-firmware: ' + message)


def matches(data, pin):
    return len(data) == pin['bytes'] and hashlib.sha256(data).hexdigest() == pin['sha256']


def main():
    gpu = tomllib.loads((ROOT / 'board.toml').read_text())['gpu']
    names = gpu['firmware_sqe'] + gpu['firmware_gmu'] + gpu['firmware_zap']
    commit = gpu['firmware_commit']
    if len(commit) != 40 or any(c not in '0123456789abcdef' for c in commit):
        fail('firmware_commit must be a full commit id')
    check = '--check' in sys.argv[1:]
    OUT.mkdir(parents=True, exist_ok=True)
    for name in names:
        pin = gpu['firmware_pins'][name]
        target = OUT / name
        if target.exists() and matches(target.read_bytes(), pin):
            print('kept ' + name)
            continue
        if check:
            fail(name + ' is missing or differs from its pin')
        url = '%s/-/raw/%s/%s/%s' % (gpu['firmware_repository'], commit, gpu['firmware_path'], name)
        # curl, with the system's CA store (the python.org builds ship none).
        data = subprocess.run(['curl', '-fsSL', '--proto', '=https', '--retry', '3', '--max-time', '120',
                               '--max-filesize', str(pin['bytes']), url], check=True, capture_output=True).stdout
        if not matches(data, pin):
            fail(name + ' from ' + url + ' does not match its pin')
        tmp = target.with_suffix(target.suffix + '.tmp')
        tmp.write_bytes(data)
        tmp.replace(target)
        print('fetched %s (%d bytes, sha256 %s)' % (name, pin['bytes'], pin['sha256'][:16]))


main()
