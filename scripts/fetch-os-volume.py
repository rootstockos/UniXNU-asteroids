#!/usr/bin/env python3
"""fetch-os-volume.py UNIXNU_DIR OS_PROFILE OUT_DIR: the OS disk image of an OS profile's IPSW (its [ipsw] url, product, device class,
chip and board), read with HTTP ranges and, where the IPSW ships it as AEA, decrypted with UniXNU's decrypt_aea (macOS aea and the
ipsw tool, or ipsw alone elsewhere). Prints the image's path: a UDIF image (iPadOS 17) or a raw disk image (iOS 27)."""
import plistlib, shutil, sys, tomllib, zipfile
from pathlib import Path
unixnu, profile_name, out = Path(sys.argv[1]).resolve(), sys.argv[2], Path(sys.argv[3]).resolve()
sys.path.insert(0, str(unixnu / 'scripts'))
from unixnu.ipsw import RangeFile, decrypt_aea
spec = tomllib.loads((unixnu / 'profiles/os' / profile_name / 'profile.toml').read_text())['ipsw']
z = zipfile.ZipFile(RangeFile(spec['url']))
manifest = plistlib.loads(z.read('BuildManifest.plist'))
if (manifest['ProductVersion'], manifest['ProductBuildVersion']) != (spec['version'], spec['build']) or spec['product'] not in manifest['SupportedProductTypes']:
    sys.exit(f'{spec["url"]} is not {spec["product"]} {spec["version"]} ({spec["build"]})')
identity = next(i for i in manifest['BuildIdentities'] if i['Info']['DeviceClass'] == spec['device_class'] and
                i['Info']['RestoreBehavior'] == 'Erase' and int(i['ApChipID'], 16) == spec['chip_id'] and int(i['ApBoardID'], 16) == spec['board_id'])
member = identity['Manifest']['OS']['Info']['Path']
out.mkdir(parents=True, exist_ok=True)
path = out / Path(member).name
with z.open(member) as source, path.open('wb') as target:
    shutil.copyfileobj(source, target, 8 << 20)
if path.suffix == '.aea':
    import hashlib, json
    digest = hashlib.sha256(path.read_bytes()).hexdigest()
    path.with_suffix(path.suffix + '.json').write_text(json.dumps({'component': 'OS', 'member': member, 'sha256': digest}) + '\n')
    decrypted = decrypt_aea(path)
    path.unlink()
    path = decrypted
print(path)
