#!/usr/bin/env python3
"""build-ipsw-trust-bundle.py UNIXNU_DIR OS_PROFILE OUT: the unchanged trust caches of an OS profile's IPSW (StaticTrustCache, then the
Cryptex1 SystemOS and App trust caches where the build identity has them), read with HTTP ranges and bundled by UniXNU's
bundle-ipsw-trust.py for DarwinBoot: module 0 the static trust cache, the others loaded as engineering modules. For the iOS 27 add-on,
whose owner trust bundle carries them after its own module (build-asteroids-metal-plugin.py --target ios27)."""
import plistlib, subprocess, sys, tempfile, tomllib, zipfile
from pathlib import Path
unixnu, profile_name, out = Path(sys.argv[1]).resolve(), sys.argv[2], Path(sys.argv[3]).resolve()
sys.path.insert(0, str(unixnu / 'scripts'))
from unixnu.ipsw import RangeFile
spec = tomllib.loads((unixnu / 'profiles/os' / profile_name / 'profile.toml').read_text())['ipsw']
z = zipfile.ZipFile(RangeFile(spec['url']))
manifest = plistlib.loads(z.read('BuildManifest.plist'))
identity = next(i for i in manifest['BuildIdentities'] if i['Info']['DeviceClass'] == spec['device_class'] and
                i['Info']['RestoreBehavior'] == 'Erase' and int(i['ApChipID'], 16) == spec['chip_id'] and int(i['ApBoardID'], 16) == spec['board_id'])
components = [c for c in ('StaticTrustCache', 'Cryptex1,SystemTrustCache', 'Cryptex1,AppTrustCache') if c in identity['Manifest']]
with tempfile.TemporaryDirectory() as work:
    inputs = []
    for component in components:
        path = Path(work) / (component.replace(',', '_') + '.im4p')
        path.write_bytes(z.read(identity['Manifest'][component]['Info']['Path']))
        inputs.append(str(path))
    out.parent.mkdir(parents=True, exist_ok=True)
    subprocess.run([sys.executable, str(unixnu / 'scripts/bundle-ipsw-trust.py'), str(out), *inputs], check=True)
print(out, components)
