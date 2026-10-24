#!/bin/bash
# CI bootstrap pass, run from the UniXNU checkout as the macOS workflow's pre_build step:
#   pre_build: bash "$DEVICE_DIR/scripts/ci-bootstrap-candidate.sh"
# Builds and prelinks bootstrap_defconfig (no device archives), keeps the prelinked kernel as the original-class
# candidate for build-ufs-original-bindings.py, then selects asteroids_defconfig again for the real build.
# The same two passes as Allenwdk/UniXNU-uke's build-kernel workflow.
set -euo pipefail
device=${DEVICE_DIR:?DEVICE_DIR is the device repository}
out=out/asteroids
# The A810 firmware, from board.toml's one pinned source; checked against its pins. The workflow's last step deletes
# device/.cache/firmware.
python3 "$device/scripts/fetch-asteroids-gpu-firmware.py"
./unixnu defconfig bootstrap_defconfig -D "$device"
./unixnu build kernel -D "$device"
mkdir -p "$out/tree/build/native-prelink"
cp "$out/kernel/kernel.macho" "$out/tree/build/native-prelink/bootstrap-candidate.macho"
cp "$out/kernel/kernel.json" "$out/tree/build/native-prelink/bootstrap-candidate.json"
# The display and HID builds (uke's and UniXNU's scripts) read the same candidate under these names.
for name in native-owner-mobile-graphics-candidate native-owner-ufs-cpu-desktop-v9-hid-init-candidate; do
  cp "$out/kernel/kernel.macho" "$out/tree/build/native-prelink/$name.macho"
  cp "$out/kernel/kernel.json" "$out/tree/build/native-prelink/$name.json"
done
# audit-mobile-graphics-legacy.py accepts the recorded runtime observation hash from its previous output when the live
# (HVF) observation is absent. The observation is of the unchanged iPad7,1 IOMobileFramebufferLegacy class in the shared
# 21H461 kernelcache, not of our kernel; uke's CI seeds the same value.
mkdir -p "$out/tree/build/display-analysis"
abi="$out/tree/build/display-analysis/mobile-graphics-legacy-abi.json"
[ -f "$abi" ] || printf '{"runtime_metadata_checkpoint_sha256": "3044c2beff552911605994c0dc7aa73182066cbb5aed0b396999ceeb5a96da2b"}\n' > "$abi"
# The bootstrap kernel must not be taken for the result.
rm -f "$out/kernel/kernel.macho" "$out/kernel/kernel.json"
./unixnu defconfig asteroids_defconfig -D "$device"
