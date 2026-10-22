#!/bin/bash
# CI bootstrap pass, run from the UniXNU checkout as the macOS workflow's pre_build step:
#   pre_build: bash "$DEVICE_DIR/scripts/ci-bootstrap-candidate.sh"
# Builds and prelinks bootstrap_defconfig (no device archives), keeps the prelinked kernel as the original-class
# candidate for build-ufs-original-bindings.py, then selects asteroids_defconfig again for the real build.
# The same two passes as Allenwdk/UniXNU-uke's build-kernel workflow.
set -euo pipefail
device=${DEVICE_DIR:?DEVICE_DIR is the device repository}
out=out/asteroids
./unixnu defconfig bootstrap_defconfig -D "$device"
./unixnu build kernel -D "$device"
mkdir -p "$out/tree/build/native-prelink"
cp "$out/kernel/kernel.macho" "$out/tree/build/native-prelink/bootstrap-candidate.macho"
cp "$out/kernel/kernel.json" "$out/tree/build/native-prelink/bootstrap-candidate.json"
# The bootstrap kernel must not be taken for the result.
rm -f "$out/kernel/kernel.macho" "$out/kernel/kernel.json"
./unixnu defconfig asteroids_defconfig -D "$device"
