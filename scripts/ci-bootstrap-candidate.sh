#!/bin/bash
# CI bootstrap pass, run from the UniXNU checkout as the macOS workflow's pre_build step:
#   pre_build: bash "$DEVICE_DIR/scripts/ci-bootstrap-candidate.sh"
# Builds and prelinks bootstrap_defconfig (no device archives), keeps the prelinked kernel as the original-class
# candidate for build-ufs-original-bindings.py, then selects asteroids_defconfig again for the real build.
# The same two passes as Allenwdk/UniXNU-uke's build-kernel workflow.
set -euo pipefail
device=${DEVICE_DIR:?DEVICE_DIR is the device repository}
# Format 2 builds each defconfig in its own directory (out/<device>/<os_profile>/<config>/, with its own tree); format 1 shares
# out/asteroids. outdir prints the selected configuration's directory.
outdir() { ./unixnu status --json -D "$device" | python3 -c 'import json,sys; print(json.load(sys.stdin)["output"])'; }
out=out/asteroids
# Status for the CI log: unixnu sends each command's output to $out/logs/*.log and Python block-buffers a pipe, so a
# long stage otherwise shows nothing. Every stage prints a timestamped line, and a heartbeat prints the newest log's
# last line once a minute.
export PYTHONUNBUFFERED=1
start=$(date +%s)
stage() { printf '== [%s +%dm] %s\n' "$(date -u +%H:%M:%S)" $(( ($(date +%s) - start) / 60 )) "$*"; }
heartbeat() {
  while sleep 60; do
    latest=$(ls -t "$out"/logs/*.log 2>/dev/null | head -1 || true)
    line=$( { [ -n "$latest" ] && tail -n 1 "$latest" | cut -c1-160; } 2>/dev/null || true)
    stage "still running${latest:+: $(basename "$latest"): $line}"
  done
}
heartbeat &
beat=$!
trap 'kill "$beat" 2>/dev/null || true' EXIT
# The A810 firmware: proprietary-files.txt's files from its one pinned source, checked against their SHA-256
# (scripts/extract-files.py, as sheng does). The workflow's last step deletes device/.cache/firmware.
stage "fetch the A810 firmware"
python3 "$device/scripts/extract-files.py" --device "$device"
# The pinned linux sources for the GMU register header (sparse checkout in device/.cache/linux-a8xx); the archive step
# generates the tree's copy from the same inputs.
stage "fetch the pinned linux sources for the GMU header"
python3 "$device/scripts/prepare-asteroids-gmu.py" --fetch
stage "build and prelink the bootstrap kernel (from scratch: about an hour)"
./unixnu defconfig bootstrap_defconfig -D "$device"
./unixnu build kernel -D "$device"
boot=$(outdir)
stage "select asteroids_defconfig for the real build"
./unixnu defconfig asteroids_defconfig -D "$device"
out=$(outdir)
if [ "$out" != "$boot" ]; then
  # Format 2: the real build starts from the bootstrap pass's patched XNU tree, objects and input stamp, as format 1 did in its
  # one shared tree, instead of compiling XNU again.
  stage "carry the bootstrap XNU tree over to $out"
  ./unixnu tree -D "$device" >/dev/null
  for path in tree/.cache/xnu-work tree/.cache/fakeroot tree/.cache/sources tree/build/xnu tree/build/availability xnu-inputs.json; do
    if [ -e "$boot/$path" ]; then mkdir -p "$(dirname "$out/$path")"; rm -rf "$out/$path"; cp -a "$boot/$path" "$out/$path"; fi
  done
  mkdir -p "$out/kernel"
fi
mkdir -p "$out/tree/build/xnu-prelink"
cp "$boot/kernel/kernel.macho" "$out/tree/build/xnu-prelink/bootstrap-candidate.macho"
cp "$boot/kernel/kernel.json" "$out/tree/build/xnu-prelink/bootstrap-candidate.json"
# The display and HID builds (uke's and UniXNU's scripts) read the same candidate under these names.
for name in native-owner-mobile-graphics-candidate native-owner-ufs-cpu-desktop-v9-hid-init-candidate; do
  cp "$boot/kernel/kernel.macho" "$out/tree/build/xnu-prelink/$name.macho"
  cp "$boot/kernel/kernel.json" "$out/tree/build/xnu-prelink/$name.json"
done
# audit-mobile-graphics-legacy.py accepts the recorded runtime observation hash from its previous output when the live
# (HVF) observation is absent. The observation is of the unchanged iPad7,1 IOMobileFramebufferLegacy class in the shared
# 21H461 kernelcache, not of our kernel; uke's CI seeds the same value.
mkdir -p "$out/tree/build/display-analysis"
abi="$out/tree/build/display-analysis/mobile-graphics-legacy-abi.json"
[ -f "$abi" ] || printf '{"runtime_metadata_checkpoint_sha256": "3044c2beff552911605994c0dc7aa73182066cbb5aed0b396999ceeb5a96da2b"}\n' > "$abi"
# The bootstrap kernel must not be taken for the result.
rm -f "$boot/kernel/kernel.macho" "$boot/kernel/kernel.json"
stage "bootstrap done"
