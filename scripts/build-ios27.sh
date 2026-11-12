#!/usr/bin/env bash
# One iPhone iOS 27 (iPhone17,4 24A446) build on a Mac, after sheng's scripts/cycle27.sh: this board's GPU addon (arm64e
# plugin, Turnip and the metal2vulkan bridge, with its owner trust bundle), the kernel with the runtime step's Data seed, and the
# payload. The boot image is packed where the firmware volume and the phone's DTB are (unixnu bootimg --fd --dtb), then booted with
# fastboot boot.
#
# usage: scripts/build-ios27.sh [--init-data] [--no-addon] [--no-kernel] [extra --kernel-arg values ...]
#   --init-data  a FIRST installation: make a new Data seed (a new Data volume UUID, so the kernel is bound to it). Only when no
#                seed is kept in out/asteroids/ios-27.0.1-24A446/asteroids_ios27/runtime/data.dbr; later builds reuse that seed.
#   --no-addon   reuse build/asteroids-metal-plugin-published from the last run
#   --no-kernel  payload only, from the last kernel
# Needs: Xcode 26.6 (iPhoneOS SDK), Homebrew llvm lld dtc mtools cmake ninja glslang spirv-tools bison pkgconf rustup and
# blacktop/tap/ipsw, the UniXNU checkout beside this repository (or UNIXNU=PATH), and UNIXNU_UNIQUE_CHIP_ID (the phone's platform
# seed: keep it out of every repository and log).
set -euo pipefail
DEVICE=$(cd "$(dirname "$0")/.." && pwd)
U=$(cd "${UNIXNU:-$DEVICE/../UniXNU}" && pwd)
OS=ios-27.0.1-24A446
W=$U/out/asteroids/$OS/asteroids_ios27
ADDON=$DEVICE/build/asteroids-metal-plugin-published
ORIGINAL=$U/build/original-trust-$OS/original.bundle
LOG=${BUILD_LOG:-/tmp/build-ios27.log}
init=0 addon=1 kernel=1
while [ $# -gt 0 ]; do
  case "$1" in
    --init-data) init=1; shift ;;
    --no-addon) addon=0; shift ;;
    --no-kernel) kernel=0; shift ;;
    *) break ;;
  esac
done
# macOS bash 3.2 treats an empty array as unbound under set -u, so the optional arguments are a list that is never empty.
args=(--os $OS)
for a in "$@"; do args+=(--kernel-arg "$a"); done
seed=(--os $OS)
[ $init = 1 ] && seed+=(--init-data)

[ "$(uname)" = Darwin ] || { echo 'macOS only: Xcode, hdiutil and codesign build the arm64e parts'; exit 1; }
ok=1
for t in xcrun xcodebuild git python3 rustup cargo dtc mformat mcopy cmake ninja glslangValidator ipsw; do
  command -v "$t" >/dev/null || { echo "missing tool: $t"; ok=0; }
done
for t in llvm-objcopy llvm-nm ld64.lld; do
  [ -x "$(brew --prefix llvm)/bin/$t" ] || [ -x "$(brew --prefix lld)/bin/$t" ] || { echo "missing tool: $t (brew install llvm lld)"; ok=0; }
done
[ "$ok" = 1 ]
[ -n "${UNIXNU_UNIQUE_CHIP_ID:-}" ] || { echo 'set UNIXNU_UNIQUE_CHIP_ID to the phone platform seed (8 hex digits)'; exit 1; }
export PATH="$(brew --prefix llvm)/bin:$(brew --prefix lld)/bin:$PATH"
if [ $init = 1 ] && [ -f "$W/runtime/data.dbr" ]; then
  echo "a Data seed is already kept in $W/runtime/data.dbr: --init-data would replace this installation's Data; move it away first"
  exit 1
fi
: > "$LOG"

if [ $addon = 1 ]; then
  echo '== GPU addon (arm64e)'
  ( cd "$U"
    git submodule update --init --depth 1 drivers/AdrenoMetal drivers/GopFramebuffer kits/AdrenoPM4
    # AdrenoMetal's GpuScanoutClient.h includes ../NativeMobileDisplay/NativeGOPWire.h, GopFramebuffer's legacy name.
    [ -e drivers/NativeMobileDisplay ] || ln -s GopFramebuffer drivers/NativeMobileDisplay
  ) >> "$LOG" 2>&1
  rustup toolchain install 1.98.0 --profile minimal --component rust-src >> "$LOG" 2>&1
  python3 "$DEVICE/scripts/build-ipsw-trust-bundle.py" "$U" $OS "$ORIGINAL" >> "$LOG" 2>&1
  python3 "$DEVICE/scripts/prepare-asteroids-turnip.py" --unixnu "$U" --target ios27 >> "$LOG" 2>&1 || { echo "Turnip failed: $LOG"; exit 1; }
  ( cd "$U"
    read -r repo commit < <(python3 -c "import json; t = json.load(open('scripts/data/gpu-metal-vulkan-inputs.json'))['translator']; print(t['repository'], t['commit'])")
    src=.cache/gpu-reference/metal2vulkan
    if [ "$(git -C "$src" rev-parse HEAD 2>/dev/null)" != "$commit" ]; then
      rm -rf "$src"; git init -q "$src"
      git -C "$src" fetch -q --depth 1 --filter=blob:none "$repo" "$commit"
      git -C "$src" -c advice.detachedHead=false checkout -q FETCH_HEAD
    fi
    python3 scripts/build-adreno-metal-bridge.py --target ios27
  ) >> "$LOG" 2>&1 || { echo "bridge failed: $LOG"; exit 1; }
  python3 "$DEVICE/scripts/build-asteroids-metal-plugin.py" --unixnu "$U" --published --target ios27 --trust-cache "$ORIGINAL" \
    >> "$LOG" 2>&1 || { echo "plugin failed: $LOG"; exit 1; }
fi
[ -f "$ADDON/asteroids-metal.addon" ] && [ -f "$ADDON/owner-trust.bundle" ] || { echo "no addon in $ADDON"; exit 1; }

cd "$U"
if [ $kernel = 1 ]; then
  echo '== kernel (runtime: Data seed)'
  ./unixnu fetch -D "$DEVICE" --os $OS >> "$LOG" 2>&1
  ./unixnu ipsw fetch -D "$DEVICE" --os $OS >> "$LOG" 2>&1
  # A fresh .config from the defconfig; the runtime step then writes the Data seed's volume UUID into it before the kernel builds
  # (it runs only when the runtime or payload step is selected, so both runtime and kernel are named).
  ./unixnu defconfig asteroids_ios27_defconfig -D "$DEVICE" --os $OS >> "$LOG" 2>&1
  ./unixnu build runtime kernel -D "$DEVICE" --gpu-addon "$ADDON/asteroids-metal.addon" \
    --owner-trust-cache "$ADDON/owner-trust.bundle" "${seed[@]}" >> "$LOG" 2>&1 \
    || { echo "kernel build failed: $LOG"; grep -n " error\|referenced from\|Error:" "$LOG" | tail -15; exit 1; }
fi
[ -f "$W/runtime/data.dbr" ] || { echo "no Data seed in $W/runtime: first install with --init-data"; exit 1; }

echo '== payload'
./unixnu build payload -D "$DEVICE" --gpu-addon "$ADDON/asteroids-metal.addon" \
  --owner-trust-cache "$ADDON/owner-trust.bundle" "${args[@]}" >> "$LOG" 2>&1 \
  || { echo "payload failed: $LOG"; tail -5 "$LOG"; exit 1; }
echo "payload ready: $W/payload ($(shasum -a 256 "$W/kernel/kernel.macho" | cut -c1-16) kernel)"
echo "copy $W/payload and $W/runtime to the packing machine, then: ./unixnu bootimg -D ../UniXNU-asteroids --os $OS --fd FD --dtb DTB"
