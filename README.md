# UniXNU-asteroids

[UniXNU](https://github.com/UniXNU/UniXNU) device repository for the **Nothing Phone (3a)** (`asteroids`, Qualcomm
SM7635 "Volcano"). Work in progress: not hardware-complete, and nothing here is meant to be flashed yet.

| Repository | Contents |
|---|---|
| [rootstockos/uefi-asteroids](https://github.com/rootstockos/uefi-asteroids) (branch `asteroids`) | Mu-Silicium UEFI port: `VolcanoPkg` and `asteroidsPkg` |
| [rootstockos/UniXNU](https://github.com/rootstockos/UniXNU) (branch `asteroids-dev`) | the UniXNU core fork the port builds with: v3/v4 boot images, the payload carried in the kernel image, CI |
| this repository | `board.toml`, kernel defconfigs, device package (`kits/asteroids`), device scripts |

Drivers and kits, each its own repository, referenced as submodules:

| Module | Repository | Use |
|---|---|---|
| `drivers/NativeUfs` | [rootstockos/NativeUfs](https://github.com/rootstockos/NativeUfs) (branch `fix/unverified-build`) | UFS root in `userdata` |
| `drivers/NativeAdreno` | [rootstockos/NativeAdreno](https://github.com/rootstockos/NativeAdreno) (branch `asteroids-a810`) | Adreno 810 (gen8) work; not built yet (`CONFIG_XNU_NATIVE_ADRENO` off) |
| `kits/AdrenoFirmware` | [rootstockos/AdrenoFirmware](https://github.com/rootstockos/AdrenoFirmware) (branch `asteroids-a8xx`) | GMU/HFI kit with the A8xx performance table and votes |

## State (2026-10-24)

Booted with `fastboot boot`; the OS images are flashed to `userdata`:

- UEFI, the UniXNU menu and payload, DarwinBoot and XNU (iPad7,1 iPadOS 17.7.11 kernelcache) on all eight cores.
- The root is mounted from UFS (`userdata`) and iPadOS userspace runs: launchd, the SystemOS cryptex and SpringBoard.
  The text console shows the boot until the first display frame.
- No display yet. The CPU-rendered legacy display opens its client but the render server never presents a frame;
  uke (SM7675) reached the same point and concluded that iOS 17 needs a GPU accelerator. SpringBoard restarts
  ("too many corpses") without one.
- The Adreno 810 (gen8) work is in software so far: the register map and init tables from mainline, per-generation
  registers in NativeAdreno, and the gen8 GMU cold start and HFI startup (host-tested, `scripts/check-asteroids-gmu.py`).
  NativeAdreno still refuses gen8 at start.

Several vendor drivers are disabled in the UEFI port for bring-up; `Platforms/Nothing/asteroidsPkg/PORTING.md` there
lists them and why.

## Build

The firmware step builds on Linux too (tested in an Ubuntu 24.04 container). It needs a Mu-Silicium checkout of
`uefi-asteroids` with the phone's own vendor drivers in `Binaries/asteroids` (extracted from its `uefi` partition with
UEFIReader; never committed), and UniXNU with support for header v3/v4 boot images
(rootstockos/UniXNU, branch `feat/bootimg-header-v4`).

```sh
./unixnu defconfig asteroids_defconfig -D ../UniXNU-asteroids
./unixnu build package firmware --uefi ../uefi-asteroids --edk2 ../uefi-asteroids/Mu_Basecore
./unixnu bootimg --uefi ../uefi-asteroids --dtb /path/to/live.dtb --ramdisk none
fastboot boot out/asteroids/boot/boot.img
```

`live.dtb` is the phone's `/sys/firmware/fdt`. It carries the `splash_region` the bootloader needs to keep the
panel on.

Target OS: the unchanged iPad7,1 iPadOS 17.7.11 restore image, as on UniXNU's sheng port.

## Third-party sources in `package/`

The boot progress kit (`BootProgress*.{c,h}`, `BootFaultProbe.c`, `BootDiagExport.c`, `BootUiTrace*`, `BootProcessProbe.c`)
is UniXNU-sheng's (commit 0e1f6e5; MIT, `BootProcessProbe.c` BSD-2-Clause-Patent), unchanged. `Font.h` is the AOSP
framebuffer font from Mu-Silicium's FrameBufferSerialPortLib (its licence is in the file). `BootWatchdogPolicy.c` is
Allenwdk/UniXNU-uke's (MIT) with asteroids additions.
