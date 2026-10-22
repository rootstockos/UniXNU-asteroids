# UniXNU-asteroids

[UniXNU](https://github.com/UniXNU/UniXNU) device repository for the **Nothing Phone (3a)** (`asteroids`, Qualcomm
SM7635 "Volcano"). Work in progress: not hardware-complete, and nothing here is meant to be flashed yet.

| Repository | Contents |
|---|---|
| [rootstockos/uefi-asteroids](https://github.com/rootstockos/uefi-asteroids) (branch `asteroids`) | Mu-Silicium UEFI port: `VolcanoPkg` and `asteroidsPkg` |
| this repository | `board.toml`, kernel defconfig, device package |

## State (2026-10-22)

Booted from RAM with `fastboot boot`, nothing flashed:

- the UEFI port reaches DXE and BDS with the correct display;
- the UniXNU menu shows and stays up (empty payload, so no `xnu` entry yet).

Several vendor drivers are disabled for bring-up; `Platforms/Nothing/asteroidsPkg/PORTING.md` in the UEFI port
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
