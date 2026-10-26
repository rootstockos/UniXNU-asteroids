# UniXNU-asteroids

[UniXNU](https://github.com/UniXNU/UniXNU) device repository for the **Nothing Phone (3a)** (`asteroids`, Qualcomm
SM7635 "Volcano"). Work in progress: not hardware-complete, and nothing here is meant to be flashed yet.

| Repository | Contents |
|---|---|
| [rootstockos/uefi-asteroids](https://github.com/rootstockos/uefi-asteroids) (branch `asteroids`) | Mu-Silicium UEFI port: `VolcanoPkg` and `asteroidsPkg` |
| [rootstockos/UniXNU](https://github.com/rootstockos/UniXNU) | the UniXNU core fork the port builds with: v3/v4 boot images, the payload carried in the kernel image, bring-up patches, CI |
| [rootstockos/UniXNU-sm7635-common](https://github.com/rootstockos/UniXNU-sm7635-common) | the shared SM7635 profile (device format 2): CPUs, GIC, UFS and Adreno 810 resources |
| this repository | `board.toml`, `dts/asteroids.dts`, kernel defconfigs, board XNU patches, device package (`kits/asteroids`), device scripts |

The repository uses UniXNU's device format 2 (`schema_version = 2`): the SM7635 SoC profile, the iPadOS 17.7.11 (21H461) OS
profile and catalog components. Its own driver forks are submodules at their catalog paths, where they take precedence over
UniXNU's:

| Module | Repository | Use |
|---|---|---|
| `drivers/ShengUfs` | [rootstockos/NativeUfs](https://github.com/rootstockos/NativeUfs) (branch `asteroids`) | UFS root in `userdata` |
| `drivers/Adreno740` | [rootstockos/NativeAdreno](https://github.com/rootstockos/NativeAdreno) (branch `asteroids`) | Adreno 810 (gen8): GMU, ring, submits and the user client |
| `drivers/ArmDTPlatform` | [rootstockos/ArmDTPlatform](https://github.com/rootstockos/ArmDTPlatform) (branch `asteroids`) | Platform expert; forwards the wall clock to the PMIC RTC (counter + offset on Data) |
| `drivers/ShengHidInput` | [UniXNU/ShengHidInput](https://github.com/UniXNU/ShengHidInput) | HID touch and buttons |
| `kits/AdrenoFirmware` | [rootstockos/AdrenoFirmware](https://github.com/rootstockos/AdrenoFirmware) (branch `asteroids`) | GMU/HFI kit with the A8xx performance table and votes |

## State (2026-10-26)

Booted with `fastboot boot`; the OS images are flashed to `userdata`. iPadOS 17.7.11 boots from UFS to the desktop on all
eight cores. The render server composites on the Adreno 810 through Metal (metal2vulkan and Turnip), the GPU copies each
frame to the panel, and touch works. The frame rate is still under 60. Test results are in `docs/asteroids-gpu-gen8.md` and
`docs/asteroids-touch.md` of rootstockos/iosonandng; `porting.toml` records each area.

Several vendor drivers are disabled in the UEFI port for bring-up; `Platforms/Nothing/asteroidsPkg/PORTING.md` there
lists them and why.

## Build

From the UniXNU checkout, with UniXNU-sm7635-common next to this repository (`local.toml` points the SoC source at
`../UniXNU-sm7635-common` until `feeds.toml`'s pin is published):

```sh
./unixnu defconfig asteroids_defconfig -D ../UniXNU-asteroids
./unixnu status --json -D ../UniXNU-asteroids
./unixnu build kernel -D ../UniXNU-asteroids
./unixnu build payload -D ../UniXNU-asteroids --uefi ../uefi-asteroids --edk2 ../uefi-asteroids/Mu_Basecore
./unixnu bootimg -D ../UniXNU-asteroids --uefi ../uefi-asteroids --dtb /path/to/live.dtb
```

Outputs are under `out/asteroids/ipados-17.7.11-21H461/asteroids/`. The kernel builds in CI (rootstockos/UniXNU's macOS
workflow, with `scripts/ci-bootstrap-candidate.sh` as its pre-build step); the payload step compiles `dts/asteroids.dts`
with `dtc` and writes the ADT with UniXNU's shared generator.

`live.dtb` is the phone's `/sys/firmware/fdt`. It carries the `splash_region` the bootloader needs to keep the
panel on.

## Third-party sources in `package/`

The boot progress kit (`BootProgress*.{c,h}`, `BootFaultProbe.c`, `BootDiagExport.c`, `BootUiTrace*`, `BootProcessProbe.c`)
is UniXNU-sheng's (commit 0e1f6e5; MIT, `BootProcessProbe.c` BSD-2-Clause-Patent), unchanged. `Font.h` is the AOSP
framebuffer font from Mu-Silicium's FrameBufferSerialPortLib (its licence is in the file). `BootWatchdogPolicy.c` is
Allenwdk/UniXNU-uke's (MIT) with asteroids additions.
