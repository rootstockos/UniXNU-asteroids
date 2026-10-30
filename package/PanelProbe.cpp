// SPDX-License-Identifier: BSD-2-Clause-Patent
// asteroids (T37): the Nothing Phone (3a)'s AMOLED panel through DSI0's command path. Boot argument asteroids-panel=<n>:
//   1  read-only: map DSI0 and record its registers (the PNL banner row)
//   2  mode 1, then a visible brightness test: full, low, then asteroids-panel-level=<1..4095> (default 2048), 2 s apart
//   3  mode 1, then the iOS brightness service (AsteroidsBacklight.cpp): iOS sets the level; level 0 turns the panel off (display off,
//      sleep in) and the next level above 0 turns it on again (sleep out, display on, the level)
//   4  mode 1, then a panel off/on test: display off and sleep in for 3 s, then sleep out, display on and full brightness
// The panel is a Visionox RM69220 (AMOLED, DSC, video mode) on DSI0 at 0x0ae94000 (qcom,dsi-ctrl-hw-v2.8), which the boot loader left
// scanning out (3a devicetree sources qcom/display/display/dsi-panel-rm69220-dsc-fhd-plus-120hz-vid-vxn.dtsi, volcano-sde-common.dtsi).
// Its brightness is DCS 0x51 with two bytes, high byte first (bl-inverted-dbv swaps the bytes the DRM helper sends low byte first),
// 10..4095; its off sequence is 28 (120 ms) then 10 (100 ms), its on sequence ends 11 (80 ms) then 29 (10 ms).
// A command goes out through the controller's TPG command FIFO, written word by word, with no DMA buffer and so no display SMMU
// mapping: the path the downstream driver (display-drivers msm/dsi, dsi_ctrl_hw_cmn_kickoff_fifo_command) uses when the display
// belongs to another VM. In video mode the transfer is scheduled for the first vertical front porch line (dsi_configure_command_scheduling:
// a trigger close to the active area can leave the engine waiting), the software trigger starts it, and INT_CTRL bit 0 (DMA done)
// ends it; then the FIFO and the trigger setup are reset as dsi_ctrl_hw_22_reset_trigger_controls does.
#include <IOKit/IOLib.h>
#include <IOKit/IOMemoryDescriptor.h>
#include <IOKit/IODeviceMemory.h>
#include <IOKit/IOService.h>
#include <libkern/OSKextLib.h>
#include <mach/kmod.h>
#include <kern/thread.h>
#include <kern/clock.h>
#include <mach/mach_time.h>
#include <pexpert/pexpert.h>
#include "BoardClasses.h"

// [0] stage (1 mapped, 2 test done, 3 service started, 4 service published, negative: the step that failed), [1] HW_VERSION, [2] DSI_CTRL,
// [3] COMMAND_MODE_DMA_CTRL, [4] TRIG_CTRL, [5] INT_CTRL, [6] VIDEO_MODE_ACTIVE_V, [7] CLK_CTRL, [8] commands sent, [9] commands that
// timed out, [10] last command's time (us), [11] last level written, [12] panel state (1 on, 0 off), [13] brightness requests from iOS,
// [14] last error, [15] the scheduled line
extern "C" {volatile int32_t native_panel_stats[16];}

namespace {
constexpr uint64_t kDsi0 = 0x0ae94000, kDsiBytes = 0x1000;
constexpr uint32_t kHwVersion = 0x000, kCtrl = 0x004, kActiveV = 0x028, kDmaCtrl = 0x03c, kDmaLength = 0x04c, kTrigCtrl = 0x084,
                   kSwTrigger = 0x090, kScheduleCtrl = 0x100, kScheduleCtrl2 = 0x104, kIntCtrl = 0x110, kClkCtrl = 0x11c,
                   kTpgCtrl = 0x15c, kTpgInit = 0x17c, kTpgFifoReset = 0x1ec;
constexpr uint32_t kMaxLevel = 4095, kMinLevel = 10, kTimeoutUs = 50000;

IOMemoryMap *gMap;
volatile uint32_t *gDsi;
IOLock *gLock;

uint32_t rd(uint32_t reg) {return gDsi[reg / 4];}
void wr(uint32_t reg, uint32_t value) {gDsi[reg / 4] = value; __asm__ volatile("dsb sy" ::: "memory");}
void fail(int step, int error) {native_panel_stats[14] = error; native_panel_stats[0] = -step;}

void snapshot() {
    native_panel_stats[1] = (int32_t)rd(kHwVersion);
    native_panel_stats[2] = (int32_t)rd(kCtrl);
    native_panel_stats[3] = (int32_t)rd(kDmaCtrl);
    native_panel_stats[4] = (int32_t)rd(kTrigCtrl);
    native_panel_stats[5] = (int32_t)rd(kIntCtrl);
    native_panel_stats[6] = (int32_t)rd(kActiveV);
    native_panel_stats[7] = (int32_t)rd(kClkCtrl);
}

// One packet through the TPG FIFO: words is the header word (data0, data1, data type, flags) and the payload words; bytes is what the
// controller sends (header plus the payload padded to a word). High speed unless lp. Returns 0, or -1 when DMA done never came.
int sendPacket(const uint32_t *words, unsigned count, bool lp) {
    const uint64_t began = mach_absolute_time();
    // Software trigger for the command DMA (DMA_TRG_MUX 0, DMA_TRIGGER_SEL 4).
    wr(kTrigCtrl, (rd(kTrigCtrl) & ~((1u << 16) | 0xfu)) | 0x4u);
    // The first line of the vertical front porch: VIDEO_MODE_ACTIVE_V holds the active area's end line in its upper half.
    const uint32_t line = ((rd(kActiveV) >> 16) & 0xffffu) + 1;
    native_panel_stats[15] = (int32_t)line;
    wr(kScheduleCtrl, (1u << 28) | line);
    wr(kCtrl, rd(kCtrl) | (1u << 2));                 // the command engine, beside the video engine
    wr(kIntCtrl, (rd(kIntCtrl) & ~((1u << 24) | (1u << 28))) | 1u);   // clear a stale DMA done (bit 0 is write-1-to-clear)
    wr(kTpgCtrl, (1u << 1) | (1u << 2) | (3u << 16));
    for (unsigned i = 0; i < count; ++i) wr(kTpgInit, words[i]);
    if (count & 1) wr(kTpgInit, 0);
    uint32_t dma = rd(kDmaCtrl) & ~((1u << 31) | (1u << 30));
    dma = lp ? (dma | (1u << 26)) : (dma & ~(1u << 26));
    wr(kDmaCtrl, dma | (1u << 28));
    wr(kDmaLength, count * 4);
    wr(kSwTrigger, 1);
    int rc = -1;
    for (uint32_t waited = 0; waited < kTimeoutUs; waited += 20) {
        if (rd(kIntCtrl) & 1u) {rc = 0; break;}
        IODelay(20);
    }
    wr(kIntCtrl, (rd(kIntCtrl) & ~((1u << 24) | (1u << 28))) | 1u);
    wr(kTpgCtrl, 0);
    wr(kTpgFifoReset, 1);
    IODelay(1);
    wr(kTpgFifoReset, 0);
    wr(kTrigCtrl, (rd(kTrigCtrl) & ~((1u << 16) | 0xfu)) | 0x4u);
    wr(kScheduleCtrl2, 0);
    wr(kScheduleCtrl, 0);
    uint64_t ns = 0;
    absolutetime_to_nanoseconds(mach_absolute_time() - began, &ns);
    native_panel_stats[10] = (int32_t)(ns / 1000);
    native_panel_stats[8] = native_panel_stats[8] + 1;
    if (rc) native_panel_stats[9] = native_panel_stats[9] + 1;
    return rc;
}

// DCS short write without a parameter (data type 0x05): header data0 = the command, last-command flag (bit 7).
int dcsShort(uint8_t command) {
    const uint32_t word = 0x80000000u | (0x05u << 16) | command;
    return sendPacket(&word, 1, false);
}
}  // namespace

// The panel's brightness, 0..4095 (values 1..9 become the panel's minimum 10): DCS long write (0x39) 51 hi lo, high speed.
extern "C" int asteroids_panel_level(uint32_t level) {
    if (!gDsi) return -1;
    if (level > kMaxLevel) level = kMaxLevel;
    if (level && level < kMinLevel) level = kMinLevel;
    const uint32_t words[2] = {0xc0000000u | (0x39u << 16) | 3u, 0x51u | ((level >> 8) & 0xffu) << 8 | (level & 0xffu) << 16 | 0xffu << 24};
    IOLockLock(gLock);
    const int rc = sendPacket(words, 2, false);
    IOLockUnlock(gLock);
    if (!rc) native_panel_stats[11] = (int32_t)level;
    return rc;
}

// Display off (28, then 120 ms) and sleep in (10, then 100 ms), or sleep out (11, then 80 ms) and display on (29, then 10 ms). The waits
// sleep the calling thread, so this runs on a thread or work loop, never in an interrupt.
extern "C" int asteroids_panel_power(bool on) {
    if (!gDsi) return -1;
    IOLockLock(gLock);
    int rc = dcsShort(on ? 0x11 : 0x28);
    IOSleep(on ? 80 : 120);
    rc |= dcsShort(on ? 0x29 : 0x10);
    IOSleep(on ? 10 : 100);
    IOLockUnlock(gLock);
    if (!rc) native_panel_stats[12] = on ? 1 : 0;
    return rc;
}

extern kmod_info_t g_kernel_kmod_info;
extern "C" void asteroids_backlight_static_init(void) __asm__("__GLOBAL__sub_I_AsteroidsBacklight.cpp");
extern "C" bool asteroids_backlight_start(void);

namespace {
unsigned gMode;
uint32_t gLevel = 2048;

void panelThread(void *, wait_result_t) {
    thread_set_thread_name(current_thread(), "asteroids-panel");
    if (gMode == 2) {
        asteroids_panel_level(kMaxLevel);
        IOSleep(2000);
        asteroids_panel_level(200);
        IOSleep(2000);
        asteroids_panel_level(gLevel);
        native_panel_stats[0] = 2;
    } else if (gMode == 4) {
        asteroids_panel_power(false);
        IOSleep(3000);
        asteroids_panel_power(true);
        asteroids_panel_level(kMaxLevel);
        native_panel_stats[0] = 2;
    } else if (gMode == 3) {
        // The service class registers with the kernel's module here (its static initializer is out of __mod_init_func), as sheng's
        // BacklightProbe.cpp does.
        const OSReturn status = OSKextRegisterBoardClasses(g_kernel_kmod_info.name, asteroids_backlight_static_init);
        if (status) {fail(5, status == kOSReturnNoMemory ? -12 : status); return;}
        if (!asteroids_backlight_start()) {fail(6, -19); return;}
        native_panel_stats[0] = 3;
    }
}
}  // namespace

extern "C" void asteroids_panel_probe(void) {
    unsigned mode = 0;
    if (!PE_parse_boot_argn("asteroids-panel", &mode, sizeof mode) || !mode || mode > 4 || native_panel_stats[0]) return;
    PE_parse_boot_argn("asteroids-panel-level", &gLevel, sizeof gLevel);
    if (gLevel > kMaxLevel) gLevel = kMaxLevel;
    gLock = IOLockAlloc();
    IODeviceMemory *memory = IODeviceMemory::withRange(kDsi0, kDsiBytes);
    gMap = memory ? memory->map(kIOMapAnywhere | kIOMapInhibitCache) : nullptr;
    if (memory) memory->release();
    gDsi = gMap ? reinterpret_cast<volatile uint32_t *>(gMap->getVirtualAddress()) : nullptr;
    if (!gLock || !gDsi) {gDsi = nullptr; fail(1, -12); return;}
    snapshot();
    native_panel_stats[12] = 1;
    native_panel_stats[0] = 1;
    if (mode == 1) return;
    gMode = mode;
    thread_t thread = nullptr;
    if (kernel_thread_start(panelThread, nullptr, &thread) == KERN_SUCCESS) thread_deallocate(thread);
    else fail(4, -12);
}
