// SPDX-License-Identifier: BSD-2-Clause-Patent
// asteroids Adreno 810 power observation and CX power-up, modes 1-3 (sheng-gpu-power=1..3; absent or 0 does nothing). The shape of
// UniXNU-sheng's and Allenwdk/UniXNU-uke's GpuPowerProbe.cpp (BSD-2-Clause-Patent) without their GMU, ring and Metal stages:
// a kernel thread waits for the console, maps the five windows uncached, runs GpuPowerBoard.h's gated sequence and logs it.
// Every block is announced in the log before its first access, so a read that hangs the bus leaves its name as the last line.
// Writes are allowed only to gcc_gpu_cfg_ahb_clk (mode 2 and up) and to the CX GDSCR, gpu_cc_ahb_clk and
// gpu_cc_hlos1_vote_gpu_smmu_clk (mode 3); anything else is refused here as well.
#include <IOKit/IOLib.h>
#include <IOKit/IOMemoryDescriptor.h>
#include <IOKit/IODeviceMemory.h>
#include <libkern/c++/OSSharedPtr.h>
#include <kern/thread.h>
#include <pexpert/pexpert.h>
extern "C" {
#include "GpuPowerBoard.h"
}

extern "C" {
struct gpb_result native_gpu_power_result;
volatile uint32_t native_gpu_power_stage;
}

namespace {
constexpr uint64_t kPhys[GPB_WINDOWS] = {GPB_GCC_GPU_PAGE, GPB_GCC_VOTE_PAGE, GPB_GPUCC_BASE, GPB_SMMU_BASE, GPB_GPU_BASE};
constexpr uint64_t kLen[GPB_WINDOWS] = {0x1000, 0x1000, GPB_GPUCC_SIZE, GPB_SMMU_GR0_SIZE, GPB_GPU_WINDOW};
constexpr const char *kName[GPB_WINDOWS] = {"GCC GPU page", "GCC vote page", "GPUCC", "GPU SMMU GR0", "GPU RBBM"};
OSSharedPtr<IOMemoryMap> gMaps[GPB_WINDOWS];
uintptr_t gVa[GPB_WINDOWS];
unsigned gMode;

int rd(void *, unsigned w, uint32_t off, uint32_t *out) {
    if (w >= GPB_WINDOWS || !gVa[w] || off >= kLen[w] || (off & 3)) return -1;
    *out = *reinterpret_cast<volatile uint32_t *>(gVa[w] + off);
    return 0;
}
int wr(void *, unsigned w, uint32_t off, uint32_t value) {
    const bool gcc = gMode >= 2 && w == GPB_WIN_GCC_GPU && off == GPB_GCC_GPU_CFG_AHB;
    const bool cc = gMode >= 3 && w == GPB_WIN_GPUCC &&
                    (off == GPB_CC_CX_GDSCR || off == GPB_CC_AHB_CLK || off == GPB_CC_SMMU_VOTE_CLK);
    if (!(gcc || cc) || !gVa[w]) return -1;
    *reinterpret_cast<volatile uint32_t *>(gVa[w] + off) = value;
    __asm__ volatile("dsb sy" ::: "memory");
    return 0;
}
void delayUs(void *, unsigned us) {IODelay(us);}
void stage(void *, unsigned w) {
    native_gpu_power_stage = 0x10 + w;
    if (w == GPB_WINDOWS) {IOLog("ASTEROIDS GPU: powering the CX GDSC (GPUCC 0x9080)\n"); return;}
    IOLog("ASTEROIDS GPU: reading %s (0x%llx)\n", w < GPB_WINDOWS ? kName[w] : "?", w < GPB_WINDOWS ? kPhys[w] : 0ULL);
}
void probeThread(void *, wait_result_t) {
    IOSleep(10000);   // after the console and the late probes, as uke does
    native_gpu_power_stage = 2;
    unsigned have = 0;
    for (unsigned w = 0; w < GPB_WINDOWS; ++w) {
        IODeviceMemory *memory = IODeviceMemory::withRange(kPhys[w], kLen[w]);
        if (!memory) continue;
        gMaps[w] = OSSharedPtr<IOMemoryMap>(memory->map(kIOMapAnywhere | kIOMapInhibitCache), OSNoRetain);
        memory->release();
        if (gMaps[w]) {gVa[w] = static_cast<uintptr_t>(gMaps[w]->getVirtualAddress()); have |= 1u << w;}
    }
    IOLog("ASTEROIDS GPU: mode %u, windows mapped %02x\n", gMode, have);
    gpb_io io{nullptr, rd, wr, stage, delayUs};
    gpb_result &r = native_gpu_power_result;
    gpb_run(&io, gMode, have, &r);
    native_gpu_power_stage = 3;
    IOLog("ASTEROIDS GPU: m%u gcc=%s cc=%s sm=%s gp=%s io=%d writes=%u ahb_set=%u\n", r.mode, gpb_state_text(r.gcc),
          gpb_state_text(r.cc), gpb_state_text(r.smmu), gpb_state_text(r.gpu), r.io_error, r.writes, r.enabled_ahb);
    IOLog("ASTEROIDS GPU: gcc ahb=%08x>%08x mem=%08x dvm=%08x axi=%08x vote=%08x\n", r.cfg_ahb, r.cfg_ahb_after, r.memnoc,
          r.dvm, r.axi, r.pll_vote);
    IOLog("ASTEROIDS GPU: cc cx=%08x hw=%08x gx=%08x ahb=%08x smmu=%08x gmu=%08x\n", r.cx_gdscr, r.cx_hwctrl, r.gx_gdscr,
          r.cc_ahb, r.cc_smmu, r.cc_gmu);
    IOLog("ASTEROIDS GPU: smmu %08x %08x %08x %08x gfsr=%08x rbbm=%08x\n", r.scr0, r.id0, r.id1, r.id2, r.gfsr, r.rbbm_status);
    if (gMode >= 3)
        IOLog("ASTEROIDS GPU: m3 cx %08x>%08x>%08x hw=%08x tries=%u refused=%u pw%u%u ahb %08x>%08x smmu %08x>%08x\n",
              r.cx_before, r.cx_init, r.cx_after, r.cx_hwctrl_after, r.cx_tries, r.cx_refused, r.powered_cx, r.powered_clocks,
              r.cc_ahb_before, r.cc_ahb, r.cc_smmu_before, r.cc_smmu);
    native_gpu_power_stage = 4;
}
}

extern "C" void sheng_gpu_power_probe(void) {
    unsigned mode = 0;
    if (!PE_parse_boot_argn("sheng-gpu-power", &mode, sizeof mode) || !mode || native_gpu_power_stage) return;
    if (mode > GPB_MAX_MODE) {
        IOLog("ASTEROIDS GPU: sheng-gpu-power=%u refused; this build has modes 1-%u only\n", mode, GPB_MAX_MODE);
        native_gpu_power_stage = 0xbad;
        return;
    }
    gMode = mode;
    native_gpu_power_stage = 1;
    thread_t thread = nullptr;
    if (kernel_thread_start(probeThread, nullptr, &thread) == KERN_SUCCESS) thread_deallocate(thread);
    else native_gpu_power_stage = 0xdead;
}
