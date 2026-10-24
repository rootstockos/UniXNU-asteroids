// SPDX-License-Identifier: BSD-2-Clause-Patent
// asteroids Adreno 810 power observation and CX power-up, modes 1-3, the GMU pool round trip, mode 4, and the GMU's SMMU
// context on that pool, mode 5 (sheng-gpu-power=1..5; absent or 0 does nothing). The shape of
// UniXNU-sheng's and Allenwdk/UniXNU-uke's GpuPowerProbe.cpp (BSD-2-Clause-Patent) without their GMU, ring and Metal stages:
// a kernel thread waits for the console, maps the five windows uncached, runs GpuPowerBoard.h's gated sequence and logs it.
// Every block is announced in the log before its first access, so a read that hangs the bus leaves its name as the last line.
// Writes are allowed only to gcc_gpu_cfg_ahb_clk (mode 2 and up) and to the CX GDSCR, gpu_cc_ahb_clk and
// gpu_cc_hlos1_vote_gpu_smmu_clk (mode 3); anything else is refused here as well. Mode 5 hands NativeAdreno's
// NativeAdrenoGmuContextBringup (uke's mode 4) its own uncached map of the whole SMMU window, and only when
// gpb_gmu_context_gate passes; the GMU itself stays in reset, so no transaction uses the context yet.
#include <IOKit/IOLib.h>
#include <IOKit/IOMemoryDescriptor.h>
#include <IOKit/IODeviceMemory.h>
#include <libkern/c++/OSSharedPtr.h>
#include <kern/thread.h>
#include <pexpert/pexpert.h>
extern "C" {
#include "GpuPowerBoard.h"
}
#include "NativeAdrenoGmuContext.h"

extern "C" {
struct gpb_result native_gpu_power_result;
struct gpb_pool_result native_gpu_pool_result;
struct NativeAdrenoGmuContextReport native_gpu_gmu_context;
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
constexpr const char *kGmuStep[] = {"-", "allocate", "prepare", "program", "attach", "vm attach", "map", "ready"};
void gmuProgress(uint32_t step) {
    // Before each step, so a hang or abort leaves the step as the last line.
    native_gpu_power_stage = 0x50 + step;
    IOLog("ASTEROIDS GPU: gmuctx step %u %s\n", step, step < 8 ? kGmuStep[step] : "?");
}
void gmuContext(const gpb_result &r) {
    const unsigned gate = gpb_gmu_context_gate(&r, &native_gpu_pool_result);
    if (gate) {IOLog("ASTEROIDS GPU: gmuctx skipped: %s (%u)\n", gpb_gmu_gate_text(gate), gate); return;}
    IOLog("ASTEROIDS GPU: gmuctx bank %u asid %u slot %u sid %u, mapping the SMMU 0x%llx + 0x%llx\n", GPB_GMU_BANK,
          GPB_GMU_ASID, GPB_GMU_SLOT, GPB_GMU_SID, GPB_SMMU_BASE, GPB_SMMU_WINDOW);
    native_gpu_power_stage = 0x50;
    IODeviceMemory *memory = IODeviceMemory::withRange(GPB_SMMU_BASE, GPB_SMMU_WINDOW);
    static OSSharedPtr<IOMemoryMap> smmu;   // the context bank points at tables in the pool for the rest of the boot
    if (memory) {
        smmu = OSSharedPtr<IOMemoryMap>(memory->map(kIOMapAnywhere | kIOMapInhibitCache), OSNoRetain);
        memory->release();
    }
    if (!smmu) {IOLog("ASTEROIDS GPU: gmuctx SMMU map failed\n"); return;}
    volatile uint8_t *va = reinterpret_cast<volatile uint8_t *>(smmu->getVirtualAddress());
    NativeAdrenoGmuContextReport &g = native_gpu_gmu_context;
    const IOReturn status = NativeAdrenoGmuContextBringup(va, GPB_SMMU_WINDOW, GPB_GMU_BANK, GPB_GMU_ASID, GPB_GMU_SLOT,
                                                          GPB_GMU_SID, gmuProgress, &g);
    // s<step> st<IOReturn> k<kit program status>/<attach status> p<phase>w<writes>r<readback>e<enabled> r<phase>w<writes>v<verified>
    IOLog("ASTEROIDS GPU: gmuctx s%u %s st=%x k%u/%u id2=%x oas=%u p%uw%ur%ue%u r%uw%uv%u spans=%u mismatch=%03x\n", g.step,
          g.step < 8 ? kGmuStep[g.step] : "?", static_cast<unsigned>(status), g.program_status, g.route_status, g.id2,
          g.output_bits, g.program_phase, g.program_writes, g.program_readback_valid, g.program_enabled, g.route_phase,
          g.route_writes, g.routes_verified, g.spans, g.mismatch);
    IOLog("ASTEROIDS GPU: gmuctx ttbr0=%016llx sctlr=%08x cbar=%08x smr=%08x s2cr=%08x\n",
          static_cast<unsigned long long>(g.ttbr0), g.sctlr, g.cbar, g.smr, g.s2cr);
    if (g.mismatch) {
        const ArmSmmu4KContextImage &v = g.image;
        IOLog("ASTEROIDS GPU: gmuctx readback cbar=%08x cba2r=%08x tcr=%08x tcr2=%08x mair=%08x/%08x ttbr1=%016llx sctlr=%08x\n",
              v.cbar, v.cba2r, v.tcr, v.tcr2, v.mair0, v.mair1, static_cast<unsigned long long>(v.ttbr1), v.sctlr);
    }
    // Fault status after the bring-up: global (sGFSR) and the GMU bank's FSR. Both should stay 0 with the GMU halted.
    const uint32_t gfsr = *reinterpret_cast<volatile uint32_t *>(va + GPB_SMMU_GFSR);
    const uint32_t fsr = *reinterpret_cast<volatile uint32_t *>(va + gpb_smmu_cb_offset(r.id1, GPB_GMU_BANK) + GPB_SMMU_CB_FSR);
    IOLog("ASTEROIDS GPU: gmuctx after gfsr=%08x fsr=%08x\n", gfsr, fsr);
    NativeAdrenoGmuPayload payloads[5];
    if (NativeAdrenoGmuContextPayloads(payloads) == kIOReturnSuccess)
        IOLog("ASTEROIDS GPU: gmuctx icache %llx+%llx@%llx hfi %llx+%llx@%llx (pool 0x%llx)\n", payloads[0].physical,
              payloads[0].bytes, payloads[0].iova, payloads[4].physical, payloads[4].bytes, payloads[4].iova, GPB_GMU_POOL_BASE);
    native_gpu_power_stage = status ? 0x5f : 0x57;
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
    if (r.smmu == GPB_DONE) {
        IOLog("ASTEROIDS GPU: smr n=%u valid=%u\n", r.num_smr, r.smr_valid);
        for (unsigned i = 0; i < r.num_smr; i += 3)
            IOLog("ASTEROIDS GPU: smr%u %08x/%08x %08x/%08x %08x/%08x\n", i, r.smr[i], r.s2cr[i],
                  i + 1 < r.num_smr ? r.smr[i + 1] : 0, i + 1 < r.num_smr ? r.s2cr[i + 1] : 0,
                  i + 2 < r.num_smr ? r.smr[i + 2] : 0, i + 2 < r.num_smr ? r.s2cr[i + 2] : 0);
    }
    if (gMode >= 3)
        IOLog("ASTEROIDS GPU: m3 cx %08x>%08x>%08x hw=%08x tries=%u refused=%u pw%u%u ahb %08x>%08x smmu %08x>%08x\n",
              r.cx_before, r.cx_init, r.cx_after, r.cx_hwctrl_after, r.cx_tries, r.cx_refused, r.powered_cx, r.powered_clocks,
              r.cc_ahb_before, r.cc_ahb, r.cc_smmu_before, r.cc_smmu);
    if (gMode >= 4) {
        // The GMU pool, reserved by the firmware (GMU_Pool): map it write-combined (uke's fix for the DC ZVA abort on the
        // default mapping), fill and check it with plain 8-byte stores, then zero and check it. No SMMU or GMU access.
        IOLog("ASTEROIDS GPU: mapping the GMU pool 0x%llx + 0x%llx\n", GPB_GMU_POOL_BASE, GPB_GMU_POOL_BYTES);
        native_gpu_power_stage = 0x40;
        IODeviceMemory *memory = IODeviceMemory::withRange(GPB_GMU_POOL_BASE, GPB_GMU_POOL_BYTES);
        OSSharedPtr<IOMemoryMap> pool;
        if (memory) {
            pool = OSSharedPtr<IOMemoryMap>(memory->map(kIOMapAnywhere | kIOMapWriteCombineCache), OSNoRetain);
            memory->release();
        }
        if (!pool) IOLog("ASTEROIDS GPU: pool map failed\n");
        else {
            IOLog("ASTEROIDS GPU: pool mapped, writing\n");
            native_gpu_power_stage = 0x41;
            gpb_pool_result &p = native_gpu_pool_result;
            gpb_pool_test(reinterpret_cast<volatile uint64_t *>(pool->getVirtualAddress()), GPB_GMU_POOL_BYTES, &p);
            __asm__ volatile("dsb sy" ::: "memory");
            IOLog("ASTEROIDS GPU: pool words=%llu pattern_bad=%llu zero_bad=%llu first_bad=%llx\n", p.words, p.pattern_bad,
                  p.zero_bad, p.first_bad);
            native_gpu_power_stage = 0x42;
        }
        // The context adopts the same pages with a mapping of its own; this one goes first.
        pool.reset();
    }
    if (gMode >= 5) gmuContext(r);
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
