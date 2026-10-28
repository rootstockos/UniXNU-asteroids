// SPDX-License-Identifier: BSD-2-Clause-Patent
// asteroids Adreno 810 power observation and CX power-up, modes 1-3, the GMU pool round trip, mode 4, the GMU's SMMU
// context on that pool, mode 5, the read-only GMU clock and command-DB snapshot, mode 6, the GMU clock bring-up, mode 7,
// the first GMU and RSCC register reads, mode 8, the GMU firmware load to HFI ready, mode 9, and the whole cold start with
// the HFI startup messages, mode 10, the GX request with the first RBBM reads, mode 11, the SW fuse and CP survey with
// the GPU's own SMMU context and the SQE copy, mode 12, a8xx hw_init's register writes, mode 13, and the CP start through
// CP_ME_INIT and a first CP_MEM_WRITE, mode 14, the zap shader with the secure-mode exit, mode 15, and the first IBs with a
// bar the GPU writes into the firmware framebuffer, mode 16, and the BO pool, fences, timer and NativeAdreno's user
// client run from the kernel, mode 17 (sheng-gpu-power=1..17;
// absent or 0 does nothing). Modes 9-10 hand the
// device package's ShengGmu.c (UniXNU-sheng's, with mainline's gen8 branches) its own uncached maps of the GMU, RSCC, GPU and CX_MISC
// windows and Normal non-cacheable maps of the five payloads mode 5 placed in the pool; its writes go through its own
// MMIO adapter (ShengGmuBindMmio), not this file's wr(). The shape of
// UniXNU-sheng's and Allenwdk/UniXNU-uke's GpuPowerProbe.cpp (BSD-2-Clause-Patent) without their GMU, ring and Metal stages:
// a kernel thread waits for the console, maps the five windows uncached, runs GpuPowerBoard.h's gated sequence and logs it.
// Every block is announced in the log before its first access, so a read that hangs the bus leaves its name as the last line.
// Writes are allowed only to gcc_gpu_cfg_ahb_clk (mode 2 and up) and to the CX GDSCR, gpu_cc_ahb_clk and
// gpu_cc_hlos1_vote_gpu_smmu_clk (mode 3), and, only while mode 7's gpb_gmu_clocks_on runs, to gpb_mode7_writable's
// GPLL0 vote, PLL0, two RCGs and branches; anything else is refused here as well. Mode 5 hands NativeAdreno's
// NativeAdrenoGmuContextBringup (uke's mode 4) its own uncached map of the whole SMMU window, and only when
// gpb_gmu_context_gate passes; the GMU itself stays in reset, so no transaction uses the context yet. Modes 9-15 write
// through ShengGmu's adapter instead: the GMU from mode 9, the GPU block only from mode 13 (gGpuWrites), CX_MISC never.
#include <IOKit/IOLib.h>
#include <IOKit/IOMemoryDescriptor.h>
#include <IOKit/IODeviceMemory.h>
#include <IOKit/IOPlatformExpert.h>
#include <libkern/c++/OSSharedPtr.h>
#include <libkern/c++/OSMetaClass.h>
#include <libkern/c++/OSSymbol.h>
#include <kern/thread.h>
#include <pexpert/pexpert.h>
#include <string.h>
extern "C" {
#include "GpuPowerBoard.h"
}
#include "NativeAdrenoGmuContext.h"
#include "ShengGmuCommandDb.h"
#include "ShengGmuCommandDbSource.h"
#include "ShengGmu.h"
#include "ShengGmuSource.h"     // build/sheng-gmu: generated registers and windows (prepare-asteroids-gmu.py)
#include "ShengGmuFirmware.h"   // build/sheng-gmu: the pinned gen80300_gmu.bin as an array; never committed
#include "NativeAdrenoSha256.h"
#include "NativeAdrenoGpuContext.h"
#include "NativeAdrenoDiag.h"
#include "NativeAdrenoRing.h"
#include "NativeAdrenoCpInit.h"
#include "NativeAdrenoPwrup.h"
#include "AdrenoPM4.h"
#include "ShengSqeFirmware.h"   // build/sheng-gmu: the pinned gen80300_sqe.fw as an array; never committed
#include "ShengZapFirmware.h"   // build/sheng-gmu: the pinned gen80300_zap.mbn as an array; never committed
#include "NativeAdrenoMemory.h"
#include "NativeAdrenoUserClient.h"   // with NativeAdrenoSubmit.h and NativeAdrenoTimer.h
#include "NativeAdrenoABI.h"
#include <IOKit/IOUserClient.h>
#include <IOKit/IOBufferMemoryDescriptor.h>
#include <kern/task.h>
#include <kern/clock.h>

extern "C" {
struct gpb_result native_gpu_power_result;
struct gpb_pool_result native_gpu_pool_result;
struct NativeAdrenoGmuContextReport native_gpu_gmu_context;
volatile uint32_t native_gpu_power_stage;
}

namespace {
// The first GPB_WINDOWS are mapped at start; the GMU and RSCC windows only by mode 8.
constexpr uint64_t kPhys[GPB_ALL_WINDOWS] = {GPB_GCC_GPU_PAGE, GPB_GCC_VOTE_PAGE, GPB_GPUCC_BASE, GPB_SMMU_BASE, GPB_GPU_BASE,
                                             GPB_GMU_BASE, GPB_RSCC_BASE};
constexpr uint64_t kLen[GPB_ALL_WINDOWS] = {0x1000, 0x1000, GPB_GPUCC_SIZE, GPB_SMMU_GR0_SIZE, GPB_GPU_WINDOW, GPB_GMU_SIZE,
                                            GPB_RSCC_SIZE};
constexpr const char *kName[GPB_ALL_WINDOWS] = {"GCC GPU page", "GCC vote page", "GPUCC", "GPU SMMU GR0", "GPU RBBM", "GMU",
                                                "GMU RSCC"};
OSSharedPtr<IOMemoryMap> gMaps[GPB_ALL_WINDOWS];
uintptr_t gVa[GPB_ALL_WINDOWS];
unsigned gMode;

int rd(void *, unsigned w, uint32_t off, uint32_t *out) {
    if (w >= GPB_ALL_WINDOWS || !gVa[w] || off >= kLen[w] || (off & 3)) return -1;
    *out = *reinterpret_cast<volatile uint32_t *>(gVa[w] + off);
    return 0;
}
bool gClocksOn;
int wr(void *, unsigned w, uint32_t off, uint32_t value) {
    const bool gcc = gMode >= 2 && w == GPB_WIN_GCC_GPU && off == GPB_GCC_GPU_CFG_AHB;
    const bool cc = gMode >= 3 && w == GPB_WIN_GPUCC &&
                    (off == GPB_CC_CX_GDSCR || off == GPB_CC_AHB_CLK || off == GPB_CC_SMMU_VOTE_CLK);
    const bool clocks = gMode >= 7 && gClocksOn && gpb_mode7_writable(w, off);
    if (!(gcc || cc || clocks) || !gVa[w]) return -1;
    *reinterpret_cast<volatile uint32_t *>(gVa[w] + off) = value;
    __asm__ volatile("dsb sy" ::: "memory");
    return 0;
}
void delayUs(void *, unsigned us) {IODelay(us);}
void stage(void *, unsigned w) {
    native_gpu_power_stage = 0x10 + w;
    if (w == GPB_STAGE_CX_GDSC) {IOLog("ASTEROIDS GPU: powering the CX GDSC (GPUCC 0x9080)\n"); return;}
    IOLog("ASTEROIDS GPU: reading %s (0x%llx)\n", w < GPB_ALL_WINDOWS ? kName[w] : "?", w < GPB_ALL_WINDOWS ? kPhys[w] : 0ULL);
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
// Mode 6 (read-only). The GMU clock tree (gpb_read_gmu_clocks), then the AOP command DB: the live DT's
// /reserved-memory/aop_cmd_db_region@81c60000 (no-map, 128 KiB; the UEFI map's AOP region 0x81c00000 + 0xa0000, uncached),
// copied with aligned 32-bit reads from a read-only uncached map and parsed by ShengGmuCommandDbInputs into the HFI startup
// table by ShengGmuBuildStartup (the A8xx GPU_PERF table with dependency votes). The version passed to the parser is the
// word just read, as uke does, so a pass shows the layout, not a qualified version. No GMU register is read: the GMU's
// clock is off until mode 7 turns it on.
constexpr const char *kDbStatus[] = {"ok", "argument", "version", "corrupt", "duplicate", "missing"};
alignas(8) uint8_t gCmdDb[SHENG_GMU_COMMAND_DB_BYTES];
ShengGmuVoteInputs gVoteInputs;
void logGmuClocks(const char *when, const gpb_gmu_clocks &k, int io_error) {
    IOLog("ASTEROIDS GPU: gmuclk %s pll0 mode=%08x op=%x st=%08x l=%08x a=%08x user=%08x io=%d\n", when, k.pll0_mode,
          k.pll0_opmode, k.pll0_status, k.pll0_l, k.pll0_alpha, k.pll0_user, io_error);
    IOLog("ASTEROIDS GPU: gmuclk %s rcg gmu=%08x/%08x hub=%08x/%08x div=%08x ff=%08x/%08x\n", when, k.gmu_cmd, k.gmu_cfg,
          k.hub_cmd, k.hub_cfg, k.hub_div, k.ff_cmd, k.ff_cfg);
    // Branch digits: 1 on, 3 enabled but off, 2 disabled and off, 0 disabled and on.
    IOLog("ASTEROIDS GPU: gmuclk %s br gmu=%08x cxo=%08x ahb=%08x hubcx=%08x hubaon=%08x memnoc=%08x\n", when, k.branch[0],
          k.branch[1], k.branch[2], k.branch[3], k.branch[4], k.branch[5]);
    IOLog("ASTEROIDS GPU: gmuclk %s aon %u%u%u%u%u%u%u gcc axi=%08x mem=%08x vote=%08x\n", when,
          gpb_branch_digit(k.critical[0]), gpb_branch_digit(k.critical[1]), gpb_branch_digit(k.critical[2]),
          gpb_branch_digit(k.critical[3]), gpb_branch_digit(k.critical[4]), gpb_branch_digit(k.critical[5]),
          gpb_branch_digit(k.critical[6]), k.axi, k.memnoc, k.gpll0_vote);
}
AdrenoHfiStartupConfig gStartup;
extern "C" {
struct gpb_gmu_clocks native_gpu_gmu_clocks;
uint32_t native_gpu_cmd_db_status[4];   // version, magic, parse status, build status
}
void gmuPrerequisites(const gpb_result &r) {
    gpb_io io{nullptr, rd, wr, stage, delayUs};
    gpb_result scratch = r;   // gpb_rd records io_error into this copy, not into the published result
    gpb_gmu_clocks &k = native_gpu_gmu_clocks;
    native_gpu_power_stage = 0x60;
    IOLog("ASTEROIDS GPU: reading the GMU clock tree (GPUCC PLL0, RCGs, branches)\n");
    gpb_read_gmu_clocks(&io, &scratch, &k);
    if (!k.done) IOLog("ASTEROIDS GPU: gmuclk skipped (GPUCC not read)\n");
    else {
        logGmuClocks("before", k, scratch.io_error);
    }
    native_gpu_power_stage = 0x61;
    IOLog("ASTEROIDS GPU: mapping the command DB 0x%llx + 0x%x\n", static_cast<unsigned long long>(SHENG_GMU_COMMAND_DB_BASE),
          SHENG_GMU_COMMAND_DB_BYTES);
    IODeviceMemory *memory = IODeviceMemory::withRange(SHENG_GMU_COMMAND_DB_BASE, SHENG_GMU_COMMAND_DB_BYTES);
    OSSharedPtr<IOMemoryMap> map;
    if (memory) {
        map = OSSharedPtr<IOMemoryMap>(memory->map(kIOMapAnywhere | kIOMapInhibitCache | kIOMapReadOnly), OSNoRetain);
        memory->release();
    }
    if (!map) {IOLog("ASTEROIDS GPU: cmddb map failed\n"); return;}
    const volatile uint32_t *db = reinterpret_cast<const volatile uint32_t *>(map->getVirtualAddress());
    for (uint32_t i = 0; i < SHENG_GMU_COMMAND_DB_BYTES / 4; ++i) {const uint32_t w = db[i]; memcpy(gCmdDb + 4 * i, &w, 4);}
    map.reset();
    uint32_t version = 0, magic = 0;
    memcpy(&version, gCmdDb, 4);
    memcpy(&magic, gCmdDb + 4, 4);
    gVoteInputs = {};
    gVoteInputs.chip_id = 0x44010000u;   // A810 (board.toml [gpu] chip_id); sku_present 0: the bin listing 0 (KGSL's rule)
    const unsigned parse = ShengGmuCommandDbInputs(gCmdDb, SHENG_GMU_COMMAND_DB_BYTES, version, &gVoteInputs);
    unsigned build = ~0u;
    if (parse == ShengGmuDbSuccess) build = ShengGmuBuildStartup(&gVoteInputs, &gStartup);
    native_gpu_cmd_db_status[0] = version; native_gpu_cmd_db_status[1] = magic;
    native_gpu_cmd_db_status[2] = parse; native_gpu_cmd_db_status[3] = build;
    IOLog("ASTEROIDS GPU: cmddb v=%x magic=%08x parse=%u %s build=%d gmxc=%u aux gfx=%u cx=%u mx=%u sh0=%u mc0=%u acv=%u "
          "cn0=%x\n", version, magic, parse, parse < 6 ? kDbStatus[parse] : "?", static_cast<int>(build),
          gVoteInputs.gmxc_present, unsigned(gVoteInputs.gfx.bytes), unsigned(gVoteInputs.cx.bytes),
          unsigned(gVoteInputs.mx.bytes), unsigned(gVoteInputs.sh0.aux.bytes), unsigned(gVoteInputs.mc0.aux.bytes),
          unsigned(gVoteInputs.acv.aux.bytes), gVoteInputs.cn0_address);
    if (parse != ShengGmuDbSuccess) {
        // The slave table (cmd_db_header.header[i], 16 bytes from offset 8: le16 slv_id, ..., le16 cnt at +6).
        for (unsigned i = 0; i < 8; ++i) {
            uint16_t id, cnt;
            memcpy(&id, gCmdDb + 8 + 16 * i, 2);
            memcpy(&cnt, gCmdDb + 8 + 16 * i + 6, 2);
            if (!id) break;
            IOLog("ASTEROIDS GPU: cmddb slave %u id=%x cnt=%u\n", i, id, cnt);
        }
        return;
    }
    if (build) return;
    const AdrenoHfiStartupConfig &c = gStartup;
    IOLog("ASTEROIDS GPU: hfi table=%u gpu=%u gmu=%u bw=%u cnoc=%u ddr=%u ifpc=%u\n", c.table_format, c.gpu_count,
          c.gmu_count, c.bandwidth_count, c.cnoc_count, c.ddr_count, c.enable_ifpc);
    for (unsigned i = 0; i < c.gpu_count && i < 16; ++i)
        IOLog("ASTEROIDS GPU: hfi gpu%u %lluk vote=%08x dep=%08x\n", i, static_cast<unsigned long long>(c.gpu[i].hz / 1000),
              c.gpu[i].vote, c.gpu_dependency[i]);
    for (unsigned i = 0; i < c.gmu_count && i < 4; ++i)
        IOLog("ASTEROIDS GPU: hfi gmu%u %lluk vote=%08x\n", i, static_cast<unsigned long long>(c.gmu[i].hz / 1000),
              c.gmu[i].vote);
    native_gpu_power_stage = 0x6f;
}
extern "C" {
struct gpb_gmu_clock_on native_gpu_gmu_clock_on;
}
// Mode 7: gpb_gmu_clocks_on from mode 6's snapshot, then the same snapshot again. The GMU stays in reset.
void gmuClocksOn(const gpb_result &r) {
    gpb_io io{nullptr, rd, wr, stage, delayUs};
    gpb_result scratch = r;
    gpb_gmu_clock_on &o = native_gpu_gmu_clock_on;
    native_gpu_power_stage = 0x70;
    IOLog("ASTEROIDS GPU: gmuclk on: GPLL0 vote, PLL0 700 MHz, hub 200 MHz, gmu 350 MHz, gpu_cc_ahb\n");
    gClocksOn = true;
    gpb_gmu_clocks_on(&io, &scratch, &native_gpu_gmu_clocks, &o);
    gClocksOn = false;
    native_gpu_power_stage = 0x71;
    static const char *const kFail[] = {"ok", "refused", "timeout", "io"};
    IOLog("ASTEROIDS GPU: gmuclk on step=%u %s writes=%u running=%u lock=%u hub=%u/%u gmu=%u/%u\n", o.step,
          o.failed < 4 ? kFail[o.failed] : "?", o.writes, o.pll_was_running, o.lock_polls, o.hub_root_polls, o.hub_polls,
          o.gmu_root_polls, o.gmu_polls);
    IOLog("ASTEROIDS GPU: gmuclk on vote=%08x pll0=%08x/%x/%08x hub=%08x/%08x gmu=%08x/%08x ahb=%08x\n", o.vote,
          o.pll0_mode, o.pll0_opmode, o.pll0_user, o.hub_cmd, o.hub_cfg, o.gmu_cmd, o.gmu_cfg, o.ahb);
    gpb_gmu_clocks after;
    gpb_read_gmu_clocks(&io, &scratch, &after);
    if (after.done) logGmuClocks("after", after, scratch.io_error);
    native_gpu_power_stage = 0x72;
}
extern "C" {
struct gpb_gmu_state native_gpu_gmu_state;
}
// Mode 8: map the GMU and RSCC windows uncached and read the cold start's preconditions. Each window is announced before
// its first read, so a read that hangs leaves its name last. Nothing is written.
void gmuState(const gpb_result &r) {
    const gpb_gmu_clock_on &o = native_gpu_gmu_clock_on;
    const int clocks_ok = o.step == 8 && !o.failed;
    native_gpu_power_stage = 0x80;
    for (unsigned w = GPB_WINDOWS; w < GPB_ALL_WINDOWS && clocks_ok; ++w) {
        IODeviceMemory *memory = IODeviceMemory::withRange(kPhys[w], kLen[w]);
        if (!memory) continue;
        gMaps[w] = OSSharedPtr<IOMemoryMap>(memory->map(kIOMapAnywhere | kIOMapInhibitCache), OSNoRetain);
        memory->release();
        if (gMaps[w]) gVa[w] = static_cast<uintptr_t>(gMaps[w]->getVirtualAddress());
    }
    if (clocks_ok && (!gVa[GPB_WIN_GMU] || !gVa[GPB_WIN_RSCC])) {
        IOLog("ASTEROIDS GPU: gmu map failed gmu=%d rscc=%d\n", gVa[GPB_WIN_GMU] != 0, gVa[GPB_WIN_RSCC] != 0);
        return;
    }
    gpb_io io{nullptr, rd, wr, stage, delayUs};
    gpb_result scratch = r;
    gpb_gmu_state &g = native_gpu_gmu_state;
    gpb_read_gmu_state(&io, &scratch, clocks_ok, &g);
    native_gpu_power_stage = 0x81;
    const uint32_t *v = g.v;
    IOLog("ASTEROIDS GPU: gmu gate=%s reset=%08x hfi=%08x ao=%08x cm3=%08x rscc=%08x io=%d\n", gpb_gmu_state_text(g.gate),
          v[0], v[1], v[2], v[3], g.rscc_busy, g.io_error);
    if (g.done && !g.io_error) {
        IOLog("ASTEROIDS GPU: gmu init=%08x boot=%08x cfg=%08x gen8=%08x gen9=%08x gen10=%08x fence=%08x\n", v[4], v[5], v[6],
              v[7], v[8], v[9], v[10]);
        IOLog("ASTEROIDS GPU: gmu itcm0=%08x dtcm_ver=%08x ao=%08x:%08x then %08x (+%u)\n", v[11], v[12], v[14], v[13], v[15],
              v[15] - v[13]);
    }
    native_gpu_power_stage = 0x82;
}
extern "C" {
ShengGmuStartupResult native_gpu_gmu_start;
uint32_t native_gpu_gmu_start_status = 0xffffffffu;
}
int gmuSha256(void *, const void *bytes, size_t length, uint8_t digest[32]) {
    NativeAdrenoSha256::hash(static_cast<const uint8_t *>(bytes), length, digest);
    return 0;
}
int gmuNowUs(void *, uint64_t *out) {
    uint64_t ns = 0;
    absolutetime_to_nanoseconds(mach_absolute_time(), &ns);
    *out = ns / 1000;
    return 0;
}
int gmuWaitUs(void *, unsigned us) {IODelay(us); return 0;}
constexpr const char *kShengStatus[] = {"ok", "argument", "firmware", "busy", "io", "state-mismatch", "timeout", "clock",
                                        "poisoned", "fault", "protocol"};
// Modes 9-10: only when mode 8 read a clean GMU (gate ok) on top of a ready GMU context, and for mode 10 a valid HFI
// startup table from mode 6. Mode 9 stops at HFI ready (ShengGmuLoadAndStartHfi); mode 10 also exchanges the startup
// messages (ShengGmuColdStart: perf table, bandwidth table, core start, start). Every map and the device stay alive:
// the GMU keeps using them after this returns. On failure ShengGmu reasserts CM3 reset itself.
ShengGmuMmio gColdMmio;       // the cold start's windows and device; mode 11 keeps using them
ShengGmuDevice gColdDevice;
void gmuColdStart() {
    native_gpu_power_stage = 0x90;
    const gpb_gmu_state &g = native_gpu_gmu_state;
    NativeAdrenoGmuPayload payloads[5];
    const bool ready = g.done && !g.gate && NativeAdrenoGmuContextPayloads(payloads) == kIOReturnSuccess;
    // Mode 6 parsed the command DB (its magic read back) and built the startup table from it.
    const bool table = native_gpu_cmd_db_status[1] == 0x0c0330dbu && native_gpu_cmd_db_status[2] == 0 &&
                       native_gpu_cmd_db_status[3] == 0;
    if (!ready || (gMode >= 10 && !table)) {
        IOLog("ASTEROIDS GPU: gmuboot skipped: gate=%s context=%d table=%d\n", gpb_gmu_state_text(g.gate), ready, table);
        return;
    }
    // GMU, RSCC (mode 8's maps), no PDC on A8xx, the GPU block (GBIF) and CX_MISC, in ShengGmu window order.
    static OSSharedPtr<IOMemoryMap> gpuMap, cxMap, payloadMap[5];
    ShengGmuMmio &mmio = gColdMmio;
    ShengGmuDevice &device = gColdDevice;
    const unsigned extra[2] = {ShengGmuGpu, ShengGmuCxMisc};
    OSSharedPtr<IOMemoryMap> *extraMap[2] = {&gpuMap, &cxMap};
    for (unsigned i = 0; i < 2; ++i) {
        const unsigned w = extra[i];
        IODeviceMemory *memory = IODeviceMemory::withRange(sheng_gmu_window_physical[w], sheng_gmu_window_bytes[w]);
        if (memory) {
            *extraMap[i] = OSSharedPtr<IOMemoryMap>(memory->map(kIOMapAnywhere | kIOMapInhibitCache), OSNoRetain);
            memory->release();
        }
        if (!*extraMap[i]) {IOLog("ASTEROIDS GPU: gmuboot map of window %u failed\n", w); return;}
        mmio.windows[w] = {reinterpret_cast<volatile unsigned char *>((*extraMap[i])->getVirtualAddress()),
                           sheng_gmu_window_physical[w], static_cast<size_t>(sheng_gmu_window_bytes[w])};
    }
    mmio.windows[ShengGmuCore] = {reinterpret_cast<volatile unsigned char *>(gVa[GPB_WIN_GMU]), GPB_GMU_BASE,
                                  static_cast<size_t>(GPB_GMU_SIZE)};
    mmio.windows[ShengGmuRscc] = {reinterpret_cast<volatile unsigned char *>(gVa[GPB_WIN_RSCC]), GPB_RSCC_BASE,
                                  static_cast<size_t>(GPB_RSCC_SIZE)};
    const ShengGmuStatus bind = ShengGmuBindMmio(&mmio, &device.io);
    if (bind != ShengGmuSuccess) {IOLog("ASTEROIDS GPU: gmuboot bind refused (%u)\n", bind); return;}
    // The payloads, Normal non-cacheable like the GMU's MAIR index 0 (write-combined: mode 4's mapping of the pool).
    ShengGmuMemory *memory[5];
    ShengGmuConfig config{};
    memory[0] = &config.icache; memory[1] = &config.dummy; memory[2] = &config.debug; memory[3] = &config.log;
    memory[4] = &config.hfi;
    for (unsigned i = 0; i < 5; ++i) {
        IODeviceMemory *range = IODeviceMemory::withRange(payloads[i].physical, payloads[i].bytes);
        if (range) {
            payloadMap[i] = OSSharedPtr<IOMemoryMap>(range->map(kIOMapAnywhere | kIOMapWriteCombineCache), OSNoRetain);
            range->release();
        }
        if (!payloadMap[i]) {IOLog("ASTEROIDS GPU: gmuboot payload %u map failed\n", i); return;}
        *memory[i] = {reinterpret_cast<volatile uint32_t *>(payloadMap[i]->getVirtualAddress()),
                      static_cast<size_t>(payloads[i].bytes), payloads[i].iova};
    }
    config.chip_id = SHENG_GMU_CHIP_ID;
    config.firmware = sheng_gmu_firmware;
    config.firmware_bytes = sizeof sheng_gmu_firmware;
    config.poll_budget = 100;   // ShengGmu's poll: 100 us apart and at most 10 ms in all
    device.services = {nullptr, gmuSha256, gmuNowUs, gmuWaitUs};
    ShengGmuStartupResult &s = native_gpu_gmu_start;
    ShengGmuStatus status;
    IOLog("ASTEROIDS GPU: gmuboot %s: firmware %u bytes, hfi@%llx log@%llx\n",
          gMode >= 10 ? "cold start with HFI startup" : "load to HFI ready", static_cast<unsigned>(sizeof sheng_gmu_firmware),
          static_cast<unsigned long long>(config.hfi.iova), static_cast<unsigned long long>(config.log.iova));
    native_gpu_power_stage = 0x91;
    if (gMode >= 10) {
        const ShengGmuStartupLimits limits = {1000, 100, 100000, 1000000};   // 100 ms per command, 1 s in all
        status = ShengGmuColdStart(&device, &config, &gStartup, &limits, &s);
    } else {
        s = ShengGmuStartupResult{};
        status = ShengGmuLoadAndStartHfi(&device, &config, &s.load);
    }
    native_gpu_gmu_start_status = status;
    native_gpu_power_stage = 0x92;
    const ShengGmuResult &l = s.load;
    IOLog("ASTEROIDS GPU: gmuboot %s phase=%u hash=%u reset_ok=%u fw_ready=%u hfi_ready=%u init=%08x hfi=%08x\n",
          status < 11 ? kShengStatus[status] : "?", l.phase, l.complete_hash_verified, l.reset_verified,
          l.firmware_ready_observed, l.hfi_ready_observed, l.firmware_init_result, l.hfi_status);
    IOLog("ASTEROIDS GPU: gmuboot writes=%u reads=%u polls=%u ao=%08x cm3=%08x\n", l.write_attempts, l.read_attempts,
          l.poll_reads, l.ao_fault_status, l.cm3_fault_status);
    if (gMode >= 10) {
        const AdrenoHfiStartupResult &p = s.protocol;
        IOLog("ASTEROIDS GPU: gmuboot hfi startup=%u stage=%u acked=%08x sent=%u polls=%u fw_err=%08x hdr=%08x t=%lluus "
              "health=%u\n", s.protocol_status, p.stage, p.acknowledged_stages, p.published_commands, p.poll_reads,
              p.firmware_error, p.wire_header, static_cast<unsigned long long>(p.elapsed_us), s.health_status);
    }
    // The GMU's own view afterwards: reset, FW_INIT_RESULT, HFI status, its always-on counter and its log's first words.
    uint32_t after[4] = {};
    const uint32_t regs[4] = {GPB_GMU_CM3_SYSRESET, GPB_GMU_CM3_FW_INIT_RESULT, GPB_GMU_HFI_CTRL_STATUS, GPB_GMU_AO_COUNTER_L};
    for (unsigned i = 0; i < 4; ++i) rd(nullptr, GPB_WIN_GMU, regs[i], &after[i]);
    IOLog("ASTEROIDS GPU: gmuboot after reset=%08x init=%08x hfi=%08x ao=%08x log %08x %08x %08x %08x\n", after[0],
          after[1], after[2], after[3], config.log.cpu[0], config.log.cpu[1], config.log.cpu[2], config.log.cpu[3]);
    native_gpu_power_stage = 0x93;
}
extern "C" {
struct gpb_gx native_gpu_gx;
}
// Mode 11's register access goes through the cold start's own MMIO adapter (ShengGmuBindMmio), as uke's mode 8 does.
// Modes 12-14 add the CX_MISC window (reads only) and, from mode 13's hw_init on, writes to the GPU block.
bool gGpuWrites;
unsigned gxWindow(unsigned w) {return w == GPB_WIN_GMU ? ShengGmuCore : w == GPB_WIN_CX ? ShengGmuCxMisc : ShengGmuGpu;}
int gxRead(void *, unsigned w, uint32_t off, uint32_t *out) {
    ShengGmuIo &io = gColdDevice.io;
    return io.read32(io.cookie, gxWindow(w), off, out);
}
int gxWrite(void *, unsigned w, uint32_t off, uint32_t value) {
    ShengGmuIo &io = gColdDevice.io;
    if (w == GPB_WIN_CX || (w != GPB_WIN_GMU && !gGpuWrites)) return 1;
    return io.write32(io.cookie, gxWindow(w), off, value) || io.barrier(io.cookie);
}
int hfiWaitOnce(void *) {IODelay(100); return 0;}
int gxVote(void *, uint32_t perf, uint32_t bw, uint32_t *firmware_error) {
    const uint32_t vote[3] = {1, perf, bw};   // a6xx_hfi_gx_bw_perf_vote_cmd: ack_type 1 (blocking), freq, bw
    uint32_t out[16];
    size_t count = 0;
    return AdrenoHfiExchange(&gColdDevice.hfi, 30, vote, 3, 1000, hfiWaitOnce, out, 16, &count, firmware_error);
}
void gxDelay(void *, unsigned us) {IODelay(us);}
extern "C" {
// T39: the HFI level and bandwidth index last voted (BootLog's [clocks] row) and that level's MHz (the banner's GPUG row)
uint32_t native_gpu_perf_vote[2];
volatile uint32_t native_gpu_mhz;
}
unsigned gBootPerf;   // the level gpuGx voted; the governor's default ceiling
// Mode 11: only after mode 10 returned success.
void gpuGx() {
    native_gpu_power_stage = 0xa0;
    if (native_gpu_gmu_start_status != ShengGmuSuccess) {
        IOLog("ASTEROIDS GPU: gx skipped: cold start status %u\n", native_gpu_gmu_start_status);
        return;
    }
    // T34: sheng-gpu-level picks the HFI level (1 = 264 MHz ... 9 = 1050 MHz on the 3a's bin, clamped); level 1 made
    // the UI slow. 6 (763 MHz, 6.7 GB/s) stays below the top three while nothing throttles for temperature.
    unsigned level = 6;
    PE_parse_boot_argn("sheng-gpu-level", &level, sizeof level);
    unsigned bw = 0;
    const unsigned perf = ShengGmuPerfVote(level, &bw);
    IOLog("ASTEROIDS GPU: gx vote perf %u bw %u (%lluk), then OOB GPU_SET\n", perf, bw,
          static_cast<unsigned long long>(gStartup.gpu[perf].hz / 1000));
    native_gpu_power_stage = 0xa1;
    gpb_gx_io io{nullptr, gxRead, gxWrite, gxVote, gxDelay};
    gpb_gx &x = native_gpu_gx;
    gpb_gx_on(&io, perf, bw, &x);
    if (!x.failed) {
        native_gpu_perf_vote[0] = perf; native_gpu_perf_vote[1] = bw;
        native_gpu_mhz = uint32_t(gStartup.gpu[perf].hz / 1000000);
        gBootPerf = perf;
    }
    native_gpu_power_stage = 0xa2;
    static const char *const kFail[] = {"ok", "vote", "no-ack", "io", "gx-off"};
    IOLog("ASTEROIDS GPU: gx %s step=%u vote=%u fw=%08x oob=%u info=%08x gx %08x>%08x later=%08x\n",
          x.failed < 5 ? kFail[x.failed] : "?", x.step, x.vote_status, x.fw_error, x.oob_polls, x.info_after, x.gx_before,
          x.gx_after, x.gx_later);
    uint32_t gdscr = 0;
    if (gVa[GPB_WIN_GPUCC]) gdscr = *reinterpret_cast<volatile uint32_t *>(gVa[GPB_WIN_GPUCC] + GPB_CC_GX_GDSCR);
    IOLog("ASTEROIDS GPU: gx rbbm=%08x gfx=%08x br=%08x gbif_halt=%08x gx_gdscr=%08x writes=%u\n", x.rbbm, x.gfx, x.gfx_br,
          x.gbif_halt, gdscr, x.writes);
    native_gpu_power_stage = x.failed ? 0xaf : 0xa7;
}
// Mode 12: after mode 11 left GX on. Reads only, then the GPU's own SMMU context and the SQE copy:
//   1 gpb_gpu_survey: the SW fuse (a6xx_gmu_secure_init writes it itself when SCM is missing; this build has no SCM
//     call and does not write it yet) and the CP/RBBM registers mode 13 is about to program
//   2 NativeAdrenoGpuContextBringup: bank 0 (arm-smmu-qcom gives the GPU bank 0), ASID 2, the DT's SID 0 alone on SMR
//     slot 0 (GpuNoStream for the second). The hypervisor aborts an SMR write for SID 1 (the first T12a: a synchronous
//     external abort reading SMR 1 back) as it does for the secure SID 2 on uke; pandora's A840 also runs on SID 0 alone.
//     Mode 5's GMU context keeps bank 1 and slot 2.
//   3 the five buffers mapped write-combined (Normal non-cacheable, as the GPU maps them), then the SQE image copied
//     without its first dword (adreno_fw_create_bo) and read back
extern "C" {
struct gpb_survey native_gpu_survey;
struct NativeAdrenoGmuContextReport native_gpu_context;
struct gpb_hw native_gpu_hw;
// cp init status, fence result, fence seen, mem-write status, its fence result, scratch word, pwrup bytes, pwrup status
uint32_t native_gpu_cp[8];
}
OSSharedPtr<IOMemoryMap> gGpuSmmu, gGpuMaps[GpuBufferCount];
volatile uint8_t *gGpuCpu[GpuBufferCount];
NativeAdrenoGpuBuffer gGpuBuffers[GpuBufferCount];
bool gGpuReady;
void gpuCtxProgress(uint32_t step) {
    native_gpu_power_stage = 0xb0 + step;
    IOLog("ASTEROIDS GPU: gpuctx step %u %s\n", step, step < 8 ? kGmuStep[step] : "?");
}
uint32_t smmuRead(size_t off) {return *reinterpret_cast<volatile uint32_t *>(gGpuSmmu->getVirtualAddress() + off);}
void gpuSurvey() {
    native_gpu_power_stage = 0xb0;
    const gpb_gx &x = native_gpu_gx;
    if (x.failed || x.step != 6) {IOLog("ASTEROIDS GPU: survey skipped: gx failed=%u step=%u\n", x.failed, x.step); return;}
    IOLog("ASTEROIDS GPU: survey: CX_MISC SW fuse, then CP and RBBM registers\n");
    gpb_gx_io io{nullptr, gxRead, gxWrite, gxVote, gxDelay};
    gpb_survey &v = native_gpu_survey;
    gpb_gpu_survey(&io, &v);
    IOLog("ASTEROIDS GPU: survey done=%d fuse=%08x freq=%08x rbbm=%08x int=%08x fuse_int=%08x halt=%08x/%08x tsb=%08x\n", v.done,
          v.fuse, v.freq_limit, v.gpu[0], v.gpu[1], v.gpu[2], v.gpu[3], v.gpu[4], v.gpu[5]);
    IOLog("ASTEROIDS GPU: survey sqe cntl=%08x base=%08x%08x rb=%08x rptr=%08x wptr=%08x aperture=%08x cp_int=%08x\n",
          v.gpu[6], v.gpu[8], v.gpu[7], v.gpu[9], v.gpu[10], v.gpu[11], v.gpu[12], v.gpu[13]);
    if (!v.done) return;
    native_gpu_power_stage = 0xb1;
    IODeviceMemory *memory = IODeviceMemory::withRange(GPB_SMMU_BASE, GPB_SMMU_WINDOW);
    if (memory) {
        gGpuSmmu = OSSharedPtr<IOMemoryMap>(memory->map(kIOMapAnywhere | kIOMapInhibitCache), OSNoRetain);
        memory->release();
    }
    if (!gGpuSmmu) {IOLog("ASTEROIDS GPU: gpuctx SMMU map failed\n"); return;}
    IOLog("ASTEROIDS GPU: gpuctx bank 0 asid 2, sid 0 on slot 0 only\n");
    const uint32_t id1 = native_gpu_power_result.id1;
    const size_t cb0 = gpb_smmu_cb_offset(id1, 0);
    // ACTLR before the kit programs and enables the bank, as arm-smmu-qcom and pandora's level 9 set it.
    const uint32_t actlr_before = smmuRead(cb0 + GPB_SMMU_CB_ACTLR);
    *reinterpret_cast<volatile uint32_t *>(gGpuSmmu->getVirtualAddress() + cb0 + GPB_SMMU_CB_ACTLR) = GPB_GPU_ACTLR;
    gColdDevice.io.barrier(gColdDevice.io.cookie);
    IOLog("ASTEROIDS GPU: gpuctx actlr %08x -> %08x\n", actlr_before, smmuRead(cb0 + GPB_SMMU_CB_ACTLR));
    NativeAdrenoGmuContextReport &q = native_gpu_context;
    const IOReturn status = NativeAdrenoGpuContextBringup(reinterpret_cast<volatile uint8_t *>(gGpuSmmu->getVirtualAddress()),
                                                          GPB_SMMU_WINDOW, 0, 2, 0, 0, 1, GpuNoStream, gpuCtxProgress, &q);
    IOLog("ASTEROIDS GPU: gpuctx s%u %s st=%x k%u/%u oas=%u p%uw%ur%ue%u r%uw%uv%u spans=%u mismatch=%03x\n", q.step,
          q.step < 8 ? kGmuStep[q.step] : "?", static_cast<unsigned>(status), q.program_status, q.route_status,
          q.output_bits, q.program_phase, q.program_writes, q.program_readback_valid, q.program_enabled, q.route_phase,
          q.route_writes, q.routes_verified, q.spans, q.mismatch);
    IOLog("ASTEROIDS GPU: gpuctx ttbr0=%016llx sctlr=%08x cbar=%08x smr=%08x s2cr=%08x\n",
          static_cast<unsigned long long>(q.ttbr0), q.sctlr, q.cbar, q.smr, q.s2cr);
    IOLog("ASTEROIDS GPU: gpuctx after gfsr=%08x fsr=%08x actlr=%08x smr %08x %08x s2cr %08x %08x\n", smmuRead(GPB_SMMU_GFSR),
          smmuRead(cb0 + GPB_SMMU_CB_FSR), smmuRead(cb0 + 0x4), smmuRead(GPB_SMMU_SMR(0)), smmuRead(GPB_SMMU_SMR(1)),
          smmuRead(GPB_SMMU_S2CR(0)), smmuRead(GPB_SMMU_S2CR(1)));
    if (status) return;
    native_gpu_power_stage = 0xb8;
    if (NativeAdrenoGpuContextBuffers(gGpuBuffers)) {IOLog("ASTEROIDS GPU: gpuctx buffers refused\n"); return;}
    for (unsigned i = 0; i < GpuBufferCount; ++i) {
        IODeviceMemory *m = IODeviceMemory::withRange(gGpuBuffers[i].physical, gGpuBuffers[i].bytes);
        if (m) {
            gGpuMaps[i] = OSSharedPtr<IOMemoryMap>(m->map(kIOMapAnywhere | kIOMapWriteCombineCache), OSNoRetain);
            m->release();
        }
        if (!gGpuMaps[i]) {IOLog("ASTEROIDS GPU: gpuctx buffer %u map failed\n", i); return;}
        gGpuCpu[i] = reinterpret_cast<volatile uint8_t *>(gGpuMaps[i]->getVirtualAddress());
    }
    IOLog("ASTEROIDS GPU: gpuctx sqe %llx+%llx@%llx ring %llx@%llx memptrs %llx@%llx\n", gGpuBuffers[0].physical,
          gGpuBuffers[0].bytes, gGpuBuffers[0].iova, gGpuBuffers[1].physical, gGpuBuffers[1].iova, gGpuBuffers[2].physical,
          gGpuBuffers[2].iova);
    native_gpu_power_stage = 0xb9;
    const size_t sqe = sizeof sheng_sqe_firmware;
    if (sqe - 4 > gGpuBuffers[0].bytes) {IOLog("ASTEROIDS GPU: sqe %u bytes do not fit\n", static_cast<unsigned>(sqe)); return;}
    for (size_t i = 4; i < sqe; ++i) gGpuCpu[0][i - 4] = sheng_sqe_firmware[i];
    gColdDevice.io.barrier(gColdDevice.io.cookie);
    size_t bad = 0;
    for (size_t i = 4; i < sqe; ++i) bad += gGpuCpu[0][i - 4] != sheng_sqe_firmware[i];
    const volatile uint32_t *w = reinterpret_cast<const volatile uint32_t *>(gGpuCpu[0]);
    IOLog("ASTEROIDS GPU: sqe copied %u bytes, %u differ, words %08x %08x %08x\n", static_cast<unsigned>(sqe - 4),
          static_cast<unsigned>(bad), w[0], w[1], w[2]);
    gGpuReady = !bad;
    native_gpu_power_stage = gGpuReady ? 0xbb : 0xbf;
}
// Mode 13: gpb_hw_init over the same adapter, now allowed to write the GPU block. Nothing starts the CP.
void gpuHwInit() {
    native_gpu_power_stage = 0xc0;
    if (!gGpuReady) {IOLog("ASTEROIDS GPU: hw_init skipped: mode 12 did not finish\n"); return;}
    IOLog("ASTEROIDS GPU: hw_init: a8xx register writes up to the SQE base\n");
    gGpuWrites = true;
    gpb_gx_io io{nullptr, gxRead, gxWrite, gxVote, gxDelay};
    gpb_hw &h = native_gpu_hw;
    gpb_hw_init(&io, &h);
    IOLog("ASTEROIDS GPU: hw_init %s step=%u writes=%u aperture=%u protect=%u nonctxt=%u mismatches=%u halt=%08x/%08x\n",
          h.failed ? "io" : "ok", h.step, h.writes, h.aperture_writes, h.protect_writes, h.nonctxt_writes, h.mismatches,
          h.gbif_halt, h.rbbm_gbif_halt);
    IOLog("ASTEROIDS GPU: hw_init check %08x %08x %08x %08x %08x %08x %08x %08x\n", h.check[0], h.check[1], h.check[2],
          h.check[3], h.check[4], h.check[5], h.check[6], h.check[7]);
    IOLog("ASTEROIDS GPU: hw_init rbbm=%08x int=%08x cp_int=%08x xoclk0=%08x\n", h.rbbm, h.int_status, h.cp_global, h.xoclk0);
    native_gpu_power_stage = h.failed ? 0xcf : 0xc9;
}
// Mode 14: the rest of a8xx hw_init through cp_init, then one command. The secure-mode exit (zap shader or
// SECVID_TRUST_CNTL), GMEM_PROTECT after it, hwcg and the OOB clear are later stages.
//   1 CP_SQE_INSTR_BASE; the ring (NativeAdrenoRing on the gen8 table: RB_BASE, RB_CNTL 32 KiB / block 32, RPTR shadow
//     at memptrs+0, WPTR 0, the fence scratch); CP_RB_RPTR_ADDR_BV at memptrs+8; one ring, so no preemption setup
//   2 CP_SQE_CNTL 1, then a8xx_cp_init (CP_THREAD_CONTROL, CP_ME_INIT) with the power-up list still zero, as Linux
//     sends it (a8xx_patch_pwrup_reglist runs after cp_init); the fence within 200 ms
//   3 CP_MEM_WRITE (opcode 0x3d) of 0x5e9a810 into the scratch page, and its fence
//   4 a8xx_patch_pwrup_reglist (NativeAdrenoPwrupList::buildA810) into the power-up page
// pandora's level 10 ran CP_MEM_WRITE while still secure, so step 3 does not wait for the zap shader.
// CP_RB_WPTR goes out as Linux's fenced write: GMU_AHB_FENCE_STATUS bit 0 means the GMU's AHB fence dropped it, so
// write it again, for up to about 2 ms.
uint32_t gWptrWrites, gWptrRetries, gWptrDropped, gWptrFence;
int ringWrite(void *, uint32_t reg, uint32_t value) {
    ShengGmuIo &io = gColdDevice.io;
    if (reg != A810_CP_RB_WPTR) return io.write32(io.cookie, ShengGmuGpu, size_t(reg) * 4, value);
    ++gWptrWrites;
    for (unsigned tries = 0; tries < 200; ++tries) {
        if (io.write32(io.cookie, ShengGmuGpu, size_t(reg) * 4, value) || io.barrier(io.cookie) ||
            io.read32(io.cookie, ShengGmuCore, GPB_GMU_AHB_FENCE_STATUS, &gWptrFence)) return 1;
        if (!(gWptrFence & 1)) return 0;
        ++gWptrRetries;
        IODelay(10);
    }
    ++gWptrDropped;
    return 1;
}
int ringRead(void *, uint32_t reg, uint32_t *value) {
    return gColdDevice.io.read32(gColdDevice.io.cookie, ShengGmuGpu, size_t(reg) * 4, value);
}
int ringBarrier(void *) {return gColdDevice.io.barrier(gColdDevice.io.cookie);}
// Ring, memptrs and lists are Normal non-cacheable on both sides (GPU MAIR 0, CPU write-combine): ordering only.
int ringPublish(void *, const volatile uint32_t *, size_t) {return ringBarrier(nullptr);}
int ringAcquire(void *, volatile uint32_t *, size_t) {return ringBarrier(nullptr);}
uint64_t ringNow(void *) {uint64_t us = 0; gmuNowUs(nullptr, &us); return us;}
// T39: the fence waits' pause, after UniXNU-sheng's (GpuPowerProbe.cpp ringIdle/gpuIrq/irqAndTimer, BSD-2-Clause-Patent). Until T38
// every wait spun 20 us at a time, so a core stayed at 100 % whenever a client waited on the GPU (the T38 banner's GOV row). Once
// irqAttach() has the GPU line, a waiter that may block sleeps until the next GPU interrupt or `sheng-gpu-pause-us` (default 1000),
// whichever is first; before that, or where it may not block, it spins 20 us as before. Every ring fence is CP_EVENT_WRITE7
// CACHE_CLEAN with its IRQ bit (NativeAdrenoRing.cpp), which raises RBBM_INT_0 CP_CACHE_FLUSH_TS (bit 20), unmasked by hw_init's
// A810_INIT_RBBM_INT_0_MASK 0x33d283c2. `sheng-gpu-irq=0` keeps the line detached and the old spin.
extern "C" {
// attach IOReturn (0xdead without ArmCpu, 0xff when off), GICD readback, interrupts after the attach / after a NOP fence, the
// INT_0_STATUS bits seen, INT_0_STATUS then
uint32_t native_gpu_irq[6];
volatile uint32_t native_gpu_irq_count;
volatile uint64_t native_gpu_idle_sleeps, native_gpu_idle_spins, native_gpu_idle_timeouts;
int armcpu_spi_attach(uint32_t, bool, void (*)(void *), void *, uint32_t *, uint64_t *) __attribute__((weak_import));
int get_preemption_level(void);
}
constexpr uint32_t kGpuIntid = 32 + 300;   // live DT kgsl-3d0 "kgsl_3d0_irq" <0 0x12c 4>: SPI 300, level high
volatile bool gIrqAttached;
volatile uint32_t gIrqSeen, gIrqEmpty;
unsigned gPauseUs = 1000;
int ringIdle(void *) {
    if (gIrqAttached && ml_get_interrupts_enabled() && !ml_at_interrupt_context() && get_preemption_level() == 0) {
        assert_wait_timeout(event_t(&native_gpu_irq_count), THREAD_UNINT, gPauseUs, NSEC_PER_USEC);
        if (thread_block(THREAD_CONTINUE_NULL) == THREAD_TIMED_OUT) native_gpu_idle_timeouts = native_gpu_idle_timeouts + 1;
        native_gpu_idle_sleeps = native_gpu_idle_sleeps + 1;
    } else {
        IODelay(20);
        native_gpu_idle_spins = native_gpu_idle_spins + 1;
    }
    return 0;
}
// a8xx_irq: read RBBM_INT_0_STATUS and clear it through RBBM_INT_CLEAR_CMD before the EOI, so the level line drops. A storm (the
// line high with nothing to clear) masks the GPU interrupts instead of holding the boot CPU; the waiters then time out each pause.
void gpuTimeFences();
void gpuIrq(void *) {
    uint32_t status = 0;
    ringRead(nullptr, A810_RBBM_INT_0_STATUS, &status);
    if (status) {ringWrite(nullptr, A810_RBBM_INT_CLEAR_CMD, status); ringBarrier(nullptr);}
    gIrqSeen = gIrqSeen | status;
    native_gpu_irq_count = native_gpu_irq_count + 1;
    gpuTimeFences();
    gIrqEmpty = status ? 0 : gIrqEmpty + 1;
    if (gIrqEmpty > 100000) {ringWrite(nullptr, A810_RBBM_INT_0_MASK, 0); ringBarrier(nullptr);}
    thread_wakeup(event_t(&native_gpu_irq_count));
}
unsigned bootArg(const char *name, unsigned fallback) {
    unsigned value = fallback;
    PE_parse_boot_argn(name, &value, sizeof value);
    return value;
}
NativeAdrenoRing gRing;
// T48: the ring's newest submitted fence and the completed one from its fence page (+4, mapped uncached), for the display's zero-copy
// probe (GopFramebuffer NativeLegacyDisplay.cpp). T42's flip wait asked NativeAdreno's service for the fence page, which this board
// never starts, so it always found the GPU done.
extern "C" bool native_gpu_fences(uint32_t *submitted, uint32_t *completed) {
    if (!gGpuCpu[2] || !submitted || !completed) return false;
    *submitted = gRing.submissions();
    *completed = reinterpret_cast<const volatile uint32_t *>(gGpuCpu[2])[1];
    return true;
}
// T51: the GPU's time per submission. The ring calls gpuSubmitted with each fence as it commits it (NativeAdreno native_adreno_ring_hook);
// the fence interrupt finds the fences completed since the last one. The ring runs one submission after another, so a submission's GPU
// time is its completion less the later of its submission and the previous completion, and the GPU waited for work whenever a
// submission came after the previous one finished. [0..8] GPU time under 1, 2, 4, 8, 12, 17, 25, 50 ms, then longer; [9] submissions
// timed, [10] total GPU time (us), [11] longest (us), [12] time the GPU had nothing queued (ms), [13] time submissions queued behind
// earlier ones (ms), [14] fences completed together in one interrupt (more than one), [15] fences without a recorded submission time.
extern "C" {volatile uint32_t native_gpu_time[16];}
extern "C" void (*volatile native_adreno_ring_hook)(uint32_t fence);   // NativeAdrenoRing.cpp
uint64_t gSubmitAt[256];
uint32_t gTimedDone;
uint64_t gLastDoneAt;
uint64_t gIdleNs, gQueuedNs;
void gpuSubmitted(uint32_t fence) {gSubmitAt[fence & 255] = mach_absolute_time() | 1;}
void gpuTimeFences() {
    if (!gGpuCpu[2]) return;
    const uint32_t done = reinterpret_cast<const volatile uint32_t *>(gGpuCpu[2])[1];
    if (int32_t(done - gTimedDone) <= 0) return;
    const uint64_t now = mach_absolute_time();
    if (done - gTimedDone > 1) native_gpu_time[14] = native_gpu_time[14] + 1;
    if (done - gTimedDone > 64) gTimedDone = done - 64;   // a gap too long to attribute
    for (uint32_t f = gTimedDone + 1; int32_t(done - f) >= 0; ++f) {
        const uint64_t at = gSubmitAt[f & 255];
        gSubmitAt[f & 255] = 0;
        if (!at) {native_gpu_time[15] = native_gpu_time[15] + 1; continue;}
        uint64_t start = at;
        if (gLastDoneAt > at) {uint64_t q = 0; absolutetime_to_nanoseconds(gLastDoneAt - at, &q); gQueuedNs += q; start = gLastDoneAt;}
        else if (gLastDoneAt) {uint64_t idle = 0; absolutetime_to_nanoseconds(at - gLastDoneAt, &idle); gIdleNs += idle;}
        uint64_t ns = 0;
        if (now > start) absolutetime_to_nanoseconds(now - start, &ns);
        const uint32_t us = uint32_t(ns / 1000);
        const unsigned b = us < 1000 ? 0 : us < 2000 ? 1 : us < 4000 ? 2 : us < 8000 ? 3 : us < 12000 ? 4 : us < 17000 ? 5 : us < 25000 ? 6 : us < 50000 ? 7 : 8;
        native_gpu_time[b] = native_gpu_time[b] + 1;
        native_gpu_time[9] = native_gpu_time[9] + 1;
        native_gpu_time[10] = native_gpu_time[10] + us;
        if (us > native_gpu_time[11]) native_gpu_time[11] = us;
        gLastDoneAt = now;
    }
    gTimedDone = done;
    native_gpu_time[12] = uint32_t(gIdleNs / 1000000);
    native_gpu_time[13] = uint32_t(gQueuedNs / 1000000);
}
NativeAdrenoCpInit gCpInit;
void cpState(const char *when) {
    uint32_t v[7] = {};
    const uint32_t regs[7] = {A810_CP_RB_RPTR, A810_CP_RB_WPTR, A810_RBBM_STATUS, A810_RBBM_INT_0_STATUS,
                              A810_CP_INTERRUPT_STATUS_GLOBAL, A810_CP_SQE_CNTL, A810_CP_APERTURE_CNTL_HOST};
    for (unsigned i = 0; i < 7; ++i) ringRead(nullptr, regs[i], &v[i]);
    uint32_t fault = 0, status = 0;   // BR's HW fault and interrupt status, through the aperture, which is then cleared
    ringWrite(nullptr, A810_CP_APERTURE_CNTL_HOST, A810_PIPE_BR << A810_CP_APERTURE_CNTL_HOST_PIPEID_SHIFT);
    ringRead(nullptr, A810_CP_HW_FAULT_STATUS_PIPE, &fault);
    ringRead(nullptr, A810_CP_INTERRUPT_STATUS_PIPE, &status);
    ringWrite(nullptr, A810_CP_APERTURE_CNTL_HOST, 0);
    const volatile uint32_t *m = reinterpret_cast<const volatile uint32_t *>(gGpuCpu[2]);
    const size_t cb0 = gpb_smmu_cb_offset(native_gpu_power_result.id1, 0);
    IOLog("ASTEROIDS GPU: cp %s rptr=%x wptr=%x rbbm=%08x int=%08x cp_int=%08x sqe=%x ap=%x br_fault=%08x br_int=%08x\n",
          when, v[0], v[1], v[2], v[3], v[4], v[5], v[6], fault, status);
    IOLog("ASTEROIDS GPU: cp %s memptrs %08x %08x %08x %08x smmu fsr=%08x far=%08x%08x fsynr0=%08x\n", when, m[0], m[1], m[2],
          m[3], smmuRead(cb0 + GPB_SMMU_CB_FSR), smmuRead(cb0 + 0x64), smmuRead(cb0 + 0x60), smmuRead(cb0 + 0x68));
    IOLog("ASTEROIDS GPU: cp %s wptr writes=%u retries=%u dropped=%u fence=%08x actlr=%08x\n", when, gWptrWrites, gWptrRetries,
          gWptrDropped, gWptrFence, smmuRead(cb0 + GPB_SMMU_CB_ACTLR));
}
void cpStart() {
    native_gpu_power_stage = 0xd0;
    const gpb_hw &h = native_gpu_hw;
    if (h.failed || h.step != 9) {IOLog("ASTEROIDS GPU: cp skipped: hw_init failed=%u step=%u\n", h.failed, h.step); return;}
    uint32_t *c = native_gpu_cp;
    const NativeAdrenoRegisters *gen8 = NativeAdrenoRegisterTable::gen8();
    if (gRing.setRegisters(gen8) || gCpInit.setRegisters(gen8)) {IOLog("ASTEROIDS GPU: cp gen8 table refused\n"); return;}
    IOLog("ASTEROIDS GPU: cp: SQE base %llx, ring %llx, then SQE_CNTL and CP_ME_INIT\n", gGpuBuffers[0].iova,
          gGpuBuffers[1].iova);
    native_gpu_power_stage = 0xd1;
    const uint64_t sqe = gGpuBuffers[0].iova, bv_rptr = gGpuBuffers[2].iova + 8;   // rbmemptr(bv_rptr)
    if (ringWrite(nullptr, A810_CP_SQE_INSTR_BASE, uint32_t(sqe)) || ringWrite(nullptr, A810_CP_SQE_INSTR_BASE + 1, uint32_t(sqe >> 32))) {
        IOLog("ASTEROIDS GPU: cp SQE base write failed\n"); return;
    }
    // No GMU AHB fence status yet (fence_cookie, fence_status): WPTR writes are not checked, as before Adreno740 5c8c145.
    const NativeAdrenoRingIo rio{nullptr, ringWrite, ringRead, ringBarrier, nullptr, nullptr};
    const NativeAdrenoRingCache cache{nullptr, ringPublish, ringAcquire};
    const IOReturn attached = gRing.attach(&rio, &cache, const_cast<uint8_t *>(gGpuCpu[1]), gGpuBuffers[1].bytes,
        gGpuBuffers[1].iova, const_cast<uint8_t *>(gGpuCpu[2]), gGpuBuffers[2].bytes, gGpuBuffers[2].iova);
    if (attached) {IOLog("ASTEROIDS GPU: cp ring attach %x\n", static_cast<unsigned>(attached)); return;}
    native_adreno_ring_hook = gpuSubmitted;
    if (ringWrite(nullptr, A810_CP_RB_RPTR_ADDR_BV, uint32_t(bv_rptr)) ||
        ringWrite(nullptr, A810_CP_RB_RPTR_ADDR_BV + 1, uint32_t(bv_rptr >> 32))) {IOLog("ASTEROIDS GPU: cp BV rptr write failed\n"); return;}
    cpState("before");
    native_gpu_power_stage = 0xd2;
    if (ringWrite(nullptr, A810_CP_SQE_CNTL, A810_INIT_CP_SQE_CNTL)) {IOLog("ASTEROIDS GPU: cp SQE_CNTL write failed\n"); return;}
    const NativeAdrenoCpInitConfig config{gGpuBuffers[0].iova, gGpuBuffers[3].iova, 1};
    uint32_t fence = 0;
    c[0] = uint32_t(gCpInit.run(&rio, &gRing, &config, &fence));
    const NativeAdrenoRingClock clock{nullptr, ringNow, ringIdle};
    if (!c[0]) c[1] = gRing.wait(fence, &clock, 200000, &c[2]);
    static const char *const kFence[] = {"done", "timeout", "poisoned", "argument"};
    IOLog("ASTEROIDS GPU: cp init st=%x fence %u %s seen=%u\n", c[0], fence, c[0] ? "-" : c[1] < 4 ? kFence[c[1]] : "?", c[2]);
    cpState("init");
    native_gpu_power_stage = 0xd3;
    if (c[0] || c[1]) {native_gpu_power_stage = 0xde; return;}
    volatile uint32_t *scratch = reinterpret_cast<volatile uint32_t *>(gGpuCpu[4]);
    scratch[0] = 0;
    ringBarrier(nullptr);
    uint32_t words[4] = {};
    if (AdrenoPM4Type7(0x3d, 3, &words[0]) != AdrenoPM4Success) {IOLog("ASTEROIDS GPU: cp pm4 refused\n"); return;}
    words[1] = uint32_t(gGpuBuffers[4].iova); words[2] = uint32_t(gGpuBuffers[4].iova >> 32); words[3] = 0x5e9a810;
    c[3] = uint32_t(gRing.submit(words, 4, &fence));
    if (!c[3]) c[4] = gRing.wait(fence, &clock, 200000, &c[2]);
    ringBarrier(nullptr);
    c[5] = scratch[0];
    IOLog("ASTEROIDS GPU: cp write st=%x fence %u %s seen=%u scratch=%08x %s\n", c[3], fence, c[3] ? "-" : c[4] < 4 ? kFence[c[4]] : "?",
          c[2], c[5], !c[3] && !c[4] && c[5] == 0x5e9a810 ? "ok" : "miss");
    cpState("write");
    native_gpu_power_stage = 0xd4;
    size_t written = 0;
    c[7] = uint32_t(NativeAdrenoPwrupList::buildA810(ringRead, ringWrite, nullptr, const_cast<uint8_t *>(gGpuCpu[3]),
                                                     gGpuBuffers[3].bytes, &written));
    c[6] = uint32_t(written);
    ringBarrier(nullptr);
    IOLog("ASTEROIDS GPU: cp pwrup st=%x bytes=%u\n", c[7], c[6]);
    native_gpu_power_stage = !c[3] && !c[4] && c[5] == 0x5e9a810 ? 0xd7 : 0xdf;
}
// Mode 15 (T13): leave secure mode with the zap shader, as a8xx hw_init does after cp_init and pandora's level 11 did on
// the A840. One boot; each step runs only when the one before it passed, and is logged before it runs:
//   1 SCM: is_call_avail for PAS_IS_SUPPORTED, then PAS_IS_SUPPORTED(13)
//   2 gen80300_zap.mbn split (gpb_mdt_parse) against the DT's 8 KiB gpu_microcode_region; the metadata (headers, then
//     the hash segment) in one physically contiguous page below 4 GiB, cleaned to memory. pandora needed no shared-
//     memory bridge of its own for it
//   3 PAS_INIT_IMAGE(13, metadata), PAS_MEM_SETUP(13, region, span), the load segment written into the region through a
//     write-combined map, PAS_AUTH_AND_RESET(13)
//   4 CP_SET_SECURE_MODE 0 in the ring, then a second CP_MEM_WRITE (0x05ec0ff1) into the scratch page
// RBBM_SECVID_TRUST_CNTL is read before step 1 and after step 4.
extern "C" void sheng_smc_call(uint64_t r[8]);   // BootWatchdogPolicy.c
extern "C" {
// step reached, then status and result of is_call_avail, is_supported, init_image, mem_setup, auth_and_reset, the mbn
// rule (0 = usable), write status, its fence result, scratch word, trust before, trust after
uint64_t native_gpu_zap[18];
}
struct ScmReply {uint64_t status, value; unsigned again;};
ScmReply scmCall(const char *what, uint32_t svc, uint32_t cmd, uint32_t arginfo, uint64_t a0, uint64_t a1, uint64_t a2) {
    IOLog("ASTEROIDS GPU: zap scm %s (%08x %02x %llx %llx %llx)\n", what, gpb_scm_fn(svc, cmd), arginfo, a0, a1, a2);
    uint64_t r[8] = {gpb_scm_fn(svc, cmd), arginfo, a0, a1, a2, 0, 0, 0};
    ScmReply out{};
    for (;;) {
        uint64_t x[8] = {r[0], r[1], r[2], r[3], r[4], r[5], r[6], r[7]};
        sheng_smc_call(x);
        if (x[0] != GPB_SCM_INTERRUPTED || out.again >= 1000) {out.status = x[0]; out.value = x[1]; break;}
        ++out.again;
        r[0] = GPB_SCM_INTERRUPTED; r[6] = x[6];
    }
    IOLog("ASTEROIDS GPU: zap scm %s -> status=%llx value=%llx again=%u\n", what, out.status, out.value, out.again);
    return out;
}
void zapStart() {
    native_gpu_power_stage = 0xe0;
    uint64_t *z = native_gpu_zap;
    if (native_gpu_cp[3] || native_gpu_cp[4] || native_gpu_cp[5] != 0x5e9a810) {IOLog("ASTEROIDS GPU: zap skipped: mode 14 did not finish\n"); return;}
    uint32_t trust = 0;
    ringRead(nullptr, A810_RBBM_SECVID_TRUST_CNTL, &trust);
    z[16] = trust;
    IOLog("ASTEROIDS GPU: zap: PAS %u into %llx+%x, then CP_SET_SECURE_MODE 0; trust_cntl=%08x\n", GPB_ZAP_PAS, GPB_ZAP_REGION,
          GPB_ZAP_REGION_BYTES, trust);
    z[0] = 1;
    native_gpu_power_stage = 0xe1;
    ScmReply q = scmCall("is_call_avail", GPB_SCM_SVC_INFO, GPB_SCM_INFO_IS_CALL_AVAIL, GPB_SCM_ARGS_1,
                         gpb_scm_query(GPB_SCM_SVC_PIL, GPB_SCM_PAS_IS_SUPPORTED), 0, 0);
    z[1] = q.status; z[2] = q.value;
    if (q.status || !q.value) {native_gpu_power_stage = 0xe9; return;}
    q = scmCall("is_supported", GPB_SCM_SVC_PIL, GPB_SCM_PAS_IS_SUPPORTED, GPB_SCM_ARGS_1, GPB_ZAP_PAS, 0, 0);
    z[3] = q.status; z[4] = q.value;
    if (q.status || !q.value) {native_gpu_power_stage = 0xea; return;}
    z[0] = 2;
    native_gpu_power_stage = 0xe2;
    struct gpb_mdt m;
    z[11] = gpb_mdt_parse(sheng_zap_firmware, sizeof sheng_zap_firmware, GPB_ZAP_REGION_BYTES, &m);
    IOLog("ASTEROIDS GPU: zap mbn rule=%llu header=%x hash=%x+%x meta=%x loads=%u reloc=%u span=%x-%x\n", z[11], m.header_bytes,
          m.hash_offset, m.hash_bytes, m.meta_bytes, m.loads, m.relocatable, m.min_addr, m.max_addr);
    if (z[11] || m.meta_bytes > 0x1000) {native_gpu_power_stage = 0xeb; return;}
    // The metadata page outlives this function: the monitor may still read it at auth_and_reset, and it is never freed.
    IOBufferMemoryDescriptor *meta = IOBufferMemoryDescriptor::inTaskWithPhysicalMask(kernel_task,
        static_cast<IOOptionBits>(kIODirectionInOut) | static_cast<IOOptionBits>(kIOMemoryMapperNone) | kIOMemoryPhysicallyContiguous,
        0x1000, 0xfffff000ull);
    if (!meta || meta->prepare() != kIOReturnSuccess) {IOLog("ASTEROIDS GPU: zap metadata page refused\n"); native_gpu_power_stage = 0xec; return;}
    uint8_t *meta_va = static_cast<uint8_t *>(meta->getBytesNoCopy());
    IOByteCount length = 0;
    const uint64_t meta_pa = meta->getPhysicalSegment(0, &length, kIOMemoryMapperNone);
    if (!meta_va || !meta_pa || length < 0x1000) {IOLog("ASTEROIDS GPU: zap metadata page has no address\n"); native_gpu_power_stage = 0xec; return;}
    memset(meta_va, 0, 0x1000);
    gpb_mdt_metadata(sheng_zap_firmware, &m, meta_va);
    if (NativeAdrenoMaintainOwnedPages(meta_va, 0x1000, NativeAdrenoClean)) {IOLog("ASTEROIDS GPU: zap metadata clean failed\n"); native_gpu_power_stage = 0xec; return;}
    ringBarrier(nullptr);
    z[0] = 3;
    native_gpu_power_stage = 0xe3;
    q = scmCall("init_image", GPB_SCM_SVC_PIL, GPB_SCM_PAS_INIT_IMAGE, GPB_SCM_ARGS_INIT_IMAGE, GPB_ZAP_PAS, meta_pa, 0);
    z[5] = q.status; z[6] = q.value;
    if (q.status || q.value) {native_gpu_power_stage = 0xed; return;}
    native_gpu_power_stage = 0xe4;
    q = scmCall("mem_setup", GPB_SCM_SVC_PIL, GPB_SCM_PAS_MEM_SETUP, GPB_SCM_ARGS_3, GPB_ZAP_PAS, GPB_ZAP_REGION,
                m.max_addr - m.min_addr);
    z[7] = q.status; z[8] = q.value;
    if (q.status || q.value) {native_gpu_power_stage = 0xed; return;}
    native_gpu_power_stage = 0xe5;
    IOLog("ASTEROIDS GPU: zap writing %x bytes into the region\n", m.max_addr - m.min_addr);
    {
        IODeviceMemory *region = IODeviceMemory::withRange(GPB_ZAP_REGION, GPB_ZAP_REGION_BYTES);
        OSSharedPtr<IOMemoryMap> map;
        if (region) {
            map = OSSharedPtr<IOMemoryMap>(region->map(kIOMapAnywhere | kIOMapWriteCombineCache), OSNoRetain);
            region->release();
        }
        if (!map) {IOLog("ASTEROIDS GPU: zap region map failed\n"); native_gpu_power_stage = 0xec; return;}
        gpb_mdt_place(sheng_zap_firmware, &m, reinterpret_cast<volatile uint8_t *>(map->getVirtualAddress()));
        ringBarrier(nullptr);
    }
    native_gpu_power_stage = 0xe6;
    q = scmCall("auth_and_reset", GPB_SCM_SVC_PIL, GPB_SCM_PAS_AUTH_AND_RESET, GPB_SCM_ARGS_1, GPB_ZAP_PAS, 0, 0);
    z[9] = q.status; z[10] = q.value;
    if (q.status || q.value) {native_gpu_power_stage = 0xed; return;}
    z[0] = 4;
    native_gpu_power_stage = 0xe7;
    volatile uint32_t *scratch = reinterpret_cast<volatile uint32_t *>(gGpuCpu[4]);
    scratch[1] = 0;
    ringBarrier(nullptr);
    uint32_t words[6] = {}, fence = 0, seen = 0;
    if (AdrenoPM4Type7(GPB_CP_SET_SECURE_MODE, 1, &words[0]) != AdrenoPM4Success ||
        AdrenoPM4Type7(0x3d, 3, &words[2]) != AdrenoPM4Success) {IOLog("ASTEROIDS GPU: zap pm4 refused\n"); return;}
    words[1] = 0;
    const uint64_t at = gGpuBuffers[4].iova + 4;
    words[3] = uint32_t(at); words[4] = uint32_t(at >> 32); words[5] = 0x05ec0ff1;
    const NativeAdrenoRingClock clock{nullptr, ringNow, ringIdle};
    z[12] = uint32_t(gRing.submit(words, 6, &fence));
    if (!z[12]) z[13] = gRing.wait(fence, &clock, 200000, &seen);
    ringBarrier(nullptr);
    z[14] = scratch[1];
    ringRead(nullptr, A810_RBBM_SECVID_TRUST_CNTL, &trust);
    z[17] = trust;
    static const char *const kFence[] = {"done", "timeout", "poisoned", "argument"};
    const bool ok = !z[12] && !z[13] && z[14] == 0x05ec0ff1;
    IOLog("ASTEROIDS GPU: zap secure off st=%llx fence %u %s seen=%u scratch=%08llx trust_cntl=%08x %s\n", z[12], fence,
          z[12] ? "-" : z[13] < 4 ? kFence[z[13]] : "?", seen, z[14], trust, ok ? "ok" : "miss");
    cpState("zap");
    native_gpu_power_stage = ok ? 0xe8 : 0xef;
}
// Mode 16 (T14): the first IBs and the first pixels the GPU writes. One boot; each step runs only when the one before it
// passed, and is logged before it runs:
//   1 a one-packet IB at 0x400000 (a CP_MEM_WRITE of 0x1b810014 into scratch+8), called from the ring with
//     CP_INDIRECT_BUFFER, as pandora's level 12 did
//   2 the console's framebuffer (PE_current_console) planned by gpb_bar_plan: the pages under a 48-row bar two thirds down
//     are adopted (IODeviceMemory, write-combined, zeroed, so the band first goes black) and mapped read/write at 0x800000;
//     the bar's IB (one CP_MEM_WRITE per row with its pixels: white edges, red, green and blue thirds) in scattered pages
//     at 0x500000
//   3 the bar IB from the ring, then three pixels of the middle row read back by the CPU; then again every 200 ms for 2
//     minutes, so the console's scrolling cannot hide it
// The rest of a8xx hw_init pandora's level 15 added (GMEM_PROTECT, the per-pipe masks) is already in mode 13's writes.
// Every object stays mapped for good: nothing here is torn down.
extern "C" {
// step reached, IB status, IB fence, scratch word, plan rule, adopt, fb map, IB alloc, IB map, draws ok, draws failed,
// first failure status
uint32_t native_gpu_bar[12];
}
NativeAdrenoMemory gIb1, gBarIb, gFb;
void barStart() {
    native_gpu_power_stage = 0xf0;
    uint32_t *g = native_gpu_bar;
    if (native_gpu_zap[12] || native_gpu_zap[13] || native_gpu_zap[14] != 0x05ec0ff1) {IOLog("ASTEROIDS GPU: bar skipped: mode 15 did not finish\n"); return;}
    NativeAdrenoVm *vm = NativeAdrenoGpuContextVm();
    const unsigned bits = native_gpu_context.output_bits;
    if (!vm || !bits) {IOLog("ASTEROIDS GPU: bar skipped: no GPU VM\n"); return;}
    const NativeAdrenoRingClock clock{nullptr, ringNow, ringIdle};
    static const char *const kFence[] = {"done", "timeout", "poisoned", "argument"};
    uint64_t handle = 0;
    uint32_t fence = 0, seen = 0, words[4] = {};
    g[0] = 1;
    native_gpu_power_stage = 0xf1;
    IOLog("ASTEROIDS GPU: ib: CP_INDIRECT_BUFFER at %llx with one CP_MEM_WRITE into scratch+8\n", GPB_IB_IOVA);
    IOReturn st = gIb1.allocate(0x1000, bits, true, 0x1000);
    if (st) {IOLog("ASTEROIDS GPU: ib alloc %x\n", st); return;}
    uint32_t *ib = static_cast<uint32_t *>(gIb1.mapping());
    const uint64_t at = gGpuBuffers[4].iova + 8;
    if (AdrenoPM4Type7(GPB_CP_MEM_WRITE, 3, &ib[0]) != AdrenoPM4Success ||
        AdrenoPM4Type7(GPB_CP_INDIRECT_BUFFER, 3, &words[0]) != AdrenoPM4Success) {IOLog("ASTEROIDS GPU: ib pm4 refused\n"); return;}
    ib[1] = uint32_t(at); ib[2] = uint32_t(at >> 32); ib[3] = 0x1b810014;
    st = gIb1.publishForDevice(0, 0x1000, false);
    if (!st) st = vm->map(&gIb1, 0, GPB_IB_IOVA, 1u, 0, 2, &handle);
    if (st) {IOLog("ASTEROIDS GPU: ib publish/map %x\n", st); return;}
    volatile uint32_t *scratch = reinterpret_cast<volatile uint32_t *>(gGpuCpu[4]);
    scratch[2] = 0;
    ringBarrier(nullptr);
    words[1] = uint32_t(GPB_IB_IOVA); words[2] = uint32_t(GPB_IB_IOVA >> 32); words[3] = 4;
    g[1] = uint32_t(gRing.submit(words, 4, &fence));
    if (!g[1]) g[2] = gRing.wait(fence, &clock, 200000, &seen);
    ringBarrier(nullptr);
    g[3] = scratch[2];
    const bool ib_ok = !g[1] && !g[2] && g[3] == 0x1b810014;
    IOLog("ASTEROIDS GPU: ib st=%x fence %u %s seen=%u scratch=%08x %s\n", g[1], fence, g[1] ? "-" : g[2] < 4 ? kFence[g[2]] : "?",
          seen, g[3], ib_ok ? "ok" : "miss");
    cpState("ib");
    if (!ib_ok) {native_gpu_power_stage = 0xf9; return;}
    g[0] = 2;
    native_gpu_power_stage = 0xf2;
    PE_Video video{};
    if (PE_current_console(&video)) {IOLog("ASTEROIDS GPU: bar: no console\n"); native_gpu_power_stage = 0xfa; return;}
    struct gpb_bar b;
    g[4] = uint32_t(gpb_bar_plan(video.v_baseAddr, uint32_t(video.v_width), uint32_t(video.v_height), uint32_t(video.v_rowBytes),
                                 uint32_t(video.v_depth), &b));
    IOLog("ASTEROIDS GPU: bar: console %lx %lux%lu row %lu depth %lu; rule=%u rows %u-%u span %llx+%llx at %llx, ib %u dwords\n",
          video.v_baseAddr, video.v_width, video.v_height, video.v_rowBytes, video.v_depth, g[4], b.y0, b.y0 + b.rows - 1, b.page,
          b.bytes, b.offset, b.ib_dwords);
    if (g[4]) {native_gpu_power_stage = 0xfa; return;}
    g[5] = uint32_t(gFb.adopt(b.page, size_t(b.bytes), bits, size_t(b.bytes)));
    if (!g[5]) g[6] = uint32_t(vm->map(&gFb, 0, GPB_FB_IOVA, 3u, 0, 2, &handle));
    IOLog("ASTEROIDS GPU: bar: framebuffer pages adopt %x map %x at %llx\n", g[5], g[6], GPB_FB_IOVA);
    if (g[5] || g[6]) {native_gpu_power_stage = 0xfb; return;}
    uint32_t header = 0;
    if (AdrenoPM4Type7(GPB_CP_MEM_WRITE, uint16_t(b.width + 2), &header) != AdrenoPM4Success) {IOLog("ASTEROIDS GPU: bar pm4 refused\n"); return;}
    g[7] = uint32_t(gBarIb.allocate(b.ib_bytes, bits, false, b.ib_bytes));
    if (!g[7]) {
        gpb_bar_fill(&b, header, GPB_FB_IOVA, static_cast<uint32_t *>(gBarIb.mapping()));
        g[7] = uint32_t(gBarIb.publishForDevice(0, b.ib_bytes, false));
    }
    if (!g[7]) g[8] = uint32_t(vm->map(&gBarIb, 0, GPB_BAR_IB_IOVA, 1u, 0, 2, &handle));
    IOLog("ASTEROIDS GPU: bar: ib alloc %x (%u spans) map %x at %llx\n", g[7], unsigned(gBarIb.spanCount()), g[8], GPB_BAR_IB_IOVA);
    if (g[7] || g[8]) {native_gpu_power_stage = 0xfb; return;}
    g[0] = 3;
    native_gpu_power_stage = 0xf3;
    words[1] = uint32_t(GPB_BAR_IB_IOVA); words[2] = uint32_t(GPB_BAR_IB_IOVA >> 32); words[3] = b.ib_dwords;
    const volatile uint32_t *fb = reinterpret_cast<const volatile uint32_t *>(static_cast<uint8_t *>(gFb.mapping()) + b.offset +
                                                                              size_t(b.rows / 2) * b.row_bytes);
    // Mode 18 draws once: the service must be up before the GPU addon's daemon gives up waiting for it.
    const unsigned draws = gMode >= 18 ? 1 : 600;
    IOLog("ASTEROIDS GPU: bar: drawing from the IB%s\n", draws > 1 ? ", then every 200 ms for 2 minutes" : " once");
    for (unsigned n = 0; n < draws; ++n) {
        uint32_t w = 0;
        st = gRing.submit(words, 4, &fence);
        if (!st) w = gRing.wait(fence, &clock, 200000, &seen);
        if (st || w) {++g[10]; if (!g[11]) g[11] = st ? uint32_t(st) : 0x100u | w; break;}
        ++g[9];
        if (!n) {
            ringBarrier(nullptr);
            const uint32_t px[3] = {fb[b.width / 6], fb[b.width / 2], fb[b.width * 5 / 6]};
            const bool same = px[0] == 0x00ff0000u && px[1] == 0x0000ff00u && px[2] == 0x000000ffu;
            IOLog("ASTEROIDS GPU: bar drawn fence %u seen=%u pixels %08x %08x %08x %s\n", fence, seen, px[0], px[1], px[2],
                  same ? "ok" : "miss");
            cpState("bar");
        }
        if (draws > 1) IOSleep(200);
    }
    IOLog("ASTEROIDS GPU: bar redraws ok=%u failed=%u first=%x %s\n", g[9], g[10], g[11], g[10] ? "stopped" : "done");
    native_gpu_power_stage = g[10] ? 0xff : 0xf8;
}
// Mode 17 (T15): what the user interface stands on, as uke's probe hands its live GPU to NativeAdreno (publishAndWrapperTest)
// and pandora's levels 13-14 built it. One boot; each step runs only when the one before it passed:
//   1 the BO pool: the loader-kept RAM at GPB_GPU_POOL_BASE written and checked with gpb_pool_test through a temporary
//     write-combined map, then adopted (NativeAdrenoMemory::adoptPool); user BOs come from it from now on
//   2 fences and time: NativeAdrenoSubmit on the live VM and ring (CP timestamp fences, at most eight jobs in flight) and
//     NativeAdrenoTimer on CP_ALWAYS_ON_COUNTER (gen8 table)
//   3 native_adreno_publish: the NativeAdrenoGpu service with the A810's device info
//   4 two rounds as a client takes them, minus the Mach trap: newUserClient, Query, GetDeviceInfo, two BOs (IB, target)
//     from the pool, BoMap, BoCpuMap and clientMemoryForType, an IB of three CP_MEM_WRITEs written through the client's own
//     mapping, Submit, WaitFence, GetTimestamp, clientClose (drain, unmap, free); the second round proves the slot recycles
//   5 a burst of 32 submissions on one connection, each writing its index, then one WaitFence on the last: every word must
//     be there (fences retire in order)
// Nothing is torn down at the end: the service stays published for user space.
extern "C" {
IOReturn native_adreno_publish(NativeAdrenoVm *, NativeAdrenoSubmit *, NativeAdrenoRing *, const NativeAdrenoRingClock *,
                               const NativeAdrenoTimer *, const NativeAdrenoAbiDeviceInfo *, uint32_t, uint32_t);
uint32_t native_adreno_publish_stats(uint32_t *);
IOService *native_adreno_published_service();
extern volatile uint32_t native_adreno_pool_stats[8];
// step reached, pool test bad words, adopt, submit attach, timer attach, publish, round 1 and 2 matched words (of 4),
// steps ok, steps, first error, its step, burst matched (of 32), BOs from the pool, opened, closed
uint32_t native_gpu_uc[16];
}
// NativeAdrenoPublish.cpp hands backboardd's boot-log pieces to sheng's boot-log export, which this board does not link; a
// weak-import reference cannot stay undefined in the kernel link, so the probe refuses every piece (-1, as Publish does
// when the export is absent). Weak, so a real export wins once one is linked.
extern "C" __attribute__((weak)) int boot_log_export_put(uint32_t, uint32_t, uint32_t, const void *, uint32_t) {return -1;}
NativeAdrenoSubmit gSubmit;
NativeAdrenoTimer gTimer;
const NativeAdrenoAbiDeviceInfo gDeviceInfo = {GPB_A810_CHIP_ID, GPB_A810_GMEM_BYTES, GPB_GMEM_RANGE_MIN, GPB_UCHE_TRAP_BASE,
                                               GPB_CLIENT_VA_START, GPB_CLIENT_VA_BYTES, GPB_UBWC_HBB, GPB_UBWC_SWIZZLE, 0,
                                               GPB_UBWC_VERSION, NativeAdrenoDeviceClientIova, 0};
int timerRead(void *, uint32_t reg, uint32_t *value) {return ringRead(nullptr, reg, value);}
const NativeAdrenoTimeIo gTimerIo{nullptr, timerRead};
IOReturn ucCall(IOUserClient *uc, uint32_t method, const void *in, void *out) {
    IOExternalMethodArguments a{};
    a.version = kIOExternalMethodArgumentsCurrentVersion;
    a.selector = method;
    const size_t is = NativeAdrenoUserClient::inputSize(method), os = NativeAdrenoUserClient::outputSize(method);
    a.structureInput = is ? in : nullptr;
    a.structureInputSize = uint32_t(is);
    a.structureOutput = os ? out : nullptr;
    a.structureOutputSize = uint32_t(os);
    return uc->externalMethod(method, &a, nullptr, nullptr, nullptr);
}
struct UcSteps {
    int ok = 0, n = 0, at = -1;
    IOReturn err = kIOReturnSuccess;
    bool operator()(IOReturn s) {
        const int i = n++;
        if (!s) ++ok;
        else if (!err) {err = s; at = i;}
        return !s;
    }
};
// Two pooled BOs mapped at ib/tgt, both CPU-mapped through clientMemoryForType. 0 on success; maps[] are the CPU views.
bool ucBuffers(IOUserClient *uc, UcSteps &step, uint64_t ib_iova, uint64_t tgt_iova, uint64_t handles[2], IOMemoryDescriptor *desc[2],
               IOMemoryMap *maps[2]) {
    const NativeAdrenoAbiBoAlloc alloc{0x1000, 12, 0};
    NativeAdrenoAbiBoAllocReply r[2] = {};
    for (unsigned i = 0; i < 2; ++i) if (!step(ucCall(uc, NativeAdrenoMethodBoAlloc, &alloc, &r[i]))) return false;
    handles[0] = r[0].handle; handles[1] = r[1].handle;
    const NativeAdrenoAbiBoMap map[2] = {{handles[0], ib_iova, 1, 0, 2, 0}, {handles[1], tgt_iova, 3, 0, 2, 0}};
    NativeAdrenoAbiBoMapReply mr{};
    for (unsigned i = 0; i < 2; ++i) if (!step(ucCall(uc, NativeAdrenoMethodBoMap, &map[i], &mr))) return false;
    for (unsigned i = 0; i < 2; ++i) {
        const NativeAdrenoAbiBoCpuMap cm{handles[i]};
        NativeAdrenoAbiBoCpuMapReply cr{};
        if (!step(ucCall(uc, NativeAdrenoMethodBoCpuMap, &cm, &cr))) return false;
        IOOptionBits options = 0;
        if (!step(uc->clientMemoryForType(cr.token, &options, &desc[i])) || !desc[i]) return false;
        maps[i] = desc[i]->createMappingInTask(kernel_task, 0, options);
        if (!step(maps[i] ? kIOReturnSuccess : kIOReturnNoMemory)) return false;
    }
    return true;
}
// The IB at w: one CP_MEM_WRITE per value, value i to tgt + 4 * (first + i). Returns the dwords written, 0 on a refused header.
unsigned ucWrites(volatile uint32_t *w, uint64_t tgt, unsigned first, const uint32_t *values, unsigned count) {
    unsigned k = 0;
    for (unsigned i = 0; i < count; ++i) {
        uint32_t header = 0;
        if (AdrenoPM4Type7(GPB_CP_MEM_WRITE, 3, &header) != AdrenoPM4Success) return 0;
        const uint64_t at = tgt + 4ull * (first + i);
        w[k++] = header; w[k++] = uint32_t(at); w[k++] = uint32_t(at >> 32); w[k++] = values[i];
    }
    return k;
}
IOReturn ucSubmit(IOUserClient *uc, uint64_t handle, uint64_t iova, unsigned dwords, uint64_t *fence) {
    NativeAdrenoAbiSubmit submit{};
    submit.buffer_count = 1;
    submit.buffers[0] = {handle, iova, uint64_t(dwords) * 4};
    NativeAdrenoAbiSubmitReply reply{};
    const IOReturn st = ucCall(uc, NativeAdrenoMethodSubmit, &submit, &reply);
    *fence = reply.fence;
    return st;
}
// One round as uke's wrapperRound: returns the matching words (of 4: three written, one untouched) or -1.
int ucRound(IOService *svc, UcSteps &step, unsigned round) {
    IOUserClient *uc = nullptr;
    if (!step(svc->newUserClient(kernel_task, nullptr, GPB_UC_OPEN_TYPE, nullptr, &uc)) || !uc) return -1;
    NativeAdrenoAbiParams params{};
    step(ucCall(uc, NativeAdrenoMethodQuery, nullptr, &params));
    NativeAdrenoAbiDeviceInfo info{};
    step(ucCall(uc, NativeAdrenoMethodGetDeviceInfo, nullptr, &info));
    step(info.chip_id == GPB_A810_CHIP_ID && info.gmem_bytes == GPB_A810_GMEM_BYTES ? kIOReturnSuccess : kIOReturnError);
    uint64_t handles[2] = {};
    IOMemoryDescriptor *desc[2] = {};
    IOMemoryMap *maps[2] = {};
    int matched = -1;
    const uint64_t ib = 0x20004000, tgt = 0x20005000;
    if (ucBuffers(uc, step, ib, tgt, handles, desc, maps)) {
        volatile uint32_t *iw = reinterpret_cast<volatile uint32_t *>(maps[0]->getVirtualAddress());
        volatile uint32_t *tw = reinterpret_cast<volatile uint32_t *>(maps[1]->getVirtualAddress());
        for (unsigned i = 0; i < 1024; ++i) tw[i] = 0xdeadbeef;
        const uint32_t values[3] = {0x0badc0deu, 0xfeed0000u | round, 0x600d0a81u};
        const unsigned dwords = ucWrites(iw, tgt, 0, values, 3);
        ringBarrier(nullptr);
        uint64_t fence = 0;
        if (step(dwords ? kIOReturnSuccess : kIOReturnInternalError) && step(ucSubmit(uc, handles[0], ib, dwords, &fence))) {
            const NativeAdrenoAbiWaitFence wait{fence, 200000000ull};
            if (step(ucCall(uc, NativeAdrenoMethodWaitFence, &wait, nullptr))) {
                ringBarrier(nullptr);
                matched = (tw[0] == values[0]) + (tw[1] == values[1]) + (tw[2] == values[2]) + (tw[3] == 0xdeadbeef);
            }
        }
        NativeAdrenoAbiTimestamp ts{};
        step(ucCall(uc, NativeAdrenoMethodGetTimestamp, nullptr, &ts));
        IOLog("ASTEROIDS GPU: uc round %u: abi %u in %u out %u max %llx, fence %llu, words %08x %08x %08x %08x, ticks %llu\n",
              round, params.abi_version, params.input_bits, params.output_bits, params.max_bo_bytes, fence, tw[0], tw[1], tw[2],
              tw[3], ts.ticks);
    }
    for (auto *m : maps) if (m) m->release();
    for (auto *d : desc) if (d) d->release();
    step(uc->clientClose());   // drains this client's jobs, unmaps and frees its BOs
    uc->release();
    return matched;
}
// 32 submissions on one connection, IB i writing i into word i of the target; one wait on the last fence. Matched words.
int ucBurst(IOService *svc, UcSteps &step) {
    IOUserClient *uc = nullptr;
    if (!step(svc->newUserClient(kernel_task, nullptr, GPB_UC_OPEN_TYPE, nullptr, &uc)) || !uc) return -1;
    uint64_t handles[2] = {};
    IOMemoryDescriptor *desc[2] = {};
    IOMemoryMap *maps[2] = {};
    int matched = -1;
    const uint64_t ib = 0x20010000, tgt = 0x20011000;
    if (ucBuffers(uc, step, ib, tgt, handles, desc, maps)) {
        volatile uint32_t *iw = reinterpret_cast<volatile uint32_t *>(maps[0]->getVirtualAddress());
        volatile uint32_t *tw = reinterpret_cast<volatile uint32_t *>(maps[1]->getVirtualAddress());
        for (unsigned i = 0; i < 1024; ++i) tw[i] = 0xdeadbeef;
        // Each submission's IB is its own 16-byte slice of the IB page.
        uint64_t fence = 0;
        bool sent = true;
        for (unsigned i = 0; i < 32 && sent; ++i) {
            const uint32_t value = 0xb0000000u | i;
            const unsigned dwords = ucWrites(iw + 4 * i, tgt, i, &value, 1);
            ringBarrier(nullptr);
            sent = step(dwords ? kIOReturnSuccess : kIOReturnInternalError) &&
                   step(ucSubmit(uc, handles[0], ib + 16 * i, dwords, &fence));
        }
        if (sent) {
            const NativeAdrenoAbiWaitFence wait{fence, 500000000ull};
            if (step(ucCall(uc, NativeAdrenoMethodWaitFence, &wait, nullptr))) {
                ringBarrier(nullptr);
                matched = 0;
                for (unsigned i = 0; i < 32; ++i) matched += tw[i] == (0xb0000000u | i);
            }
        }
        IOLog("ASTEROIDS GPU: uc burst: last fence %llu, matched %d of 32, word 31 %08x, word 32 %08x\n", fence, matched, tw[31],
              tw[32]);
    }
    for (auto *m : maps) if (m) m->release();
    for (auto *d : desc) if (d) d->release();
    step(uc->clientClose());
    uc->release();
    return matched;
}
// T40d: IO coherence (sheng-gpu-coherent, default 1), UniXNU-sheng's coherenceProbe (GpuPowerProbe.cpp at 08cde20,
// BSD-2-Clause-Patent) before the service is published. Until now nothing set native_adreno_io_coherent here, so the driver cleaned
// and invalidated every imported page around every submission: T40c's log counted 18 s and 361 GiB of maintenance for about 3000
// submissions (~6 ms each). A page of kernel memory is imported and mapped write-back inner-shareable (Linux: IOMMU_CACHE), and both
// directions are proved against DRAM:
//   read:  the CPU stores new values over lines DRAM still holds old (counted dirty through an uncached mapping), the GPU copies the
//          page, and the copy DRAM holds must be the new values, which only the CPU's caches had;
//   write: the CPU keeps clean lines of a page it read, the GPU copies other values over it, and plain cached loads (no
//          invalidate) must return the GPU's values.
// Only both proofs turn native_adreno_io_coherent on. The merged driver then maps only imports the GPU never writes coherent
// (myron 5bbeb3b: gen8 stopped inside its end-of-pass clean writing through write-back mappings) and skips the per-submission
// maintenance of imports. A failed proof leaves everything maintained as before.
// Row ([coherence] in the boot log): H d<dirty lines of 64> a<attempts> r<GPU read new>/<old> w<CPU saw GPU>/<DRAM> s<status> k<step>
extern "C" volatile uint32_t native_adreno_io_coherent __attribute__((weak));
// T40f test: sheng-gpu-import-skip=1 has the driver skip the per-submission clean of imports without coherence (NativeAdrenoVm.cpp).
extern "C" volatile uint32_t native_adreno_import_skip __attribute__((weak));
// T40g: the driver's import policy (NativeAdrenoVm.cpp, default 1): GPU-written imports skip per-submission maintenance, read-only ones
// keep their clean. sheng-gpu-import-policy=0 maintains every import around every submission, as before.
extern "C" volatile uint32_t native_adreno_import_policy __attribute__((weak));
extern "C" {char native_gpu_coherence_row[120]; uint32_t native_gpu_coherence[8];}
// A throwaway uncached kernel mapping (io_map with VM_WIMG_IO), not IOMemoryDescriptor::map, which would change the page's own
// cache attribute for every mapping of it (sheng int-098).
extern "C" vm_offset_t ml_io_map_unmappable(vm_offset_t phys_addr, vm_size_t size, unsigned int flags);
extern "C" void ml_io_unmap(vm_offset_t addr, vm_size_t size);
bool dramPage(uint64_t pa, uint32_t *out) {
    const vm_offset_t page = ml_io_map_unmappable(vm_offset_t(pa), 0x1000, 0x7 /* VM_WIMG_IO */);
    if (!page) return false;
    for (unsigned i = 0; i < 1024; ++i) out[i] = reinterpret_cast<const volatile uint32_t *>(page)[i];
    ml_io_unmap(page, 0x1000);
    return true;
}
bool cpFence(const uint32_t *words, size_t count) {
    uint32_t fence = 0, seen = 0;
    if (gRing.submit(words, count, &fence)) return false;
    const NativeAdrenoRingClock clock{nullptr, ringNow, ringIdle};
    return !gRing.wait(fence, &clock, 200000, &seen);
}
// CP_MEMCPY (0x75: dwords, src lo/hi, dst lo/hi) of one page, then CP_WAIT_MEM_WRITES (0x12), behind a fence.
bool gpuCopyPage(uint64_t src, uint64_t dst) {
    uint32_t words[8] = {};
    if (AdrenoPM4Type7(0x75, 5, &words[0]) != AdrenoPM4Success || AdrenoPM4Type7(0x12, 0, &words[6]) != AdrenoPM4Success) return false;
    words[1] = 1024; words[2] = uint32_t(src); words[3] = uint32_t(src >> 32); words[4] = uint32_t(dst); words[5] = uint32_t(dst >> 32);
    return cpFence(words, 7);
}
unsigned gCoherenceAttempts;
void coherenceProbe(NativeAdrenoVm *vm, unsigned bits) {
    uint32_t *k = native_gpu_coherence;
    // Off by default since T40d: the probe passed on the 3a, but with coherence on the GPU stopped at the 134th submission
    // (every later fence wait timed out, the UI never drew). sheng-gpu-coherent=1 runs the probe.
    const unsigned wanted = bootArg("sheng-gpu-coherent", 0);
    if (!wanted || !vm || !&native_adreno_io_coherent) {
        snprintf(native_gpu_coherence_row, sizeof native_gpu_coherence_row, "H skip (arg %u vm %d)", wanted, vm != nullptr);
        return;
    }
    constexpr uint64_t kIova = 0x18000000;   // clear of the IB, bar and framebuffer IOVAs (GpuPowerBoard.h); unmapped before publishing
    constexpr size_t kBytes = 4 * 4096;
    uint8_t *buf = static_cast<uint8_t *>(IOMallocAligned(kBytes, 4096));
    NativeAdrenoMemory mem;
    uint64_t handle = 0;
    IOReturn st = buf ? mem.importTask(kernel_task, uint64_t(uintptr_t(buf)), kBytes, bits, kBytes) : kIOReturnNoMemory;
    k[0] = 1;
    if (!st) {k[0] = 2; st = vm->map(&mem, 0, kIova, 3, 1, 3, &handle);}
    uint64_t phys[4] = {};
    for (size_t j = 0; !st && j < mem.spanCount(); ++j)
        for (size_t at = 0; at < mem.spans()[j].bytes; at += 4096) phys[(mem.spans()[j].offset + at) / 4096] = mem.spans()[j].physical + at;
    static uint32_t words[1024];
    bool pass = false;
    if (!st && phys[0] && phys[1] && phys[2] && phys[3]) {
        k[0] = 3;
        volatile uint32_t *S = reinterpret_cast<volatile uint32_t *>(buf), *D = S + 1024, *R = S + 2048, *T = S + 3072;
        // read direction. The proof needs S's new words still dirty in a CPU cache (DRAM still holding the old ones, k[1] >= 48 of 64
        // lines) when the GPU copies S, so the setup repeats until the lines are seen dirty, up to eight times (sheng int-150: one
        // early write-back left the whole boot maintained although both directions were right).
        bool dirty = false;
        for (unsigned attempt = 0; attempt < 8 && !st && !dirty; ++attempt) {
            gCoherenceAttempts = attempt + 1;
            for (unsigned i = 0; i < 1024; ++i) {S[i] = 0x11110000u | i; D[i] = 0; R[i] = 0x33330000u | i; T[i] = 0x44440000u | i;}
            st = NativeAdrenoMaintainOwnedPages(buf, kBytes, NativeAdrenoCleanInvalidate);
            for (unsigned i = 0; i < 1024; ++i) S[i] = 0x22220000u | i;
            __asm__ volatile("dsb sy" ::: "memory");
            k[1] = 0;
            if (!st && dramPage(phys[0], words)) for (unsigned i = 0; i < 1024; i += 16) k[1] += words[i] == (0x11110000u | i);
            dirty = k[1] >= 48;
        }
        if (!st && dirty) {
            k[0] = 4;
            if (gpuCopyPage(kIova, kIova + 4096) && dramPage(phys[1], words)) {
                k[0] = 5;
                for (unsigned i = 0; i < 1024; ++i) {k[2] += words[i] == (0x22220000u | i); k[3] += words[i] == (0x11110000u | i);}
            }
        }
        // write direction: T written back and kept (clean, resident), then read so its lines are present when the GPU writes
        if (!st && k[0] == 5) st = NativeAdrenoMaintainOwnedPages(const_cast<uint32_t *>(T), 4096, NativeAdrenoClean);
        uint32_t touch = 0;
        for (unsigned i = 0; i < 1024; ++i) touch += T[i];
        __asm__ volatile("dsb sy" ::: "memory");
        if (!st && k[0] == 5 && gpuCopyPage(kIova + 8192, kIova + 12288)) {
            k[0] = 6;
            for (unsigned i = 0; i < 1024; ++i) k[4] += T[i] == (0x33330000u | i);
            if (dramPage(phys[3], words)) for (unsigned i = 0; i < 1024; ++i) k[5] += words[i] == (0x33330000u | i);
        }
        (void)touch;   /* the loads are volatile; the sum only keeps them one expression */
        pass = k[0] == 6 && k[1] >= 48 && k[2] == 1024 && k[4] == 1024 && k[5] == 1024;
    }
    k[6] = uint32_t(st);
    if (handle) (void)vm->unmap(handle);
    // Whatever the result, no line of these pages stays in a CPU cache (a stale clean copy would outlive the free).
    if (buf) (void)NativeAdrenoMaintainOwnedPages(buf, kBytes, NativeAdrenoCleanInvalidate);
    (void)mem.release();
    if (buf) IOFreeAligned(buf, kBytes);
    k[7] = pass;
    native_adreno_io_coherent = pass ? 1 : 0;
    snprintf(native_gpu_coherence_row, sizeof native_gpu_coherence_row, "H d%u/64 a%u r%u/%u w%u/%u s%x k%u -> %s", k[1], gCoherenceAttempts,
             k[2], k[3], k[4], k[5], k[6], k[0], pass ? "on" : "off");
    IOLog("ASTEROIDS GPU: coherence %s\n", native_gpu_coherence_row);
}
void ucStart() {
    native_gpu_power_stage = 0x100;
    uint32_t *u = native_gpu_uc;
    if (native_gpu_bar[9] == 0 || native_gpu_bar[10]) {IOLog("ASTEROIDS GPU: uc skipped: mode 16 did not finish\n"); return;}
    NativeAdrenoVm *vm = NativeAdrenoGpuContextVm();
    const unsigned bits = native_gpu_context.output_bits;
    if (!vm || !bits) {IOLog("ASTEROIDS GPU: uc skipped: no GPU VM\n"); return;}
    u[0] = 1;
    native_gpu_power_stage = 0x101;
    // The pool is mapped whole (adoptPool keeps one write-combined kernel view). pandora's 1 GiB failed in ml_io_map's small IO submap
    // (pgpu2: kmem_alloc of 1 GiB failed, a panic); this map is IOKit's into the kernel map, which returns no map instead. So the full
    // size first, then halves down to the 128 MiB T15-T18 used, and the log says which size was had.
    uint64_t poolBytes = 0;
    for (uint64_t bytes = GPB_GPU_POOL_BYTES; bytes >= 0x8000000ULL && !poolBytes; bytes >>= 1) {
        IOLog("ASTEROIDS GPU: pool: %llx+%llx (loader-kept RAM outside XNU's window), pattern test, then adopt\n", GPB_GPU_POOL_BASE, bytes);
        IODeviceMemory *memory = IODeviceMemory::withRange(GPB_GPU_POOL_BASE, bytes);
        OSSharedPtr<IOMemoryMap> pool;
        if (memory) {
            pool = OSSharedPtr<IOMemoryMap>(memory->map(kIOMapAnywhere | kIOMapWriteCombineCache), OSNoRetain);
            memory->release();
        }
        if (!pool) {IOLog("ASTEROIDS GPU: pool map of %llu MiB failed\n", bytes >> 20); continue;}
        gpb_pool_result p{};
        gpb_pool_test(reinterpret_cast<volatile uint64_t *>(pool->getVirtualAddress()), bytes, &p);
        ringBarrier(nullptr);
        u[1] = uint32_t(p.pattern_bad + p.zero_bad);
        IOLog("ASTEROIDS GPU: pool words=%llu pattern_bad=%llu zero_bad=%llu first_bad=%llx\n", p.words, p.pattern_bad, p.zero_bad,
              p.first_bad);
        if (u[1] || p.words != bytes / 8) {native_gpu_power_stage = 0x10f; return;}
        poolBytes = bytes;
    }
    if (!poolBytes) {IOLog("ASTEROIDS GPU: pool map failed\n"); native_gpu_power_stage = 0x10f; return;}
    u[2] = uint32_t(NativeAdrenoMemory::adoptPool(GPB_GPU_POOL_BASE, size_t(poolBytes), bits));
    IOLog("ASTEROIDS GPU: pool adopt %x, %u MiB\n", u[2], unsigned(NativeAdrenoMemory::poolBytes() >> 20));
    if (u[2]) {native_gpu_power_stage = 0x10f; return;}
    u[0] = 2;
    native_gpu_power_stage = 0x102;
    const NativeAdrenoRingClock clock{nullptr, ringNow, ringIdle};
    u[3] = uint32_t(gSubmit.attach(vm, &gRing));
    u[4] = uint32_t(NativeAdrenoTimer::setRegisters(NativeAdrenoRegisterTable::gen8()));
    if (!u[4]) u[4] = uint32_t(gTimer.attach(&gTimerIo, NativeAdrenoTimeCp));
    uint64_t t0 = 0, t1 = 0;
    if (!u[4]) {gTimer.read(&t0); IODelay(1000); gTimer.read(&t1);}
    IOLog("ASTEROIDS GPU: fences: submit attach %x; timer attach %x, always-on %llu then %llu (+%llu in 1 ms)\n", u[3], u[4], t0, t1,
          t1 - t0);
    if (u[3] || u[4]) {native_gpu_power_stage = 0x10f; return;}
    coherenceProbe(vm, bits);
    if (&native_adreno_import_skip) native_adreno_import_skip = bootArg("sheng-gpu-import-skip", 0) == 1 ? 1 : 0;
    if (&native_adreno_import_policy) native_adreno_import_policy = bootArg("sheng-gpu-import-policy", 1) ? 1 : 0;
    u[0] = 3;
    native_gpu_power_stage = 0x103;
    u[5] = uint32_t(native_adreno_publish(vm, &gSubmit, &gRing, &clock, &gTimer, &gDeviceInfo, 31, bits));
    IOService *svc = u[5] ? nullptr : native_adreno_published_service();
    IOLog("ASTEROIDS GPU: publish %x service %s chip %08x gmem %x va %llx+%llx\n", u[5], svc ? "NativeAdrenoGpu" : "-",
          gDeviceInfo.chip_id, gDeviceInfo.gmem_bytes, gDeviceInfo.va_start, gDeviceInfo.va_size);
    if (!svc) {native_gpu_power_stage = 0x10f; return;}
    u[0] = 4;
    native_gpu_power_stage = 0x104;
    UcSteps step;
    const int v1 = ucRound(svc, step, 1);
    const int v2 = ucRound(svc, step, 2);
    u[6] = uint32_t(v1); u[7] = uint32_t(v2);
    u[0] = 5;
    native_gpu_power_stage = 0x105;
    const int burst = ucBurst(svc, step);
    u[12] = uint32_t(burst);
    u[8] = uint32_t(step.ok); u[9] = uint32_t(step.n); u[10] = uint32_t(step.err); u[11] = uint32_t(step.at);
    u[13] = native_adreno_pool_stats[1];
    u[14] = native_adreno_publish_stats(&u[15]);
    const bool ok = v1 == 4 && v2 == 4 && burst == 32 && step.ok == step.n;
    IOLog("ASTEROIDS GPU: uc steps %d/%d err %x@%d rounds %d/%d of 4 burst %d of 32, pool BOs %u, opened %u closed %u, jobs done %u %s\n",
          step.ok, step.n, unsigned(step.err), step.at, v1, v2, burst, u[13], u[14], u[15], unsigned(gSubmit.stats().completed),
          ok ? "ok" : "miss");
    cpState("uc");
    native_gpu_power_stage = ok ? 0x108 : 0x10e;
}
// Mode 18 (T16): what user space does with the published service. The GPU addon (boot-arg sheng-gpu-addon=1) brings a test
// daemon that opens NativeAdrenoGpu from user space, runs the shared user test (two rounds of three CP_MEM_WRITEs through its own
// mappings, uncached then write-combined) and then a Turnip smoke test (Vulkan instance, device, a compute fill and a triangle).
// It reports through the driver's open log, progress markers (refused opens of type 0x4147xxxx) and SetMetadata lines; this
// prints them every two seconds while they change, for ten minutes, then once more.
extern "C" {
// The kernel's GPU addon mount (patches/xnu/zzy-native-gpu-addon-mount.patch): weak, so a kernel without it still links.
extern volatile int native_gpu_addon_stage __attribute__((weak)), native_gpu_addon_status __attribute__((weak));
extern volatile int native_gpu_addon2_stage __attribute__((weak)), native_gpu_addon2_status __attribute__((weak));
// Its install step: files in the addon (-1 none), written, already equal, failed, last error.
extern volatile int native_gpu_addon_files __attribute__((weak)), native_gpu_addon_written __attribute__((weak)),
    native_gpu_addon_same __attribute__((weak)), native_gpu_addon_failed __attribute__((weak)), native_gpu_addon_error __attribute__((weak));
// The exec log (BootWatchdogPolicy.c, with sheng-gpu-addon=1): all execs, the last eight names, and the daemon's candidate names.
extern volatile uint32_t asteroids_exec_total __attribute__((weak)), asteroids_exec_ngpu __attribute__((weak)),
    asteroids_exec_bluetoothd __attribute__((weak)), asteroids_exec_fdr __attribute__((weak));
extern char asteroids_exec_last[8][24] __attribute__((weak));
extern char asteroids_exec_ngpu_path[64] __attribute__((weak));
// T16c: bluetoothd starts turned into the daemon, and the user the daemon ran as.
extern volatile uint32_t asteroids_exec_swaps __attribute__((weak));
extern volatile int32_t asteroids_exec_ngpu_uid __attribute__((weak));
// T20: the display's swap calls (LegacyDisplayProbe.cpp) and its successful or failed copies to the panel. T30: the T24+ display
// probe defines none of the asteroids_swap_* counters; LegacyDisplayStubs.c has weak zeros for them (the static link needs one).
extern volatile uint32_t asteroids_swap_end_calls __attribute__((weak_import)), asteroids_swap_end_size __attribute__((weak_import)),
    asteroids_swap_end_id __attribute__((weak_import)), asteroids_swap_end_status __attribute__((weak_import)),
    asteroids_swap_wait_calls __attribute__((weak_import)), native_legacy_transfer_calls __attribute__((weak_import));
uint32_t native_gpu_user[8];
}
// Mode 19 (T17): the accelerator for the original Metal discovery (IOServiceMatching("IOAcceleratorES") -> MetalPluginName ->
// /System/Library/Extensions/<name>.bundle), after UniXNU-sheng's publishMetal (BSD-2-Clause-Patent). The class is NativeAdreno's
// staged AGXAccelerator: its static initializer is out of __mod_init_func (build-native-legacy-display.py) and runs here, after the
// original IOGPUFamily kext has loaded, because the parent metaclass does not exist before that. The plugin bundle arrives through
// the GPU addon's Extensions overlay; without a NativeAdrenoMetal.publish file beside it, it only runs its tests in backboardd.
extern "C" void native_adreno_accelerator_static_init(void) __asm__("__GLOBAL__sub_I_NativeAdrenoAccelerator.cpp") __attribute__((weak));
extern "C" IOService *native_adreno_accelerator_allocate(void) __attribute__((weak));
extern "C" unsigned native_adreno_accelerator_instance_size(void) __attribute__((weak));
extern "C" kern_return_t OSKextLoadKextWithIdentifier(const char *);
extern "C" kmod_info_t g_kernel_kmod_info;
// Stages: 1 kext load attempted, 2 parent metaclass found, 3 own class registered, 4 object created and initialised, 5 published.
volatile uint32_t gMetalStage = 0, gMetalRegistered = 0;
volatile int gMetalKext = -99, gMetalStatus = 0;
void publishMetal() {
    gMetalStage = 1;
    if (!native_adreno_accelerator_static_init || !native_adreno_accelerator_allocate || !native_adreno_accelerator_instance_size) {
        gMetalStatus = kIOReturnUnsupported;
        return;
    }
    gMetalKext = OSKextLoadKextWithIdentifier("com.apple.iokit.IOGPUFamily");
    if (gMetalKext) {gMetalStatus = gMetalKext; return;}
    const OSSymbol *name = OSSymbol::withCString("IOAcceleratorES");
    const OSMetaClass *parent = name ? OSMetaClass::getMetaClassWithName(name) : nullptr;
    OSSafeReleaseNULL(name);
    if (!parent || parent->getClassSize() != 136) {gMetalStatus = kIOReturnNotFound; return;}
    gMetalStage = 2;
    void *handle = OSMetaClass::preModLoad(g_kernel_kmod_info.name);
    if (!handle) {gMetalStatus = kIOReturnNoResources; return;}
    native_adreno_accelerator_static_init();
    gMetalStatus = OSMetaClass::postModLoad(handle);
    if (gMetalStatus) return;
    if (native_adreno_accelerator_instance_size() != 136) {gMetalStatus = kIOReturnInvalid; return;}
    gMetalStage = 3;
    IOService *accelerator = native_adreno_accelerator_allocate();
    if (!accelerator) {gMetalStatus = kIOReturnNoMemory; return;}
    if (!accelerator->init()) {accelerator->release(); gMetalStatus = kIOReturnError; return;}
    // The two keys the original discovery reads. The plugin name is ours; no Apple device identity is claimed.
    accelerator->setProperty("MetalPluginName", "NativeAdrenoMetal");
    accelerator->setProperty("MetalPluginClassName", "NativeAdrenoMetalDevice");
    // The render server, the one process whose open the kernel grants in place of the sandbox verdict
    // (UniXNU patches/xnu/zzw-native-adreno-sandbox-grant.patch); T17's plugin tests only in backboardd.
    // T31: the render server plus the three Metal clients the plugin's NativeAdrenoMetal.clients lists (NativeAdreno has four connection
    // slots): the active wallpaper (WallpaperKit's CollectionsPoster), PosterBoard and iconservicesagent, after UniXNU-sheng's list.
    accelerator->setProperty("NativeSandboxGrantProcess", "backboardd,CollectionsPoster,PosterBoard,iconservicesagent");
    gMetalStage = 4;
    if (!accelerator->attach(IOService::getPlatform())) {accelerator->release(); gMetalStatus = kIOReturnNotAttached; return;}
    accelerator->registerService();
    gMetalRegistered = 1;
    gMetalStage = 5;
}
static void metalReport(const char *when) {
    const NativeAdrenoMetalDiag &d = native_adreno_metal_diag;
    IOLog("ASTEROIDS GPU: metal %s: stage %u status %x kext %x registered %u; property reads %u (last %.16s) MetalPluginName reads %u (%.16s)\n",
          when, gMetalStage, unsigned(gMetalStatus), unsigned(gMetalKext), gMetalRegistered, d.property_reads, d.reader[0], d.name_reads,
          d.name_reader);
}
static void userReport(const char *when) {
    const NativeAdrenoMetalDiag &d = native_adreno_metal_diag;
    const uint32_t n = d.marks_total, at = (n + 15) & 15;
    IOLog("ASTEROIDS GPU: user %s: addon %d/%d daemons %d/%d; opens %u closes %u busy %u bad %u foreign %u (type %x) mount_fail %u/%x\n",
          when, &native_gpu_addon_stage ? native_gpu_addon_stage : -9, &native_gpu_addon_status ? native_gpu_addon_status : -9,
          &native_gpu_addon2_stage ? native_gpu_addon2_stage : -9, &native_gpu_addon2_status ? native_gpu_addon2_status : -9,
          d.opens, d.closes, d.busy, d.bad, d.foreign_opens, d.foreign_type, native_adreno_mount_fail[0], native_adreno_mount_fail[1]);
    IOLog("ASTEROIDS GPU: user %s: calls %u tunnel %u probes %u last sel %u st %x inflight %u; marks %u: %x %x %x %x %x %x\n", when,
          d.calls, d.tunnel_calls, d.probes, d.last_selector, d.last_status, d.inflight, n, d.marks[(at + 11) & 15], d.marks[(at + 12) & 15],
          d.marks[(at + 13) & 15], d.marks[(at + 14) & 15], d.marks[(at + 15) & 15], d.marks[at & 15]);
    const uint32_t opened = native_adreno_open_log_count;
    for (uint32_t i = opened > 3 ? opened - 3 : 0; i < opened; ++i) {
        const NativeAdrenoOpenRecord &o = native_adreno_open_log[i & 7];
        IOLog("ASTEROIDS GPU: user %s: open %u by %.19s type %x macf %x res %x attach %u\n", when, i, o.name, o.type, o.macf, o.res, o.attach);
    }
#define GPB_WEAK(v) (&(v) ? int(v) : -9)
    IOLog("ASTEROIDS GPU: user %s: addon files %d written %d same %d failed %d error %d\n", when, GPB_WEAK(native_gpu_addon_files),
          GPB_WEAK(native_gpu_addon_written), GPB_WEAK(native_gpu_addon_same), GPB_WEAK(native_gpu_addon_failed), GPB_WEAK(native_gpu_addon_error));
    if (&asteroids_exec_total && asteroids_exec_last && asteroids_exec_ngpu_path) {
        const uint32_t e = asteroids_exec_total;
        IOLog("ASTEROIDS GPU: user %s: execs %u, ngpu %u bluetoothd %u fdr_loader %u; last %.23s %.23s %.23s %.23s\n", when, e,
              asteroids_exec_ngpu, asteroids_exec_bluetoothd, asteroids_exec_fdr, asteroids_exec_last[(e + 4) & 7],
              asteroids_exec_last[(e + 5) & 7], asteroids_exec_last[(e + 6) & 7], asteroids_exec_last[(e + 7) & 7]);
        IOLog("ASTEROIDS GPU: user %s: swaps %d, daemon uid %d\n", when, &asteroids_exec_swaps ? int(asteroids_exec_swaps) : -9,
              &asteroids_exec_ngpu_uid ? int(asteroids_exec_ngpu_uid) : -9);
        if (asteroids_exec_ngpu) IOLog("ASTEROIDS GPU: user %s: ngpu exec %.63s\n", when, asteroids_exec_ngpu_path);
    } else {
        IOLog("ASTEROIDS GPU: user %s: no exec log in this kernel\n", when);
    }
    IOLog("ASTEROIDS GPU: user %s: display swap-end %d (size %d id %d status %x) swap-wait %d copies %d\n", when,
          GPB_WEAK(asteroids_swap_end_calls), GPB_WEAK(asteroids_swap_end_size), GPB_WEAK(asteroids_swap_end_id),
          GPB_WEAK(asteroids_swap_end_status), GPB_WEAK(asteroids_swap_wait_calls), GPB_WEAK(native_legacy_transfer_calls));
#undef GPB_WEAK
    if (gMode >= 19) metalReport(when);
    IOLog("ASTEROIDS GPU: user %s: report \"%.63s\"\n", when, d.report);
    IOLog("ASTEROIDS GPU: user %s: turnip \"%.63s\"\n", when, d.report2);
}
void userWatch() {
    IOLog("ASTEROIDS GPU: user: watching the service for 10 minutes (%s, Turnip)\n", gMode >= 19 ? "the Metal plugin in backboardd" : "the addon's daemon");
    uint32_t last[6] = {~0u, ~0u, ~0u, ~0u, ~0u, ~0u};
    for (unsigned t = 0; t < 300; ++t) {
        const NativeAdrenoMetalDiag &d = native_adreno_metal_diag;
        const uint32_t execs = &asteroids_exec_ngpu ? asteroids_exec_ngpu + asteroids_exec_bluetoothd + asteroids_exec_fdr : 0;
        const uint32_t now[6] = {d.marks_total, d.opens + d.closes + d.foreign_opens + d.busy, d.reports, native_adreno_open_log_count, execs,
                                 d.property_reads + d.name_reads};
        if (memcmp(now, last, sizeof now)) {
            memcpy(last, now, sizeof now);
            char when[8];
            snprintf(when, sizeof when, "%us", t * 2);
            userReport(when);
        }
        IOSleep(2000);
    }
    native_gpu_user[0] = native_adreno_metal_diag.opens;
    native_gpu_user[1] = native_adreno_metal_diag.marks_total;
    native_gpu_user[2] = native_adreno_metal_diag.reports;
    userReport("end");
    cpState("user");
}
// T39: attach the GPU line (after mode 16, before the service is published, as sheng's irqAndTimer does after its zap step), then one
// CP_NOP fence: a working line counts interrupts after the attach (bits latched by the earlier fences) and again after the fence.
void irqAttach() {
    uint32_t *q = native_gpu_irq;
    if (!bootArg("sheng-gpu-irq", 1)) {q[0] = 0xff; IOLog("ASTEROIDS GPU: irq off (sheng-gpu-irq=0): fence waits spin\n"); return;}
    const unsigned us = bootArg("sheng-gpu-pause-us", 1000);
    if (us >= 20 && us <= 100000) gPauseUs = us;
    uint64_t router = ~0ull;
    q[0] = armcpu_spi_attach ? uint32_t(armcpu_spi_attach(kGpuIntid, false, gpuIrq, nullptr, &q[1], &router)) : 0xdead;
    gIrqAttached = q[0] == 0;
    IOSleep(20);
    q[2] = native_gpu_irq_count;
    uint32_t nop = 0, fence = 0, seen = 0;
    const NativeAdrenoRingClock clock{nullptr, ringNow, ringIdle};
    const bool fenced = AdrenoPM4Type7(0x10, 0, &nop) == AdrenoPM4Success && !gRing.submit(&nop, 1, &fence) &&
                        !gRing.wait(fence, &clock, 200000, &seen);
    IOSleep(20);
    q[3] = native_gpu_irq_count;
    q[4] = gIrqSeen;
    ringRead(nullptr, A810_RBBM_INT_0_STATUS, &q[5]);
    IOLog("ASTEROIDS GPU: irq intid %u attach %x gicd %x router %llx count %u, NOP fence %s then %u, seen %08x status %08x, pause %u us, "
          "sleeps %llu spins %llu timeouts %llu\n", kGpuIntid, q[0], q[1], (unsigned long long)router, q[2], fenced ? "ok" : "failed", q[3],
          q[4], q[5], gPauseUs, (unsigned long long)native_gpu_idle_sleeps, (unsigned long long)native_gpu_idle_spins,
          (unsigned long long)native_gpu_idle_timeouts);
}
// T39: GPU frequency governor (`sheng-gpu-dcvs=1`, after the service is published), UniXNU-sheng's gpuDcvsThread (a338c71a, 36ec562c)
// on the A810's levels. The GMU's XO power counter 0 counts 19.2 MHz ticks while the GPU is busy (hw_init's CX_BUSY_MASK 0xff000000,
// countable 5, enable; read at its A8XX offsets). Every 16 ms it takes the frequency that would have kept that window 50 % busy and
// goes up at once to the lowest level at or above 1.1 x of it, to the ceiling after a window over 85 % busy or 50 ms busy without a
// break; it goes down one level after 4 windows in a row (5 right after a step up) that would have needed at most 0.8 x the next lower
// level, and to the idle level after 80 ms without work. Work after the idle drop starts at the lowest level at or above max(2 x now,
// 578 MHz) for 50 ms, and a touch report holds the ceiling until `sheng-gpu-dcvs-boost-ms` (default 1000, 0 = off) after the last one.
// Levels and MHz come from the startup table the GMU was given (1 = 264 ... 9 = 1050 MHz on the 3a's bin). `sheng-gpu-dcvs-min` is
// the floor (default 3, 510 MHz); `sheng-gpu-dcvs-max` the ceiling, by default the boot vote (`sheng-gpu-level`, default 6, 763 MHz),
// which T34 kept below the top three because nothing throttles the GPU for temperature yet. asteroids T40e: `sheng-gpu-dcvs-idle`
// (default 1, 264 MHz) is the level after the idle drop, below the floor, which then applies only while there is work (or the wake
// boost runs): with the board's floor at level 6 the GPU otherwise sat at 763 MHz with nothing to draw. Each change is the boot's HFI
// GX_BW_PERF_VOTE with that level's bandwidth.
extern "C" {
// [0] 1 running, [1] level now, [2] changes, [3] failed votes, [4] busy % over the last 10 s, [5..12] % of those 10 s at the eight
// highest levels (2..9, 362..1050 MHz on the 3a's bin), [13] touch boosts, [14] wake boosts (sheng's layout, which BootLog.c reads)
volatile uint32_t native_gpu_dcvs[15];
// T43: how loaded the GPU is while it has work, to tell a GPU-bound frame from one held up elsewhere (opening apps stutters under
// blur): 16 ms windows with work, of them [1] under 25 % busy, [2] under 50 %, [3] under 85 %, [4] 85 % or more, [5] those at the
// ceiling (the GPU was the limit at the highest level allowed), [6] the longest run of fully busy windows (ms), [7] the ceiling level.
volatile uint32_t native_gpu_windows[8];
extern volatile uint32_t native_hid_input_events[8] __attribute__((weak_import));
}
uint64_t gpuBusyTicks() {
    uint32_t hi = 0, lo = 0, hi2 = 0;
    gxRead(nullptr, GPB_WIN_GMU, GPB_GMU_XOCLK_0_H, &hi);
    gxRead(nullptr, GPB_WIN_GMU, GPB_GMU_XOCLK_0_L, &lo);
    gxRead(nullptr, GPB_WIN_GMU, GPB_GMU_XOCLK_0_H, &hi2);
    if (hi2 != hi) gxRead(nullptr, GPB_WIN_GMU, GPB_GMU_XOCLK_0_L, &lo);
    return (uint64_t)hi2 << 32 | lo;
}
uint64_t gpuMHz(unsigned level) {return gStartup.gpu[level].hz / 1000000;}
bool gpuVote(unsigned level) {
    unsigned bw = 0;
    const unsigned perf = ShengGmuPerfVote(level, &bw);
    uint32_t error = 0;
    if (gxVote(nullptr, perf, bw, &error) || error) return false;
    native_gpu_perf_vote[0] = perf; native_gpu_perf_vote[1] = bw;
    native_gpu_mhz = uint32_t(gpuMHz(perf));
    return true;
}
unsigned gpuLevelFor(uint64_t mhz, unsigned ceiling) {
    unsigned l = 1;
    while (l < ceiling && gpuMHz(l) < mhz) ++l;
    return l;
}
extern "C" volatile uint32_t native_adreno_submit_ibs[4] __attribute__((weak_import));   // [0] submissions (NativeAdrenoSubmit.cpp)
void gpuDcvsThread(void *, wait_result_t) {
    if (gStartup.gpu_count < 2 || gStartup.gpu_count > 16) return;
    const unsigned top = unsigned(gStartup.gpu_count) - 1;
    unsigned ceiling = bootArg("sheng-gpu-dcvs-max", gBootPerf ? gBootPerf : top);
    if (ceiling < 1) ceiling = 1;
    if (ceiling > top) ceiling = top;
    unsigned floor = bootArg("sheng-gpu-dcvs-min", 3);
    if (floor < 1) floor = 1;
    if (floor > ceiling) floor = ceiling;
    unsigned idleLevel = bootArg("sheng-gpu-dcvs-idle", 1);
    if (idleLevel < 1) idleLevel = 1;
    if (idleLevel > floor) idleLevel = floor;
    const unsigned base = top > 8 ? top - 7 : 1;   // the eight highest levels go to [5..12]
    unsigned cur = native_gpu_perf_vote[0] ? native_gpu_perf_vote[0] : ceiling;
    native_gpu_dcvs[0] = 1; native_gpu_dcvs[1] = cur; native_gpu_windows[7] = ceiling;
    uint64_t lastBusy = gpuBusyTicks(), lastT = mach_absolute_time(), idleSince = lastT;
    uint64_t runNs = 0, winBusy = 0, winNs = 0, atLevel[16] = {};
    unsigned down = 0, hold = 0;
    uint32_t lastSubmits = &native_adreno_submit_ibs ? native_adreno_submit_ibs[0] : 0;
    uint64_t boostAbs = 0, touchEnd = 0, wakeEnd = 0, wakeAbs = 0;
    nanoseconds_to_absolutetime((uint64_t)bootArg("sheng-gpu-dcvs-boost-ms", 1000) * 1000000, &boostAbs);
    nanoseconds_to_absolutetime(50000000ull, &wakeAbs);
    uint32_t touches = &native_hid_input_events ? native_hid_input_events[6] : 0;
    bool idle = false;
    for (;;) {
        IOSleep(16);
        const uint64_t busy = gpuBusyTicks(), t = mach_absolute_time();
        uint64_t ns = 0;
        absolutetime_to_nanoseconds(t - lastT, &ns);
        uint64_t busyNs = (busy - lastBusy) * 625 / 12;   // 52.083 ns per XO tick
        if (busyNs > ns) busyNs = ns;
        lastBusy = busy; lastT = t;
        if (!ns) continue;
        winBusy += busyNs; winNs += ns; atLevel[cur] += ns;
        // T40e2: activity is new submissions or a busy share over 2 % of the window. busyNs > 0 || rptr != wptr never settled on the 3a:
        // 4 submissions in two minutes at 0 % busy and the idle drop never fired (T40e2 log, 63-194 s).
        const uint32_t submits = &native_adreno_submit_ibs ? native_adreno_submit_ibs[0] : 0;
        const bool active = submits != lastSubmits || busyNs * 50 > ns;
        lastSubmits = submits;
        if (active) {
            native_gpu_windows[0] = native_gpu_windows[0] + 1;
            const uint64_t pct = busyNs * 100 / ns;
            const unsigned b = pct < 25 ? 1 : pct < 50 ? 2 : pct < 85 ? 3 : 4;
            native_gpu_windows[b] = native_gpu_windows[b] + 1;
            if (b == 4 && cur == ceiling) native_gpu_windows[5] = native_gpu_windows[5] + 1;
        }
        const uint64_t need = busyNs * gpuMHz(cur) * 2 / ns;
        runNs = busyNs * 100 >= ns * 95 ? runNs + ns : 0;
        if (runNs / 1000000 > native_gpu_windows[6]) native_gpu_windows[6] = (uint32_t)(runNs / 1000000);
        unsigned want = cur;
        if (busyNs * 100 > ns * 85 || runNs >= 50000000ull) {
            want = ceiling;
        } else if (gpuLevelFor(need * 11 / 10, ceiling) > cur) {
            want = gpuLevelFor(need * 11 / 10, ceiling);
        } else if (active) {
            if (cur > floor && need * 10 <= gpuMHz(cur - 1) * 8) {
                if (++down >= 4 + hold) want = cur - 1;
            } else {
                down = 0;
            }
        } else {
            uint64_t idleNs = 0;
            absolutetime_to_nanoseconds(t - idleSince, &idleNs);
            if (idleNs >= 80000000ull) want = idleLevel;
        }
        if (active) idleSince = t;
        if (!active && want == idleLevel) idle = true;
        if (active && idle) {
            idle = false; wakeEnd = t + wakeAbs; native_gpu_dcvs[14] = native_gpu_dcvs[14] + 1;
        }
        if (boostAbs && &native_hid_input_events && native_hid_input_events[6] != touches) {
            touches = native_hid_input_events[6];
            if (t >= touchEnd) native_gpu_dcvs[13] = native_gpu_dcvs[13] + 1;
            touchEnd = t + boostAbs;
        }
        if (t < touchEnd) want = ceiling;
        else if (t < wakeEnd) {
            const unsigned wake = gpuLevelFor(gpuMHz(cur) * 2 > 578 ? gpuMHz(cur) * 2 : 578, ceiling);
            if (want < wake) want = wake;
        }
        // The floor holds while there is work or the wake boost runs; only an idle GPU goes below it.
        const unsigned low = active || t < wakeEnd || t < touchEnd ? floor : idleLevel;
        if (want < low) want = low;
        if (want > ceiling) want = ceiling;
        if (want != cur) {
            if (gpuVote(want)) {hold = want > cur; cur = want; native_gpu_dcvs[1] = cur; native_gpu_dcvs[2] = native_gpu_dcvs[2] + 1;}
            else native_gpu_dcvs[3] = native_gpu_dcvs[3] + 1;
            down = 0;
        }
        if (winNs >= 10000000000ull) {
            native_gpu_dcvs[4] = (uint32_t)(winBusy * 100 / winNs);
            for (unsigned l = base; l <= top && l < base + 8; ++l) native_gpu_dcvs[5 + l - base] = (uint32_t)(atLevel[l] * 100 / winNs);
            for (unsigned l = 0; l < 16; ++l) atLevel[l] = 0;
            winBusy = winNs = 0;
        }
    }
}
void startGpuDcvs() {
    thread_t thread = nullptr;
    if (kernel_thread_start(gpuDcvsThread, nullptr, &thread) == KERN_SUCCESS) thread_deallocate(thread);
    IOLog("ASTEROIDS GPU: dcvs started at level %u (%u MHz)\n", native_gpu_perf_vote[0], native_gpu_mhz);
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
    if (gMode >= 6) gmuPrerequisites(r);
    if (gMode >= 7) gmuClocksOn(r);
    if (gMode >= 8) gmuState(r);
    if (gMode >= 9) gmuColdStart();
    if (gMode >= 11) gpuGx();
    if (gMode >= 12) gpuSurvey();
    if (gMode >= 13) gpuHwInit();
    if (gMode >= 14) cpStart();
    if (gMode >= 15) zapStart();
    if (gMode >= 16) barStart();
    if (gMode >= 17 && native_gpu_bar[9] && !native_gpu_bar[10]) irqAttach();
    if (gMode >= 17) ucStart();
    if (gMode >= 19 && native_gpu_power_stage == 0x108) {
        publishMetal();
        metalReport("published");
    }
    if (gMode >= 17 && native_gpu_power_stage == 0x108 && bootArg("sheng-gpu-dcvs", 0)) startGpuDcvs();
    if (gMode >= 18 && native_gpu_power_stage == 0x108) userWatch();
    native_gpu_power_stage = 4;
}
}

extern "C" void sheng_gpu_power_probe(void) {
    // asteroids T33: mode 19 (the accelerator for the published Metal plugin) by default; sheng-gpu-power=0 leaves the GPU off.
    unsigned mode = 19;
    PE_parse_boot_argn("sheng-gpu-power", &mode, sizeof mode);
    if (!mode || native_gpu_power_stage) return;
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
