// SPDX-License-Identifier: BSD-2-Clause-Patent
// asteroids Adreno 810 power observation and CX power-up, modes 1-3, the GMU pool round trip, mode 4, the GMU's SMMU
// context on that pool, mode 5, the read-only GMU clock and command-DB snapshot, mode 6, the GMU clock bring-up, mode 7,
// the first GMU and RSCC register reads, mode 8, the GMU firmware load to HFI ready, mode 9, and the whole cold start with
// the HFI startup messages, mode 10, the GX request with the first RBBM reads, mode 11, the SW fuse and CP survey with
// the GPU's own SMMU context and the SQE copy, mode 12, a8xx hw_init's register writes, mode 13, and the CP start through
// CP_ME_INIT and a first CP_MEM_WRITE, mode 14 (sheng-gpu-power=1..14; absent or 0 does nothing). Modes 9-10 hand the
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
// gpb_gmu_context_gate passes; the GMU itself stays in reset, so no transaction uses the context yet. Modes 9-14 write
// through ShengGmu's adapter instead: the GMU from mode 9, the GPU block only from mode 13 (gGpuWrites), CX_MISC never.
#include <IOKit/IOLib.h>
#include <IOKit/IOMemoryDescriptor.h>
#include <IOKit/IODeviceMemory.h>
#include <libkern/c++/OSSharedPtr.h>
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
#include "NativeAdrenoRing.h"
#include "NativeAdrenoCpInit.h"
#include "NativeAdrenoPwrup.h"
#include "AdrenoPM4.h"
#include "ShengSqeFirmware.h"   // build/sheng-gmu: the pinned gen80300_sqe.fw as an array; never committed
#include <kern/clock.h>

extern "C" {
struct gpb_result native_gpu_power_result;
struct gpb_pool_result native_gpu_pool_result;
struct NativeAdrenoGmuContextReport native_gpu_gmu_context;
volatile uint32_t native_gpu_power_stage;
// NativeAdrenoGpuContext.cpp reads it; its owner, NativeAdrenoUserClient.cpp, is not linked here. 0: no pool remap.
volatile uint32_t native_adreno_wc_enabled;
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
// Mode 11: only after mode 10 returned success.
void gpuGx() {
    native_gpu_power_stage = 0xa0;
    if (native_gpu_gmu_start_status != ShengGmuSuccess) {
        IOLog("ASTEROIDS GPU: gx skipped: cold start status %u\n", native_gpu_gmu_start_status);
        return;
    }
    unsigned bw = 0;
    const unsigned perf = ShengGmuPerfVote(1, &bw);
    IOLog("ASTEROIDS GPU: gx vote perf %u bw %u (%lluk), then OOB GPU_SET\n", perf, bw,
          static_cast<unsigned long long>(gStartup.gpu[perf].hz / 1000));
    native_gpu_power_stage = 0xa1;
    gpb_gx_io io{nullptr, gxRead, gxWrite, gxVote, gxDelay};
    gpb_gx &x = native_gpu_gx;
    gpb_gx_on(&io, perf, bw, &x);
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
//   2 NativeAdrenoGpuContextBringup: bank 0 (arm-smmu-qcom gives the GPU bank 0), ASID 2, the DT's SID 0 on SMR slot 0
//     and SID 1 (mainline's second adreno stream, the one CP_SMMU_STREAM_ID_LPAC names) on slot 1; the secure SID 2 is
//     left out as on uke, where its SMR write aborted. Mode 5's GMU context keeps bank 1 and slot 2.
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
    IOLog("ASTEROIDS GPU: gpuctx bank 0 asid 2, sid 0 on slot 0, sid 1 on slot 1\n");
    NativeAdrenoGmuContextReport &q = native_gpu_context;
    const IOReturn status = NativeAdrenoGpuContextBringup(reinterpret_cast<volatile uint8_t *>(gGpuSmmu->getVirtualAddress()),
                                                          GPB_SMMU_WINDOW, 0, 2, 0, 0, 1, 1, gpuCtxProgress, &q);
    IOLog("ASTEROIDS GPU: gpuctx s%u %s st=%x k%u/%u oas=%u p%uw%ur%ue%u r%uw%uv%u spans=%u mismatch=%03x\n", q.step,
          q.step < 8 ? kGmuStep[q.step] : "?", static_cast<unsigned>(status), q.program_status, q.route_status,
          q.output_bits, q.program_phase, q.program_writes, q.program_readback_valid, q.program_enabled, q.route_phase,
          q.route_writes, q.routes_verified, q.spans, q.mismatch);
    IOLog("ASTEROIDS GPU: gpuctx ttbr0=%016llx sctlr=%08x cbar=%08x smr=%08x s2cr=%08x\n",
          static_cast<unsigned long long>(q.ttbr0), q.sctlr, q.cbar, q.smr, q.s2cr);
    const uint32_t id1 = native_gpu_power_result.id1;
    const size_t cb0 = gpb_smmu_cb_offset(id1, 0);
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
int ringWrite(void *, uint32_t reg, uint32_t value) {
    return gColdDevice.io.write32(gColdDevice.io.cookie, ShengGmuGpu, size_t(reg) * 4, value);
}
int ringRead(void *, uint32_t reg, uint32_t *value) {
    return gColdDevice.io.read32(gColdDevice.io.cookie, ShengGmuGpu, size_t(reg) * 4, value);
}
int ringBarrier(void *) {return gColdDevice.io.barrier(gColdDevice.io.cookie);}
// Ring, memptrs and lists are Normal non-cacheable on both sides (GPU MAIR 0, CPU write-combine): ordering only.
int ringPublish(void *, const volatile uint32_t *, size_t) {return ringBarrier(nullptr);}
int ringAcquire(void *, volatile uint32_t *, size_t) {return ringBarrier(nullptr);}
uint64_t ringNow(void *) {uint64_t us = 0; gmuNowUs(nullptr, &us); return us;}
int ringIdle(void *) {IODelay(20); return 0;}
NativeAdrenoRing gRing;
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
    const NativeAdrenoRingIo rio{nullptr, ringWrite, ringRead, ringBarrier};
    const NativeAdrenoRingCache cache{nullptr, ringPublish, ringAcquire};
    const IOReturn attached = gRing.attach(&rio, &cache, const_cast<uint8_t *>(gGpuCpu[1]), gGpuBuffers[1].bytes,
        gGpuBuffers[1].iova, const_cast<uint8_t *>(gGpuCpu[2]), gGpuBuffers[2].bytes, gGpuBuffers[2].iova);
    if (attached) {IOLog("ASTEROIDS GPU: cp ring attach %x\n", static_cast<unsigned>(attached)); return;}
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
