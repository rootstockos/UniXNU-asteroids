// SPDX-License-Identifier: BSD-2-Clause-Patent
// The 3a's touch bring-up (FocalTech FT3683G on QUPv3 wrapper 0 SE0, SPI), staged as docs/asteroids-touch.md in iosonandng.
// `asteroids-touch=<n>`:
//   1  read-only snapshot: the QUP0 clock votes and branches, SE0's DFS levels, the touch and SPI pins, the RPMh rail
//      addresses in the command DB, and the apps SMMU's stream table. SE0's own registers only when its clocks already run.
// Written from the hardware facts in that note (live DT, the vendor kernel's clock and pinctrl tables, ios-asteroids'
// measurements); no code from the GPL sources. Structure after UniXNU-sheng's TouchProbe.cpp (BSD-2-Clause-Patent).
// Results: the diagnostics page (row 42, native_touch_rows[0]) and "ASTEROIDS TOUCH:" lines in the kernel log, which reach
// the persistent unified log on Data; the summary is logged again at 60 s and 180 s, after the UI has taken the screen.
#include <IOKit/IOService.h>
#include <IOKit/IOLib.h>
#include <IOKit/IOMemoryDescriptor.h>
#include <libkern/c++/OSSharedPtr.h>
#include <kern/thread.h>
#include <pexpert/pexpert.h>
#include <string.h>

namespace {
// GCC (live DT clock-controller@100000; gcc-volcano.c).
constexpr uint64_t kGcc = 0x100000;
constexpr uint32_t kVote = 0x52008;                 // bits 18-22: wrap0 core_2x, core, m_ahb, s_ahb, s0
constexpr uint32_t kVoteQup0 = 0x1fu << 18;
constexpr uint32_t kHaltS0 = 0x18004, kHaltMAhb = 0x23000, kHaltSAhb = 0x23004, kHaltCore = 0x23008, kHaltCore2x = 0x23018;
constexpr uint32_t kS0Rcg = 0x18010;                // cmd +0, cfg +4, DFSR +0x14, perf cfg/M/N +0x1c/+0x5c/+0x9c (+4 per level)
constexpr unsigned kDfsLevels = 8;
constexpr uint32_t kClkOff = 1u << 31;
// TLMM: the DT's 0xf000000 plus REG_BASE 0x100000; GPIO n at +n*0x1000 (ctl +0, io +4, intr_cfg +8). Without REG_BASE a read
// reset the phone (ios-asteroids).
constexpr uint64_t kTlmm = 0xf100000;
constexpr unsigned kPins[] = {0, 1, 2, 3, 19, 20};  // SPI MISO, MOSI, CLK, CS; touch irq, reset
// QUPv3 wrapper 0 and SE0.
constexpr uint64_t kWrapper = 0xac0000, kSe0 = 0xa80000;
constexpr uint32_t kSeIfDisable = 0x64, kSeFwRevision = 0x68, kSeHwParam0 = 0xe24, kSeStatus = 0x40;
// AOP command DB (live DT /reserved-memory/aop_cmd_db_region@81c60000, 128 KiB).
constexpr uint64_t kCmdDb = 0x81c60000;
constexpr uint32_t kCmdDbBytes = 0x20000;
// apps SMMU (live DT apps-smmu@15000000): GR0 sCR0 +0, ID0 +0x20, SMRn +0x800, S2CRn +0xc00.
constexpr uint64_t kSmmu = 0x15000000;
constexpr uint16_t kStreamSe = 0x563, kStreamGpi = 0x576, kStreamUfs = 0x60;

struct Region {OSSharedPtr<IOMemoryMap> keep; uintptr_t va = 0;};
bool mapRegion(uint64_t phys, uint64_t length, Region &region, bool readOnly) {
    IODeviceMemory *memory = IODeviceMemory::withRange(phys, length);
    if (!memory) return false;
    region.keep = OSSharedPtr<IOMemoryMap>(memory->map(kIOMapAnywhere | kIOMapInhibitCache | (readOnly ? kIOMapReadOnly : 0)),
                                           OSNoRetain);
    memory->release();
    region.va = region.keep ? static_cast<uintptr_t>(region.keep->getVirtualAddress()) : 0;
    return region.va != 0;
}
// One plain 32-bit load: the SMMU is emulated by the hypervisor, which aborted a post-indexed form (ios-asteroids).
uint32_t rd32(uintptr_t base, uint32_t reg) {
    uint32_t value;
    __asm__ volatile("ldr %w0, [%1]" : "=r"(value) : "r"(base + reg) : "memory");
    return value;
}
// A branch with hardware clock gating (bit 1) reads CLK_OFF while idle and wakes on access.
bool running(uint32_t cbcr) {return !(cbcr & kClkOff) || (cbcr & 2u);}

struct Snapshot {
    uint32_t vote, s0, mahb, sahb, core, core2x;
    uint32_t rcgCmd, rcgCfg, dfsr, perf[kDfsLevels], m[kDfsLevels], n[kDfsLevels];
    uint32_t pin[6][3];
    bool seRead;
    uint32_t wrapperHw, seIf, seFw, seHw, seStatus;
    int dbStatus;                                    // 0 ok, -1 map failed, -2 bad magic
    uint32_t dbVersion, ldob11, ldob22;
    int smmuStatus;
    uint32_t scr0, id0, smrValid, smrCount;
    int smrSe, smrGpi, smrUfs;                       // index of the SMR matching the stream, -1 none
    uint32_t s2crSe, s2crUfs;
    int step, error;
};
Snapshot gSnap;
}

extern "C" {
char native_touch_rows[4][120];
volatile unsigned native_touch_stage_req;
volatile uint32_t native_touch_stage_done, native_touch_step;
volatile int native_touch_error;
}

namespace {
void rows(const Snapshot &s) {
    snprintf(native_touch_rows[0], sizeof native_touch_rows[0],
        "TCH T1 s%u/%u st%d e%d v%08x s0%08x ma%08x sa%08x se%c if%x fw%x l11 %x l22 %x smr563 %d/%d",
        native_touch_stage_done, native_touch_stage_req, s.step, s.error, s.vote, s.s0, s.mahb, s.sahb, s.seRead ? 'R' : '-',
        s.seIf, s.seFw, s.ldob11, s.ldob22, s.smrSe, s.smrGpi);
    snprintf(native_touch_rows[1], sizeof native_touch_rows[1],
        "TCH pins 0:%x/%x 1:%x/%x 2:%x/%x 3:%x/%x 19:%x/%x/%x 20:%x/%x dfs%x cfg%x",
        s.pin[0][0], s.pin[0][1], s.pin[1][0], s.pin[1][1], s.pin[2][0], s.pin[2][1], s.pin[3][0], s.pin[3][1],
        s.pin[4][0], s.pin[4][1], s.pin[4][2], s.pin[5][0], s.pin[5][1], s.dfsr, s.rcgCfg);
}
void logSummary(const char *when, const Snapshot &s) {
    IOLog("ASTEROIDS TOUCH: %s step=%d error=%d vote=%08x (qup0 %02x) cbcr s0=%08x m_ahb=%08x s_ahb=%08x core=%08x core2x=%08x\n",
          when, s.step, s.error, s.vote, (s.vote & kVoteQup0) >> 18, s.s0, s.mahb, s.sahb, s.core, s.core2x);
    IOLog("ASTEROIDS TOUCH: %s pins gpio0 %x/%x gpio1 %x/%x gpio2 %x/%x gpio3 %x/%x gpio19 %x/%x/%x gpio20 %x/%x/%x\n", when,
          s.pin[0][0], s.pin[0][1], s.pin[1][0], s.pin[1][1], s.pin[2][0], s.pin[2][1], s.pin[3][0], s.pin[3][1],
          s.pin[4][0], s.pin[4][1], s.pin[4][2], s.pin[5][0], s.pin[5][1], s.pin[5][2]);
    IOLog("ASTEROIDS TOUCH: %s se %s if_disable=%x fw=%x hw0=%08x status=%08x wrapper_hw=%08x\n", when,
          s.seRead ? "read" : "skipped (clocks off)", s.seIf, s.seFw, s.seHw, s.seStatus, s.wrapperHw);
    IOLog("ASTEROIDS TOUCH: %s cmddb status=%d version=%x ldob11=%x ldob22=%x\n", when, s.dbStatus, s.dbVersion, s.ldob11, s.ldob22);
    IOLog("ASTEROIDS TOUCH: %s smmu status=%d scr0=%08x id0=%08x smrs=%u valid=%u 0x563=%d (s2cr %08x) 0x576=%d 0x60=%d (s2cr %08x)\n",
          when, s.smmuStatus, s.scr0, s.id0, s.smrCount, s.smrValid, s.smrSe, s.s2crSe, s.smrGpi, s.smrUfs, s.s2crUfs);
}

void readClocks(Snapshot &s) {
    s.step = 1;
    Region s0, halt, vote;
    if (!mapRegion(kGcc + 0x18000, 0x1000, s0, true) || !mapRegion(kGcc + 0x23000, 0x1000, halt, true) ||
        !mapRegion(kGcc + 0x52000, 0x1000, vote, true)) {s.error = -12; return;}
    s.vote = rd32(vote.va, kVote & 0xfff);
    s.s0 = rd32(s0.va, kHaltS0 & 0xfff);
    s.mahb = rd32(halt.va, kHaltMAhb & 0xfff);
    s.sahb = rd32(halt.va, kHaltSAhb & 0xfff);
    s.core = rd32(halt.va, kHaltCore & 0xfff);
    s.core2x = rd32(halt.va, kHaltCore2x & 0xfff);
    s.rcgCmd = rd32(s0.va, kS0Rcg & 0xfff);
    s.rcgCfg = rd32(s0.va, (kS0Rcg + 4) & 0xfff);
    s.dfsr = rd32(s0.va, (kS0Rcg + 0x14) & 0xfff);
    for (unsigned l = 0; l < kDfsLevels; ++l) {
        s.perf[l] = rd32(s0.va, (kS0Rcg + 0x1c + 4 * l) & 0xfff);
        s.m[l] = rd32(s0.va, (kS0Rcg + 0x5c + 4 * l) & 0xfff);
        s.n[l] = rd32(s0.va, (kS0Rcg + 0x9c + 4 * l) & 0xfff);
        IOLog("ASTEROIDS TOUCH: dfs level %u cfg=%08x m=%08x n=%08x\n", l, s.perf[l], s.m[l], s.n[l]);
    }
}
void readPins(Snapshot &s) {
    s.step = 2;
    for (unsigned i = 0; i < sizeof kPins / sizeof *kPins; ++i) {
        Region pin;
        if (!mapRegion(kTlmm + kPins[i] * 0x1000ull, 0x1000, pin, true)) {s.error = -13; return;}
        for (unsigned r = 0; r < 3; ++r) s.pin[i][r] = rd32(pin.va, 4 * r);
    }
}
void readEngine(Snapshot &s) {
    s.step = 3;
    // Never touch the wrapper or the engine with their clocks off: an unclocked access can stall the bus.
    if (!running(s.mahb) || !running(s.sahb) || (s.core & kClkOff) || (s.core2x & kClkOff) || (s.s0 & kClkOff)) return;
    Region wrapper, se;
    if (!mapRegion(kWrapper, 0x2000, wrapper, true) || !mapRegion(kSe0, 0x4000, se, true)) {s.error = -14; return;}
    s.wrapperHw = rd32(wrapper.va, 0);
    s.seIf = rd32(se.va, kSeIfDisable);
    s.seFw = rd32(se.va, kSeFwRevision);
    s.seHw = rd32(se.va, kSeHwParam0);
    s.seStatus = rd32(se.va, kSeStatus);
    s.seRead = true;
}
// cmd_db_header: le32 version, magic, 8 slave headers of 16 bytes (le16 slv_id, header_offset, data_offset, cnt, version,
// reserved[3]), le32 checksum, le32 reserved, then the data; each entry 24 bytes: char id[8], le32 priority[2], le32 addr,
// le16 len, le16 offset.
uint32_t lookup(const volatile uint8_t *db, const char *name) {
    auto u16 = [&](uint32_t o) {return uint32_t(db[o]) | uint32_t(db[o + 1]) << 8;};
    auto u32 = [&](uint32_t o) {return u16(o) | u16(o + 2) << 16;};
    constexpr uint32_t kData = 8 + 8 * 16 + 8;
    for (unsigned h = 0; h < 8; ++h) {
        const uint32_t hdr = 8 + 16 * h, offset = u16(hdr + 2), count = u16(hdr + 6);
        if (!u16(hdr)) break;
        for (uint32_t e = 0; e < count && e < 512; ++e) {
            const uint32_t at = kData + offset + 24 * e;
            if (at + 24 > kCmdDbBytes) break;
            bool same = true;
            for (unsigned c = 0; c < 8 && same; ++c) {
                const char want = name[c];
                same = db[at + c] == static_cast<uint8_t>(want);
                if (!want) break;
            }
            if (same) return u32(at + 16);
        }
    }
    return 0;
}
void readCommandDb(Snapshot &s) {
    s.step = 4;
    Region db;
    if (!mapRegion(kCmdDb, kCmdDbBytes, db, true)) {s.dbStatus = -1; return;}
    // Copy with aligned 32-bit loads (uncached device memory), as GpuPowerProbe's mode 6.
    uint8_t *copy = static_cast<uint8_t *>(IOMallocData(kCmdDbBytes));
    if (!copy) {s.dbStatus = -12; return;}
    for (uint32_t i = 0; i < kCmdDbBytes / 4; ++i) {const uint32_t w = rd32(db.va, 4 * i); memcpy(copy + 4 * i, &w, 4);}
    memcpy(&s.dbVersion, copy, 4);
    uint32_t magic = 0;
    memcpy(&magic, copy + 4, 4);
    if (magic != 0x0c0330dbu) s.dbStatus = -2;
    else {s.ldob11 = lookup(copy, "ldob11"); s.ldob22 = lookup(copy, "ldob22");}
    IOFreeData(copy, kCmdDbBytes);
}
void readSmmu(Snapshot &s) {
    s.step = 5;
    s.smrSe = s.smrGpi = s.smrUfs = -1;
    Region gr0;
    if (!mapRegion(kSmmu, 0x1000, gr0, true)) {s.smmuStatus = -1; return;}
    s.scr0 = rd32(gr0.va, 0);
    s.id0 = rd32(gr0.va, 0x20);
    s.smrCount = s.id0 & 0xff;
    if (s.smrCount > 128) s.smrCount = 128;
    for (unsigned i = 0; i < s.smrCount; ++i) {
        const uint32_t smr = rd32(gr0.va, 0x800 + 4 * i);
        if (!(smr & (1u << 31))) continue;
        ++s.smrValid;
        const uint32_t s2cr = rd32(gr0.va, 0xc00 + 4 * i);
        const uint16_t id = smr & 0xffff, mask = (smr >> 16) & 0x7fff;
        IOLog("ASTEROIDS TOUCH: smr %u id=%x mask=%x s2cr=%08x (type %u cb %u)\n", i, id, mask, s2cr, (s2cr >> 16) & 3, s2cr & 0xff);
        auto matches = [&](uint16_t stream) {return ((id ^ stream) & ~mask & 0x7fff) == 0;};
        if (matches(kStreamSe) && s.smrSe < 0) {s.smrSe = int(i); s.s2crSe = s2cr;}
        if (matches(kStreamGpi) && s.smrGpi < 0) s.smrGpi = int(i);
        if (matches(kStreamUfs) && s.smrUfs < 0) {s.smrUfs = int(i); s.s2crUfs = s2cr;}
    }
}

void touchThread(void *, wait_result_t) {
    thread_set_thread_name(current_thread(), "asteroids-touch");
    Snapshot &s = gSnap;
    IOLog("ASTEROIDS TOUCH: T1 read-only snapshot start\n");
    readClocks(s); native_touch_step = 1; rows(s);
    if (!s.error) {readPins(s); native_touch_step = 2; rows(s);}
    if (!s.error) {readEngine(s); native_touch_step = 3; rows(s);}
    if (!s.error) {readCommandDb(s); native_touch_step = 4; rows(s);}
    if (!s.error) {readSmmu(s); native_touch_step = 5;}
    native_touch_error = s.error;
    if (!s.error) {s.step = 6; native_touch_stage_done = 1;}
    rows(s);
    logSummary("t1", s);
    IOSleep(60000);
    logSummary("t1 at 60 s", s);
    IOSleep(120000);
    logSummary("t1 at 180 s", s);
}
}

extern "C" void asteroids_touch_probe(void) {
    unsigned stage = 0;
    if (!PE_parse_boot_argn("asteroids-touch", &stage, sizeof stage) || stage < 1 || native_touch_stage_req) return;
    native_touch_stage_req = 1;   // only the read-only stage exists so far
    thread_t thread = nullptr;
    if (kernel_thread_start(touchThread, nullptr, &thread) == KERN_SUCCESS) thread_deallocate(thread);
    else native_touch_error = -12;
}
