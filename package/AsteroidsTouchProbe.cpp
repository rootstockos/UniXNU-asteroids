// SPDX-License-Identifier: BSD-2-Clause-Patent
// The 3a's touch bring-up (FocalTech FT3683G on QUPv3 wrapper 0 SE0, SPI), staged as docs/asteroids-touch.md in iosonandng.
// `asteroids-touch=<v>`: the stage is v & 15, the orientation of stage 6 is (v >> 4) & 7 (1 swap axes, 2 mirror x, 4 mirror y,
// applied in that order), so "asteroids-touch=0x16" is stage 6 with the axes swapped. Each boot runs every stage up to the one asked
// for and stops at the first failure:
//   1  read-only snapshot: the QUP0 clock votes and branches, SE0's DFS levels, the touch and SPI pins, the RPMh rail
//      addresses in the command DB, and the apps SMMU's stream table. SE0's own registers only when its clocks already run.
//   2  power: reset low, the 3.3 V (ldob22) and 1.8 V (ldob11) rails voted on active TCS 2 of the apps RSC, reset high; the QUP0
//      branch votes; the SPI pins; then SE0's identity (expect FW_REVISION 0x102, FIFO interface disabled) and 2 s of the
//      interrupt line.
//   3  bus: a 5 s countdown on the page, then the SMMU pass-through for stream 0x563 (the bank NativeUfs already set up), SE DMA
//      (UniXNU-sheng's way on SM8550 with the same disabled FIFO interface), the clock from DFS level 0 (TCXO) and a 16-byte
//      engine loopback.
//   4  chip: FocalTech read frames of 0xA3/0x9F (chip id, expect 0x56/0x72), 0xA6 (firmware) and 0xA8 (vendor), SPI mode 0
//      first, then 3, 1, 2.
//   5  reports: 84 bytes from register 0x01 every 10 ms, decoded (protocol v2 and the legacy layout), nothing published.
//   6  publishes the contacts to TouchState (scaled to 30480 x 20320); with sheng-touch-hid=1 NativeHidInput reports them.
// Written from the hardware facts in that note (live DT, the vendor kernel's clock and pinctrl tables and the FocalTech frame
// layout as facts, ios-asteroids' measurements); no code from the GPL sources. Structure after UniXNU-sheng's TouchProbe.cpp,
// SmmuBypass.h and QcomRpmh (BSD-2-Clause-Patent); the SPI engine is the core's QcomGeniSpi.
// Results: the diagnostics page (row 42, native_touch_rows[0]) and "ASTEROIDS TOUCH:" lines in the kernel log, which reach the
// persistent unified log on Data; the summary is logged again at 60 s and 180 s, after the UI has taken the screen.
#include <IOKit/IOService.h>
#include <IOKit/IOLib.h>
#include <IOKit/IOMemoryDescriptor.h>
#include <IOKit/IOBufferMemoryDescriptor.h>
#include <libkern/c++/OSSharedPtr.h>
#include <kern/thread.h>
#include <pexpert/pexpert.h>
#include <string.h>
extern "C" {
#include "GeniSpi.h"
#include "QcomRpmh.h"
#include "SmmuBypass.h"
}

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
// reset the phone (ios-asteroids). ctl: pull 1:0 (3 up), function 5:2, drive 8:6 ((mA/2)-1), output enable 9; io: in 0, out 1.
constexpr uint64_t kTlmm = 0xf100000;
constexpr unsigned kPins[] = {0, 1, 2, 3, 19, 20};  // SPI MISO, MOSI, CLK, CS; touch irq, reset
constexpr unsigned kPinIrq = 4, kPinReset = 5;      // indexes into kPins
constexpr uint32_t kSpiPinCfg = (1u << 2) | (2u << 6);   // function 1 (qup0_se0), 6 mA, no pull
constexpr uint32_t kIrqPinCfg = 3u;                       // GPIO input, pull-up
constexpr uint32_t kResetPinCfg = 1u << 9;                // GPIO output, 2 mA, no pull
// QUPv3 wrapper 0 and SE0.
constexpr uint64_t kWrapper = 0xac0000, kSe0 = 0xa80000;
constexpr uint32_t kSeIfDisable = 0x64, kSeFwRevision = 0x68, kSeHwParam0 = 0xe24, kSeStatus = 0x40;
// AOP command DB (live DT /reserved-memory/aop_cmd_db_region@81c60000, 128 KiB) and the apps RSC DRV2 (rsc@17a00000 drv@2,
// tcs-offset 0xd00; TCS 0-2 are active).
constexpr uint64_t kCmdDb = 0x81c60000;
constexpr uint32_t kCmdDbBytes = 0x20000;
constexpr uint64_t kRsc = 0x17a20000;
constexpr uint32_t kTcsOffset = 0xd00;
constexpr unsigned kTcs = 2;
// apps SMMU (live DT apps-smmu@15000000): GR0 sCR0 +0, ID0 +0x20, SMRn +0x800, S2CRn +0xc00; context banks above.
constexpr uint64_t kSmmu = 0x15000000;
constexpr uint64_t kSmmuBytes = 0xd0000;
constexpr uint16_t kStreamSe = 0x563, kStreamGpi = 0x576, kStreamUfs = 0x60;
// SPI: 0.96 MHz from DFS level 0 (TCXO 19.2 MHz), the rate ios-asteroids ran; the DT allows 12 MHz.
constexpr unsigned kSpiHz = 960000;
constexpr size_t kDmaBytes = 64 * 1024;
// FocalTech SPI v2 frames: read = reg, 0xA0 (read | CRC), length (big-endian), 3 dummy bytes, then the data and its CRC-16.
constexpr unsigned kHeader = 7, kReport = 84;
constexpr uint8_t kRegReport = 0x01, kRegChipHi = 0xA3, kRegChipLo = 0x9F, kRegFirmware = 0xA6, kRegVendor = 0xA8;
// The panel (display-coords 0 0 1080 2392) and TouchState's range.
constexpr int kPanelW = 1080, kPanelH = 2392, kStateW = 30480, kStateH = 20320;
constexpr unsigned kSlots = 12;

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
// One plain 32-bit load or store: the SMMU is emulated by the hypervisor, which aborted a post-indexed form (ios-asteroids).
uint32_t rd32(uintptr_t base, uint32_t reg) {
    uint32_t value;
    __asm__ volatile("ldr %w0, [%1]" : "=r"(value) : "r"(base + reg) : "memory");
    return value;
}
void wr32(uintptr_t base, uint32_t reg, uint32_t value) {
    __asm__ volatile("str %w0, [%1]\n\tdsb sy" :: "r"(value), "r"(base + reg) : "memory");
}
// A branch with hardware clock gating (bit 1) reads CLK_OFF while idle and wakes on access.
bool running(uint32_t cbcr) {return !(cbcr & kClkOff) || (cbcr & 2u);}

uint32_t ioRd(void *ctx, uint32_t reg) {return rd32(reinterpret_cast<uintptr_t>(ctx), reg);}
void ioWr(void *ctx, uint32_t reg, uint32_t value) {wr32(reinterpret_cast<uintptr_t>(ctx), reg, value);}
uint32_t smmuRd(void *ctx, unsigned o) {return rd32(reinterpret_cast<uintptr_t>(ctx), o);}
void smmuWr(void *ctx, unsigned o, uint32_t v) {wr32(reinterpret_cast<uintptr_t>(ctx), o, v);}
// Short waits spin on the counter (IODelay blocks the thread even for 1 us; UniXNU-sheng int-182).
void ioDelayUs(void *, unsigned us) {
    if (us > 50) {IODelay(us); return;}
    uint64_t frequency = 0, start = 0, now = 0;
    __asm__ volatile("mrs %0, cntfrq_el0" : "=r"(frequency));
    __asm__ volatile("isb\n\tmrs %0, cntpct_el0" : "=r"(start));
    const uint64_t ticks = (frequency ? frequency : 19200000) * us / 1000000 + 1;
    do {__asm__ volatile("isb\n\tmrs %0, cntpct_el0" : "=r"(now));} while (now - start < ticks);
}
// The engine's DMA master reaches the buffers through the pass-through mapping; caches are maintained explicitly.
void cacheRange(void *p, size_t len, int op) {   // 0 clean, 1 clean+invalidate, 2 invalidate
    uint64_t ctr;
    __asm__ volatile("mrs %0, ctr_el0" : "=r"(ctr));
    const size_t line = 4u << ((ctr >> 16) & 15);
    uintptr_t a = reinterpret_cast<uintptr_t>(p) & ~(line - 1);
    const uintptr_t end = reinterpret_cast<uintptr_t>(p) + len;
    for (; a < end; a += line) {
        if (op == 0) __asm__ volatile("dc cvac, %0" :: "r"(a) : "memory");
        else if (op == 1) __asm__ volatile("dc civac, %0" :: "r"(a) : "memory");
        else __asm__ volatile("dc ivac, %0" :: "r"(a) : "memory");
    }
    __asm__ volatile("dsb sy" ::: "memory");
}
void dmaClean(void *, void *p, size_t len) {cacheRange(p, len, 0);}
void dmaCleanInval(void *, void *p, size_t len) {cacheRange(p, len, 1);}
void dmaInval(void *, void *p, size_t len) {cacheRange(p, len, 2);}

// DFS level rate (clk-rcg2 calc_rate facts: source 10:8, mode 13:12, half-integer divider 4:0, N stored inverted).
uint64_t levelHz(uint32_t cfg, uint32_t mreg, uint32_t nreg) {
    const unsigned src = (cfg >> 8) & 7, mode = (cfg >> 12) & 3, hid = cfg & 0x1f;
    uint64_t rate = src == 0 ? 19200000ull : src == 1 ? 600000000ull : src == 6 ? 300000000ull : 0;
    if (hid) {rate *= 2; rate /= hid + 1;}
    if (mode) {
        const uint32_t m = mreg & 0xffff, n = ((~nreg) & 0xffff) + m;
        if (n) {rate *= m; rate /= n;}
    }
    return rate;
}
// CRC-16 over the data bytes: reflected polynomial 0x8408, initial 0xFFFF, no final xor; stored little-endian after the data.
uint16_t crc16(const uint8_t *p, size_t n) {
    uint16_t crc = 0xffff;
    for (size_t i = 0; i < n; ++i) {
        crc ^= p[i];
        for (unsigned b = 0; b < 8; ++b) crc = (crc & 1) ? static_cast<uint16_t>((crc >> 1) ^ 0x8408) : static_cast<uint16_t>(crc >> 1);
    }
    return crc;
}

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

struct Session {
    Snapshot s;
    bool rw;                                         // stage 2 and later write registers
    Region gccS0, gccHalt, gccVote, pins[6], wrapper, se, smmu, rsc;
    // Stage 2.
    uint32_t rscId, tcsStatus[2], irqLow, irqSamples, cbcrWait;
    // Stage 3.
    IOBufferMemoryDescriptor *dmaMem;
    uint64_t dmaPhys;
    int smmuSlot, smmuBank, smmuReuse, smmuReady;
    const char *smmuError;
    unsigned spiDiv;
    uint64_t sclkHz, spiHz;
    int loopRc, loopMatch;
    uint8_t loopRx[16];
    uint32_t loopTxIn, loopRxIn;
    struct geni_spi spi;
    // Stage 4.
    unsigned mode;
    int idRc;
    uint8_t chipHi, chipLo, firmware, vendor, idStatus;
    int idCrcOk;
    uint8_t firstRx[12];
    // Stage 5-6.
    uint8_t *tx, *rx;
    uint32_t reads, xferErrors, statusErrors, crcErrors, idle, initFrames, active, published;
    uint8_t last[24];
    unsigned lastType, lastCount, orient;
    int lastX, lastY;
};
Session *gSession;

void fail(Snapshot &s, int step, int code) {
    if (!s.error) {s.step = step; s.error = code ? code : -1;}
}
}

extern "C" {
char native_touch_rows[4][120];
volatile unsigned native_touch_stage_req;
volatile uint32_t native_touch_stage_done, native_touch_step;
volatile int native_touch_error;
extern volatile uint32_t native_touch_seq, native_touch_count;
extern volatile int native_touch_xs[12], native_touch_ys[12], native_touch_ids[12];
void native_touch_published(void);
}

namespace {
// Row 42 of the page shows only native_touch_rows[0]: the stage's own summary. The other rows go to the log.
void rows(const Session &x) {
    const Snapshot &s = x.s;
    char *r = native_touch_rows[0];
    const size_t n = sizeof native_touch_rows[0];
    const unsigned done = native_touch_stage_done;
    if (done < 2)
        snprintf(r, n, "TCH T1 s%u/%u st%d e%d v%08x s0%08x ma%08x sa%08x se%c if%x fw%x l11 %x l22 %x smr563 %d/%d",
            done, native_touch_stage_req, s.step, s.error, s.vote, s.s0, s.mahb, s.sahb, s.seRead ? 'R' : '-',
            s.seIf, s.seFw, s.ldob11, s.ldob22, s.smrSe, s.smrGpi);
    else if (done < 3)
        snprintf(r, n, "TCH T2 s%u/%u st%d e%d v%08x s0%08x fw%x if%x hw%08x rsc%08x tcs%x/%x irq low %u/%u rst%x",
            done, native_touch_stage_req, s.step, s.error, s.vote, s.s0, s.seFw, s.seIf, s.seHw, x.rscId, x.tcsStatus[0],
            x.tcsStatus[1], x.irqLow, x.irqSamples, s.pin[kPinReset][1]);
    else if (done < 4)
        snprintf(r, n, "TCH T3 s%u/%u st%d e%d smmu %d/%d r%d %s pa%llx %lluHz/d%u loop rc%d ok%d %02x%02x%02x%02x in%x/%x",
            done, native_touch_stage_req, s.step, s.error, x.smmuSlot, x.smmuBank, x.smmuReuse, x.smmuError ? x.smmuError : "ok",
            static_cast<unsigned long long>(x.dmaPhys), static_cast<unsigned long long>(x.spiHz), x.spiDiv, x.loopRc, x.loopMatch,
            x.loopRx[0], x.loopRx[1], x.loopRx[2], x.loopRx[3], x.loopTxIn, x.loopRxIn);
    else if (done < 5)
        snprintf(r, n, "TCH T4 s%u/%u st%d e%d mode%u rc%d id %02x %02x fw %02x ven %02x st%02x crc%d rx %02x%02x%02x%02x%02x%02x%02x%02x",
            done, native_touch_stage_req, s.step, s.error, x.mode, x.idRc, x.chipHi, x.chipLo, x.firmware, x.vendor, x.idStatus,
            x.idCrcOk, x.firstRx[0], x.firstRx[1], x.firstRx[2], x.firstRx[3], x.firstRx[4], x.firstRx[5], x.firstRx[6], x.firstRx[7]);
    else
        snprintf(r, n, "TCH T%u e%d rd%u x%u st%u crc%u idle%u init%u act%u pub%u t%x n%u %d,%d %02x%02x%02x%02x%02x%02x%02x%02x",
            done, s.error, x.reads, x.xferErrors, x.statusErrors, x.crcErrors, x.idle, x.initFrames, x.active, x.published,
            x.lastType, x.lastCount, x.lastX, x.lastY, x.last[0], x.last[1], x.last[2], x.last[3], x.last[4], x.last[5],
            x.last[6], x.last[7]);
}
void mark(Session &x, int step) {x.s.step = step; native_touch_step = static_cast<uint32_t>(step); rows(x);}

void logSummary(const char *when, const Session &x) {
    const Snapshot &s = x.s;
    IOLog("ASTEROIDS TOUCH: %s stage %u/%u step=%d error=%d vote=%08x (qup0 %02x) cbcr s0=%08x m_ahb=%08x s_ahb=%08x core=%08x core2x=%08x\n",
          when, native_touch_stage_done, native_touch_stage_req, s.step, s.error, s.vote, (s.vote & kVoteQup0) >> 18, s.s0,
          s.mahb, s.sahb, s.core, s.core2x);
    IOLog("ASTEROIDS TOUCH: %s pins gpio0 %x/%x gpio1 %x/%x gpio2 %x/%x gpio3 %x/%x gpio19 %x/%x/%x gpio20 %x/%x/%x\n", when,
          s.pin[0][0], s.pin[0][1], s.pin[1][0], s.pin[1][1], s.pin[2][0], s.pin[2][1], s.pin[3][0], s.pin[3][1],
          s.pin[4][0], s.pin[4][1], s.pin[4][2], s.pin[5][0], s.pin[5][1], s.pin[5][2]);
    IOLog("ASTEROIDS TOUCH: %s se %s if_disable=%x fw=%x hw0=%08x status=%08x wrapper_hw=%08x\n", when,
          s.seRead ? "read" : "skipped (clocks off)", s.seIf, s.seFw, s.seHw, s.seStatus, s.wrapperHw);
    IOLog("ASTEROIDS TOUCH: %s cmddb status=%d version=%x ldob11=%x ldob22=%x\n", when, s.dbStatus, s.dbVersion, s.ldob11, s.ldob22);
    IOLog("ASTEROIDS TOUCH: %s smmu status=%d scr0=%08x id0=%08x smrs=%u valid=%u 0x563=%d (s2cr %08x) 0x576=%d 0x60=%d (s2cr %08x)\n",
          when, s.smmuStatus, s.scr0, s.id0, s.smrCount, s.smrValid, s.smrSe, s.s2crSe, s.smrGpi, s.smrUfs, s.s2crUfs);
    if (native_touch_stage_req < 2) return;
    IOLog("ASTEROIDS TOUCH: %s power rsc=%08x tcs %x/%x irq low %u of %u cbcr wait %u\n", when, x.rscId, x.tcsStatus[0],
          x.tcsStatus[1], x.irqLow, x.irqSamples, x.cbcrWait);
    if (native_touch_stage_req < 3) return;
    IOLog("ASTEROIDS TOUCH: %s bus smmu slot %d bank %d reuse %d ready %d (%s) dma %llx sclk %llu spi %llu div %u loop rc %d match %d "
          "rx %02x%02x%02x%02x%02x%02x%02x%02x in %x/%x\n", when, x.smmuSlot, x.smmuBank, x.smmuReuse, x.smmuReady,
          x.smmuError ? x.smmuError : "ok", static_cast<unsigned long long>(x.dmaPhys), static_cast<unsigned long long>(x.sclkHz),
          static_cast<unsigned long long>(x.spiHz), x.spiDiv, x.loopRc, x.loopMatch, x.loopRx[0], x.loopRx[1], x.loopRx[2],
          x.loopRx[3], x.loopRx[4], x.loopRx[5], x.loopRx[6], x.loopRx[7], x.loopTxIn, x.loopRxIn);
    if (native_touch_stage_req < 4) return;
    IOLog("ASTEROIDS TOUCH: %s chip mode %u rc %d id %02x/%02x fw %02x vendor %02x status %02x crc %d\n", when, x.mode, x.idRc,
          x.chipHi, x.chipLo, x.firmware, x.vendor, x.idStatus, x.idCrcOk);
    if (native_touch_stage_req < 5) return;
    IOLog("ASTEROIDS TOUCH: %s reports %u xfer errors %u status errors %u crc errors %u idle %u init %u active %u published %u "
          "last type %x n %u at %d,%d\n", when, x.reads, x.xferErrors, x.statusErrors, x.crcErrors, x.idle, x.initFrames, x.active,
          x.published, x.lastType, x.lastCount, x.lastX, x.lastY);
}

// ---- Stage 1 (read-only) ----
void readClocks(Session &x) {
    Snapshot &s = x.s;
    mark(x, 1);
    if (!mapRegion(kGcc + 0x18000, 0x1000, x.gccS0, !x.rw) || !mapRegion(kGcc + 0x23000, 0x1000, x.gccHalt, !x.rw) ||
        !mapRegion(kGcc + 0x52000, 0x1000, x.gccVote, !x.rw)) {fail(s, 1, -12); return;}
    s.vote = rd32(x.gccVote.va, kVote & 0xfff);
    s.s0 = rd32(x.gccS0.va, kHaltS0 & 0xfff);
    s.mahb = rd32(x.gccHalt.va, kHaltMAhb & 0xfff);
    s.sahb = rd32(x.gccHalt.va, kHaltSAhb & 0xfff);
    s.core = rd32(x.gccHalt.va, kHaltCore & 0xfff);
    s.core2x = rd32(x.gccHalt.va, kHaltCore2x & 0xfff);
    s.rcgCmd = rd32(x.gccS0.va, kS0Rcg & 0xfff);
    s.rcgCfg = rd32(x.gccS0.va, (kS0Rcg + 4) & 0xfff);
    s.dfsr = rd32(x.gccS0.va, (kS0Rcg + 0x14) & 0xfff);
    for (unsigned l = 0; l < kDfsLevels; ++l) {
        s.perf[l] = rd32(x.gccS0.va, (kS0Rcg + 0x1c + 4 * l) & 0xfff);
        s.m[l] = rd32(x.gccS0.va, (kS0Rcg + 0x5c + 4 * l) & 0xfff);
        s.n[l] = rd32(x.gccS0.va, (kS0Rcg + 0x9c + 4 * l) & 0xfff);
        IOLog("ASTEROIDS TOUCH: dfs level %u cfg=%08x m=%08x n=%08x (%llu Hz)\n", l, s.perf[l], s.m[l], s.n[l],
              static_cast<unsigned long long>(levelHz(s.perf[l], s.m[l], s.n[l])));
    }
}
void readPins(Session &x) {
    Snapshot &s = x.s;
    mark(x, 2);
    for (unsigned i = 0; i < sizeof kPins / sizeof *kPins; ++i) {
        if (!mapRegion(kTlmm + kPins[i] * 0x1000ull, 0x1000, x.pins[i], !x.rw)) {fail(s, 2, -13); return;}
        for (unsigned r = 0; r < 3; ++r) s.pin[i][r] = rd32(x.pins[i].va, 4 * r);
    }
}
bool clocksRunning(const Snapshot &s) {
    return running(s.mahb) && running(s.sahb) && !(s.core & kClkOff) && !(s.core2x & kClkOff) && !(s.s0 & kClkOff);
}
void readEngine(Session &x) {
    Snapshot &s = x.s;
    mark(x, 3);
    // Never touch the wrapper or the engine with their clocks off: an unclocked access can stall the bus.
    if (!clocksRunning(s)) return;
    if ((!x.wrapper.va && !mapRegion(kWrapper, 0x2000, x.wrapper, !x.rw)) ||
        (!x.se.va && !mapRegion(kSe0, 0x4000, x.se, !x.rw))) {fail(s, 3, -14); return;}
    s.wrapperHw = rd32(x.wrapper.va, 0);
    s.seIf = rd32(x.se.va, kSeIfDisable);
    s.seFw = rd32(x.se.va, kSeFwRevision);
    s.seHw = rd32(x.se.va, kSeHwParam0);
    s.seStatus = rd32(x.se.va, kSeStatus);
    s.seRead = true;
}
// cmd_db_header: le32 version, magic, 8 slave headers of 16 bytes (le16 slv_id, header_offset, data_offset, cnt, version,
// reserved[3]), le32 checksum, le32 reserved, then the data; each entry 24 bytes: char id[8], le32 priority[2], le32 addr,
// le16 len, le16 offset.
uint32_t lookup(const uint8_t *db, const char *name) {
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
void readCommandDb(Session &x) {
    Snapshot &s = x.s;
    mark(x, 4);
    Region db;
    if (!mapRegion(kCmdDb, kCmdDbBytes, db, true)) {s.dbStatus = -1; return;}
    // Copy with aligned 32-bit loads (uncached device memory), as GpuPowerProbe's mode 6.
    uint8_t *copy = static_cast<uint8_t *>(IOMallocData(kCmdDbBytes));
    if (!copy) {s.dbStatus = -12; return;}
    for (uint32_t i = 0; i < kCmdDbBytes / 4; ++i) {const uint32_t w = rd32(db.va, 4 * i); memcpy(copy + 4 * i, &w, 4);}
    memcpy(&s.dbVersion, copy, 4);
    uint32_t magic = 0;
    memcpy(&magic, copy + 4, 4);
    if (magic != QCOM_CMD_DB_MAGIC) s.dbStatus = -2;
    else {s.ldob11 = lookup(copy, "ldob11"); s.ldob22 = lookup(copy, "ldob22");}
    IOFreeData(copy, kCmdDbBytes);
}
void readSmmu(Session &x) {
    Snapshot &s = x.s;
    mark(x, 5);
    s.smrSe = s.smrGpi = s.smrUfs = -1;
    if (!mapRegion(kSmmu, kSmmuBytes, x.smmu, !x.rw)) {s.smmuStatus = -1; return;}
    s.scr0 = rd32(x.smmu.va, 0);
    s.id0 = rd32(x.smmu.va, 0x20);
    s.smrCount = s.id0 & 0xff;
    if (s.smrCount > 128) s.smrCount = 128;
    for (unsigned i = 0; i < s.smrCount; ++i) {
        const uint32_t smr = rd32(x.smmu.va, 0x800 + 4 * i);
        if (!(smr & (1u << 31))) continue;
        ++s.smrValid;
        const uint32_t s2cr = rd32(x.smmu.va, 0xc00 + 4 * i);
        const uint16_t id = smr & 0xffff, mask = (smr >> 16) & 0x7fff;
        IOLog("ASTEROIDS TOUCH: smr %u id=%x mask=%x s2cr=%08x (type %u cb %u)\n", i, id, mask, s2cr, (s2cr >> 16) & 3, s2cr & 0xff);
        auto matches = [&](uint16_t stream) {return ((id ^ stream) & ~mask & 0x7fff) == 0;};
        if (matches(kStreamSe) && s.smrSe < 0) {s.smrSe = int(i); s.s2crSe = s2cr;}
        if (matches(kStreamGpi) && s.smrGpi < 0) s.smrGpi = int(i);
        if (matches(kStreamUfs) && s.smrUfs < 0) {s.smrUfs = int(i); s.s2crUfs = s2cr;}
    }
}

// ---- Stage 2: power, clocks, pins, engine identity ----
void setPin(Session &x, unsigned index, uint32_t ctl) {wr32(x.pins[index].va, 0, ctl); x.s.pin[index][0] = rd32(x.pins[index].va, 0);}
void setReset(Session &x, bool high) {
    wr32(x.pins[kPinReset].va, 4, high ? 2u : 0u);
    x.s.pin[kPinReset][1] = rd32(x.pins[kPinReset].va, 4);
}
// One active RPMh request on TCS 2 (the same TCS UniXNU-sheng's audio rail vote uses): mode, voltage and enable of a VRM.
bool voteRail(Session &x, uint32_t address, uint32_t millivolts, unsigned which) {
    const QcomRscLayout layout = qcom_rsc_layout(x.rscId);
    const uint32_t addresses[3] = {address + 8, address, address + 4}, values[3] = {7, millivolts, 1};   // high power mode, mV, on
    QcomRpmhWrite writes[16];
    const unsigned n = qcom_rpmh_active_request(&layout, kTcs, addresses, values, 3, writes, 16);
    if (!n) return false;
    for (unsigned i = 0; i < n; ++i) wr32(x.rsc.va, kTcsOffset + writes[i].offset, writes[i].value);
    // The completion is the TCS's bit in the IRQ status register; give it up to 20 ms, then return the TCS to idle either way.
    uint32_t status = 0;
    for (unsigned i = 0; i < 200 && !((status = rd32(x.rsc.va, kTcsOffset + layout.irq_status)) & (1u << kTcs)); ++i) IODelay(100);
    x.tcsStatus[which] = status;
    const unsigned m = qcom_rpmh_finish(&layout, kTcs, writes, 16);
    for (unsigned i = 0; i < m; ++i) wr32(x.rsc.va, kTcsOffset + writes[i].offset, writes[i].value);
    return (status & (1u << kTcs)) != 0;
}
bool power(Session &x) {
    Snapshot &s = x.s;
    mark(x, 20);
    if (!s.ldob11 || !s.ldob22) {fail(s, 20, -2); return false;}
    // Controller in reset before its rails come up (the vendor driver's order); the interrupt line as an input with pull-up.
    setReset(x, false);
    setPin(x, kPinReset, kResetPinCfg);
    setPin(x, kPinIrq, kIrqPinCfg);
    mark(x, 21);
    if (!mapRegion(kRsc, 0x10000, x.rsc, false)) {fail(s, 21, -12); return false;}
    x.rscId = rd32(x.rsc.va, 0);
    mark(x, 22);
    const bool vci = voteRail(x, s.ldob22, 3300, 0);
    mark(x, 23);
    const bool vddio = voteRail(x, s.ldob11, 1800, 1);
    if (!vci || !vddio) {fail(s, 23, -110); return false;}
    IOSleep(2);
    setReset(x, true);
    IOSleep(200);
    // QUP0's five branch votes (a shared vote register), then wait until the branches run.
    mark(x, 24);
    wr32(x.gccVote.va, kVote & 0xfff, rd32(x.gccVote.va, kVote & 0xfff) | kVoteQup0);
    bool ok = false;
    for (unsigned i = 0; i < 2000 && !ok; ++i) {
        s.vote = rd32(x.gccVote.va, kVote & 0xfff);
        s.s0 = rd32(x.gccS0.va, kHaltS0 & 0xfff);
        s.mahb = rd32(x.gccHalt.va, kHaltMAhb & 0xfff);
        s.sahb = rd32(x.gccHalt.va, kHaltSAhb & 0xfff);
        s.core = rd32(x.gccHalt.va, kHaltCore & 0xfff);
        s.core2x = rd32(x.gccHalt.va, kHaltCore2x & 0xfff);
        ok = clocksRunning(s);
        x.cbcrWait = i;
        if (!ok) IODelay(10);
    }
    if (!ok) {fail(s, 24, -110); return false;}   // never touch the engine without running clocks
    for (unsigned i = 0; i < 4; ++i) setPin(x, i, kSpiPinCfg);
    mark(x, 25);
    readEngine(x);
    if (s.error) return false;
    if (!s.seRead) {fail(s, 25, -110); return false;}
    // 2 s of the interrupt line (active low): the first sign of a running controller in ios-asteroids' runs.
    mark(x, 26);
    for (unsigned i = 0; i < 2000; ++i) {
        if (!(rd32(x.pins[kPinIrq].va, 4) & 1)) ++x.irqLow;
        ++x.irqSamples;
        IODelay(1000);
    }
    s.pin[kPinIrq][1] = rd32(x.pins[kPinIrq].va, 4);
    return true;
}

// ---- Stage 3: SMMU route, SE DMA, clock, loopback ----
bool openBus(Session &x) {
    Snapshot &s = x.s;
    for (unsigned left = 5; left; --left) {   // a reset from here on is attributable to the route or the first DMA transfer
        snprintf(native_touch_rows[0], sizeof native_touch_rows[0], "TCH T3 SMMU route for 0x563 and the first SPI DMA in %u s", left);
        IOLog("ASTEROIDS TOUCH: smmu route and the first SPI DMA in %u s\n", left);
        IOSleep(1000);
    }
    mark(x, 30);
    x.spi.se = (struct geni_io){reinterpret_cast<void *>(x.se.va), ioRd, ioWr, ioDelayUs};
    x.spi.wrapper = (struct geni_io){reinterpret_cast<void *>(x.wrapper.va), ioRd, ioWr, ioDelayUs};
    int r = geni_spi_read_info(&x.spi);
    if (r) {fail(s, 30, r); return false;}
    mark(x, 31);
    x.dmaMem = IOBufferMemoryDescriptor::inTaskWithPhysicalMask(kernel_task,
        static_cast<IOOptionBits>(kIODirectionInOut) | static_cast<IOOptionBits>(kIOMemoryPhysicallyContiguous), 2 * kDmaBytes,
        0xfffffff000ULL);
    if (!x.dmaMem || x.dmaMem->prepare() != kIOReturnSuccess) {fail(s, 31, -12); return false;}
    addr64_t segment = 0;
    x.dmaPhys = x.dmaMem->getPhysicalSegment(0, &segment);
    uint8_t *virt = static_cast<uint8_t *>(x.dmaMem->getBytesNoCopy());
    if (!virt || !x.dmaPhys || segment < 2 * kDmaBytes) {fail(s, 31, -14); return false;}
    memset(virt, 0, 2 * kDmaBytes);
    cacheRange(virt, 2 * kDmaBytes, 1);
    mark(x, 32);
    struct smmu_io io = {reinterpret_cast<void *>(x.smmu.va), smmuRd, smmuWr};
    struct smmu_plan plan = smmu_plan_bypass(&io, kStreamSe, x.dmaPhys, 2 * kDmaBytes);
    x.smmuSlot = static_cast<int>(plan.slot); x.smmuBank = static_cast<int>(plan.bank);
    x.smmuReuse = plan.reuse; x.smmuReady = plan.bank_ready;
    IOLog("ASTEROIDS TOUCH: smmu plan slot %d bank %d pages %u reuse %d ready %d error %s\n", x.smmuSlot, x.smmuBank, plan.pages,
          plan.reuse, plan.bank_ready, plan.error ? plan.error : "none");
    if (plan.error) {x.smmuError = plan.error; fail(s, 32, -100); return false;}
    mark(x, 33);
    x.smmuError = smmu_configure_bypass(&io, &plan, kStreamSe);
    if (x.smmuError) {fail(s, 33, -101); return false;}
    IOLog("ASTEROIDS TOUCH: smmu routed 0x563 smr %d -> bank %d\n", x.smmuSlot, x.smmuBank);
    x.spi.use_dma = 1;
    x.spi.dma.ctx = nullptr;
    x.spi.dma.clean = dmaClean;
    x.spi.dma.clean_inval = dmaCleanInval;
    x.spi.dma.inval = dmaInval;
    x.spi.dma.tx_virt = virt;
    x.spi.dma.rx_virt = virt + kDmaBytes;
    x.spi.dma.tx_phys = x.dmaPhys;
    x.spi.dma.rx_phys = x.dmaPhys + kDmaBytes;
    x.spi.dma.size = kDmaBytes;
    x.spi.poll_limit = 400000;
    x.spi.poll_only = 2;             // keep the top-level IRQ enable: the status registers must stay visible to the poll
    mark(x, 34);
    r = geni_spi_init(&x.spi);
    if (r) {fail(s, 34, r); return false;}
    // DFS level 0 only (TCXO): no PLL vote needed, and the rate ios-asteroids proved.
    mark(x, 35);
    x.sclkHz = levelHz(s.perf[0], s.m[0], s.n[0]);
    if (!x.sclkHz || !x.spi.oversampling) {fail(s, 35, -22); return false;}
    uint64_t div = (x.sclkHz + uint64_t(x.spi.oversampling) * kSpiHz - 1) / (uint64_t(x.spi.oversampling) * kSpiHz);
    if (div < 1) div = 1;
    x.spiDiv = static_cast<unsigned>(div);
    x.spiHz = x.sclkHz / (x.spi.oversampling * div);
    r = geni_spi_set_clock(&x.spi, kSpiHz, static_cast<unsigned>(x.sclkHz), 0);
    if (r) {fail(s, 35, r); return false;}
    // Engine loopback: MOSI fed back to MISO inside the controller, so this tests the DMA, packing and clock path alone.
    mark(x, 36);
    static const uint8_t pattern[16] = {0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88, 0x99, 0xaa, 0xbb, 0xcc, 0xdd, 0xee, 0xff, 0x00};
    uint8_t got[16] = {};
    geni_spi_setup(&x.spi, GENI_SPI_LOOP, 0);
    x.loopRc = geni_spi_transfer(&x.spi, pattern, got, sizeof pattern, 0);
    memcpy(x.loopRx, got, sizeof got);
    x.loopTxIn = x.spi.last_tx_in;
    x.loopRxIn = x.spi.last_rx_in;
    x.loopMatch = memcmp(got, pattern, sizeof pattern) == 0;
    geni_spi_setup(&x.spi, 0, 0);
    if (x.loopRc) {fail(s, 36, x.loopRc); return false;}
    if (!x.loopMatch) {fail(s, 36, -5); return false;}
    return true;
}

// ---- Stage 4: the FocalTech frame and the chip id ----
// Read n bytes of register reg; the data lands in out. Returns 0, a GENI error, -70 status bits set, -71 CRC mismatch.
int ftsRead(Session &x, uint8_t reg, uint8_t *out, unsigned n, bool keepFirst) {
    const unsigned len = kHeader + n + 2;
    memset(x.tx, 0, len);
    x.tx[0] = reg;
    x.tx[1] = 0xA0;
    x.tx[2] = static_cast<uint8_t>(n >> 8);
    x.tx[3] = static_cast<uint8_t>(n);
    int rc = 0;
    for (unsigned attempt = 0; attempt < 3; ++attempt) {
        rc = geni_spi_transfer(&x.spi, x.tx, x.rx, len, 0);
        ioDelayUs(nullptr, 150);   // CS high between frames
        if (keepFirst && !attempt) memcpy(x.firstRx, x.rx, sizeof x.firstRx);
        if (rc) {++x.xferErrors; continue;}
        x.idStatus = x.rx[3];
        if (x.rx[3] & 0xA0) {++x.statusErrors; rc = -70; continue;}
        // A CRC mismatch is counted and logged, not dropped: the CRC parameters are read from the vendor driver, not yet seen
        // on the wire, so a wrong guess must not hide good frames. The status byte above already passed.
        const uint16_t want = static_cast<uint16_t>(x.rx[kHeader + n] | x.rx[kHeader + n + 1] << 8);
        const uint16_t got = crc16(x.rx + kHeader, n);
        memcpy(out, x.rx + kHeader, n);
        if (got != want) {
            if (++x.crcErrors <= 8) IOLog("ASTEROIDS TOUCH: crc reg %02x len %u computed %04x received %04x\n", reg, n, got, want);
            return -71;
        }
        return 0;
    }
    memcpy(out, x.rx + kHeader, n);   // the last attempt's bytes, for the log
    return rc;
}
bool identify(Session &x) {
    Snapshot &s = x.s;
    mark(x, 40);
    x.tx = static_cast<uint8_t *>(IOMallocData(256));
    x.rx = static_cast<uint8_t *>(IOMallocData(256));
    if (!x.tx || !x.rx) {fail(s, 40, -12); return false;}
    static const unsigned order[4] = {0, 3, 1, 2};
    for (unsigned i = 0; i < 4; ++i) {
        x.mode = order[i];
        geni_spi_setup(&x.spi, ((x.mode & 1) ? GENI_SPI_CPHA : 0) | ((x.mode & 2) ? GENI_SPI_CPOL : 0), 0);
        uint8_t hi = 0, lo = 0;
        x.idRc = ftsRead(x, kRegChipHi, &hi, 1, i == 0);
        if (!x.idRc || x.idRc == -71) {
            const int rcLo = ftsRead(x, kRegChipLo, &lo, 1, false);
            if (rcLo && rcLo != -71) x.idRc = rcLo;
        }
        x.chipHi = hi; x.chipLo = lo;
        x.idCrcOk = x.idRc != -71;
        IOLog("ASTEROIDS TOUCH: id mode %u rc %d chip %02x/%02x status %02x rx %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x\n",
              x.mode, x.idRc, hi, lo, x.idStatus, x.rx[0], x.rx[1], x.rx[2], x.rx[3], x.rx[4], x.rx[5], x.rx[6], x.rx[7], x.rx[8],
              x.rx[9]);
        rows(x);
        if ((!x.idRc || x.idRc == -71) && hi == 0x56) break;
    }
    if (x.idRc && x.idRc != -71) {fail(s, 41, x.idRc); return false;}
    if (x.chipHi != 0x56) {fail(s, 41, -19); return false;}
    ftsRead(x, kRegFirmware, &x.firmware, 1, false);
    ftsRead(x, kRegVendor, &x.vendor, 1, false);
    return true;
}

// ---- Stages 5-6: reports ----
// Decoded contact in panel pixels.
struct Contact {int id, x, y;};
unsigned decode(const uint8_t *b, unsigned &type, Contact *out) {
    // b[0] is the first data byte (register 0x01's content).
    type = b[1] >> 4;
    unsigned n = 0;
    if (type == 2) {                     // protocol v2: count in b[1] & 15, 8-byte records from b[4]
        const unsigned count = b[1] & 15;
        for (unsigned i = 0; i < count && i < 10 && n < kSlots; ++i) {
            const uint8_t *r = b + 4 + 8 * i;
            if ((r[0] >> 6) == 1) continue;  // lifted
            out[n].id = r[2] >> 4;
            out[n].x = (((r[0] & 15) << 12) | (r[1] << 4) | (r[4] >> 4)) / 16;
            out[n].y = (((r[2] & 15) << 12) | (r[3] << 4) | (r[4] & 15)) / 16;
            ++n;
        }
    } else if (type == 0) {              // legacy: 6-byte records from b[2], 12-bit pixels, event in the top bits of x, id 15 ends
        for (unsigned i = 0; i < 10 && n < kSlots; ++i) {
            const uint8_t *r = b + 2 + 6 * i;
            const int id = r[2] >> 4;
            if (id >= 15) break;
            if ((r[0] >> 6) == 1) continue;  // lifted
            out[n].id = id;
            out[n].x = ((r[0] & 15) << 8) | r[1];
            out[n].y = ((r[2] & 15) << 8) | r[3];
            ++n;
        }
    }
    return n;
}
void publish(Session &x, const Contact *c, unsigned n) {
    uint32_t seq = __atomic_load_n(&native_touch_seq, __ATOMIC_RELAXED);
    __atomic_store_n(&native_touch_seq, seq + 1, __ATOMIC_RELEASE);
    __asm__ volatile("dmb ish" ::: "memory");
    for (unsigned i = 0; i < n; ++i) {
        int px = c[i].x, py = c[i].y, w = kPanelW, h = kPanelH;
        if (x.orient & 1) {const int t = px; px = py; py = t; w = kPanelH; h = kPanelW;}
        if (x.orient & 2) px = w - 1 - px;
        if (x.orient & 4) py = h - 1 - py;
        if (px < 0) px = 0;
        if (py < 0) py = 0;
        native_touch_xs[i] = static_cast<int>(int64_t(px) * kStateW / w);
        native_touch_ys[i] = static_cast<int>(int64_t(py) * kStateH / h);
        native_touch_ids[i] = c[i].id;
    }
    native_touch_count = n;
    __asm__ volatile("dmb ish" ::: "memory");
    __atomic_store_n(&native_touch_seq, seq + 2, __ATOMIC_RELEASE);
    native_touch_published();
    ++x.published;
}
void reportLoop(Session &x, bool publishing) {
    mark(x, 50);
    uint8_t frame[kReport];
    unsigned logged = 0, lastPublished = 0;
    uint64_t round = 0;
    for (;;) {
        ++round;
        const int rc = ftsRead(x, kRegReport, frame, kReport, false);
        ++x.reads;
        if (!rc || rc == -71) {
            if (frame[1] == 0xff && frame[2] == 0xff && frame[3] == 0xff && frame[4] == 0xff) {
                // All 0xFF: idle, or the firmware just (re)started.
                ++x.idle;
                if (frame[0] == 0xff) ++x.initFrames;
                if (publishing && lastPublished) {publish(x, nullptr, 0); lastPublished = 0;}
            } else {
                Contact c[kSlots];
                unsigned type = 0;
                const unsigned n = decode(frame, type, c);
                x.lastType = type; x.lastCount = n;
                memcpy(x.last, frame, sizeof x.last);
                if (n) {++x.active; x.lastX = c[0].x; x.lastY = c[0].y;}
                if (logged < 32) {
                    ++logged;
                    IOLog("ASTEROIDS TOUCH: frame %u type %x n %u %d,%d raw %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x\n",
                          x.reads, type, n, n ? c[0].x : -1, n ? c[0].y : -1, frame[0], frame[1], frame[2], frame[3], frame[4],
                          frame[5], frame[6], frame[7], frame[8], frame[9], frame[10], frame[11]);
                }
                if (publishing && (n || lastPublished)) {publish(x, c, n); lastPublished = n;}
            }
        }
        if (!(round % 10)) rows(x);
        if (!(round % 1000)) logSummary("loop", x);   // every ~10 s
        IOSleep(10);
    }
}

void touchThread(void *, wait_result_t) {
    thread_set_thread_name(current_thread(), "asteroids-touch");
    Session &x = *gSession;
    Snapshot &s = x.s;
    const unsigned stage = native_touch_stage_req;
    x.rw = stage >= 2;
    IOLog("ASTEROIDS TOUCH: stage %u (orientation %u) start\n", stage, x.orient);
    readClocks(x);
    if (!s.error) readPins(x);
    if (!s.error) readEngine(x);
    if (!s.error) readCommandDb(x);
    if (!s.error) readSmmu(x);
    native_touch_error = s.error;
    if (!s.error) {s.step = 6; native_touch_stage_done = 1;}
    rows(x);
    logSummary("t1", x);
    do {
        if (s.error || stage < 2) break;
        if (!power(x)) break;
        native_touch_stage_done = 2; rows(x); logSummary("t2", x);
        if (stage < 3) break;
        if (!openBus(x)) break;
        native_touch_stage_done = 3; rows(x); logSummary("t3", x);
        if (stage < 4) break;
        if (!identify(x)) break;
        native_touch_stage_done = 4; rows(x); logSummary("t4", x);
        if (stage < 5) break;
        native_touch_stage_done = stage >= 6 ? 6 : 5;
        reportLoop(x, stage >= 6);   // does not return
    } while (false);
    native_touch_error = s.error;
    rows(x);
    logSummary("end", x);
    IOSleep(60000);
    logSummary("at 60 s", x);
    IOSleep(120000);
    logSummary("at 180 s", x);
}
}

extern "C" void asteroids_touch_probe(void) {
    unsigned value = 0;
    if (!PE_parse_boot_argn("asteroids-touch", &value, sizeof value) || !(value & 15) || native_touch_stage_req) return;
    gSession = static_cast<Session *>(IOMallocType(Session));
    if (!gSession) {native_touch_error = -12; return;}
    bzero(gSession, sizeof *gSession);
    unsigned stage = value & 15;
    if (stage > 6) stage = 6;
    gSession->orient = (value >> 4) & 7;
    gSession->smmuSlot = gSession->smmuBank = -1;
    gSession->loopRc = 99;
    native_touch_stage_req = stage;
    thread_t thread = nullptr;
    if (kernel_thread_start(touchThread, nullptr, &thread) == KERN_SUCCESS) thread_deallocate(thread);
    else native_touch_error = -12;
}
