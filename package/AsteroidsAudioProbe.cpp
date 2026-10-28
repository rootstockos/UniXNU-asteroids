// SPDX-License-Identifier: BSD-2-Clause-Patent
// Speaker playback on the Nothing Phone (3a) (boot argument sheng-audio, bits: 1 serve, 2 play a test tone first, 8 press the power button
// once at 150 s for the lock sound; needs sheng-adsp=2). From UniXNU-sheng package/AudioProbe.cpp: the DSP graphs, the service loop and
// the shared timeline are sheng's; the board steps are the 3a's. The microphone (WCD9378) comes up in stages with asteroids-mic=<n>; see
// micBringUp. One kernel thread brings the path up:
//   1  pass-through stream mapping in the apps SMMU for the DSP's audio streams (0x1001, 0x1061), and the DSP's period buffers
//   2  pins: primary MI2S (gpio15 sck, 16 data0, 17 data1, 18 ws, function 2), the amplifiers' I2C (gpio59/60, function 1) and their two
//      reset lines (gpio141 P, gpio142 S: high 1 ms, then low, as NXP's driver pulses them)
//   3  QUP clocks and the GENI I2C engine (QUPv3 wrapper 1 SE6 = SE13, 0x898000) at 400 kHz from the 19.2 MHz DFS level
//   4  the two TFA9873 amplifiers: revision, then the container's settings (AsteroidsTfa9873.c); they stay powered down
//   5  the audio DSP: "adsp_apps" packet router channel, framework readiness, the LPASS core vote and the MI2S bit clock through the PRM
//      (the vendor's machine driver votes both: qcom,mi2s-tdm-is-hw-vote-needed), speaker device and playback stream graphs, buffer
//      mapping
//   6  the audio service (ShengAudioService.cpp) that iOS's audio server plug-in opens; with sheng-audio=2 a 750 Hz test tone plays first
//      for sheng-audio-tone-ms (default 1500), attenuated by sheng-audio-atten dB (default 30)
// The amplifiers take two 32-bit slots per frame (TDMSSIZE/TDMSLLN 31 in the container: P slot 0, S slot 1), so the speaker device runs
// 32-bit while the stream stays 16-bit; the stream graph's converter widens it. asteroids-audio-i2s = interface | data lines << 8 | bits
// << 16 overrides the port (default primary, SD1, 32: the 3a's amplifiers take their data on gpio17, T40k-spk-sd1; SD0 gave clocks but
// silence). asteroids-audio-prm (bits, default 3): 1 LPASS core vote, 2 MI2S bit clock.
// On start the amplifiers power up after the graphs run: they wait for the bit clock (CLKS) before the amplifier turns on.
#include <IOKit/IOLib.h>
#include <IOKit/IOMemoryDescriptor.h>
#include <IOKit/IOBufferMemoryDescriptor.h>
#include <IOKit/IODeviceMemory.h>
#include <libkern/c++/OSSharedPtr.h>
#include <kern/thread.h>
#include <kern/clock.h>
#include <pexpert/pexpert.h>
#include <string.h>
extern "C" {
#include "ShengAdsp.h"
#include "ShengGpr.h"
#include "ShengAudioReach.h"
#include "AsteroidsTfa9873.h"
#include "GeniI2c.h"
#include "SmmuBypass.h"
#include "QcomRpmh.h"
#include "ShengAudioAbi.h"
#include "BootResetCore.h"
}
#include <IOKit/IOService.h>
#include <libkern/OSKextLib.h>
#include <mach/kmod.h>

extern kmod_info_t g_kernel_kmod_info;
extern "C" void native_audio_service_static_init(void) __asm__("__GLOBAL__sub_I_ShengAudioService.cpp");
extern "C" void sheng_audio_service_start(void);

extern "C" ShengAdspChannel *sheng_adsp_channel(const char *, uint32_t, uint32_t, int, const ShengAdspClient *, void *) __attribute__((weak_import));
extern "C" int sheng_adsp_send(ShengAdspChannel *, const void *, uint32_t) __attribute__((weak_import));
// [0] stage (see the list above, x10 for sub-steps, 60 serving; negative: the step that failed), [1] last DSP status, [2] DSP
// answers, [3] buffer map handle, [4] periods sent, [5] periods returned, [6] amplifiers identified (bit per amplifier),
// [7] amplifiers with their clocks stable (CLKS), [8] amplifiers reporting errors, [9] SMMU result (bit per stream, or an error code << 8),
// [10] I2C engine (proto << 24 | clock config), [11] PRM results: core vote status | bit clock status << 8 (0xff no answer), [12] last
// opcode received, [13] I2C failures, [14] output starts | stops << 16, [15] ms from start to serving
extern "C" {volatile int32_t native_audio_stats[16];}
extern "C" {volatile int32_t native_audio_passed;}   // buffer answers command() passed on to handle()
// Per amplifier (rows 0 P, 1 S of the log's six): revision, power-up result (1 its clock wait ran out), STATUS0, STATUS1 while playing.
extern "C" {volatile uint32_t native_audio_amps[6][4];}
extern "C" {volatile uint64_t native_audio_pa;}   // the DSP period buffer

namespace {
constexpr uint32_t kIntentSize = 512, kIntentCount = 20, kRing = 16, kMessage = 128;
constexpr uint32_t kPort = 0x2001;                   // our GPR port; every answer comes back to it
constexpr uint32_t kRate = SHENG_AUDIO_RATE, kChannels = SHENG_AUDIO_OUT_CHANNELS, kPeriodFrames = SHENG_AUDIO_PERIOD_FRAMES;
constexpr uint32_t kPeriodBytes = kPeriodFrames * kChannels * 2, kSlots = 8, kBufferBytes = 64 * 1024, kRingFrames = SHENG_AUDIO_RING_FRAMES;
// kLeadFrames: the least lead the plug-in is given (300 ms; sheng's i539), raised per run when the DSP's queue needs more (syncPlayback).
constexpr uint32_t kInFlight = 4, kLeadFrames = 30 * kPeriodFrames, kKeepMs = 5000;
constexpr uint64_t kGcc = 0x00100000ULL, kTlmm = 0x0f100000ULL, kSmmu = 0x15000000ULL, kSmmuBytes = 0xd0000ULL, kSe = 0x00898000ULL;
// GCC (gcc-volcano.c): GPLL0 (mode 0x0, vote 0x52020 bit 0); QUPv3 wrapper 1 votes in 0x52010: core 0, s_ahb 1, m_ahb 2, core_2x 3, s6 10;
// their branch halt registers; SE6's DFS clock source.
constexpr uint32_t kGpll0Mode = 0x0, kGpll0Vote = 0x52020, kVote = 0x52010, kVoteBits = 0xfu | 1u << 10;
constexpr uint32_t kHalts[] = {0x23158, 0x23154, 0x23150, 0x23168, 0x1e62c};
constexpr uint32_t kRcg = 0x1e638, kRcgPerfCfg = 0x1c, kDfsLevels = 8;
constexpr uint32_t kCbcrOff = 1u << 31;
// TLMM pins (pinctrl-volcano: 0x1000 per gpio): MI2S function 2 at 8 mA, I2C function 1 at 2 mA with pull-up, amplifier resets.
constexpr unsigned kI2sPins[] = {15, 16, 17, 18}, kI2cPins[] = {59, 60};
constexpr uint16_t kStreams[] = {0x1001, 0x1061};   // the DT's adsp audio iommus
// GENI I2C at 400 kHz from 19.2 MHz (i2c-qcom-geni.c): divider 2, high 5, low 11, cycle 22.
constexpr uint32_t kSclCounters = 5u << 20 | 11u << 10 | 22u, kClkCfg = 2u << 4 | 1u;
// PRM (q6prm.h): the LPASS core hardware vote, and the primary MI2S internal bit clock at 48 kHz x 2 slots x 32 bits.
constexpr uint32_t kPrmClkPriMi2sIbit = 0x100, kPrmClkAttrCoupleNo = 1;
const int16_t kSine[64] = {0, 3212, 6393, 9512, 12539, 15446, 18204, 20787, 23170, 25329, 27245, 28898, 30273, 31356, 32137, 32609,
    32767, 32609, 32137, 31356, 30273, 28898, 27245, 25329, 23170, 20787, 18204, 15446, 12539, 9512, 6393, 3212, 0, -3212, -6393, -9512,
    -12539, -15446, -18204, -20787, -23170, -25329, -27245, -28898, -30273, -31356, -32137, -32609, -32767, -32609, -32137, -31356, -30273,
    -28898, -27245, -25329, -23170, -20787, -18204, -15446, -12539, -9512, -6393, -3212};

struct Region {OSSharedPtr<IOMemoryMap> keep; uintptr_t va = 0;};
bool mapRegion(uint64_t phys, uint64_t length, Region &region, IOOptionBits cache = kIOMapInhibitCache) {
    IODeviceMemory *memory = IODeviceMemory::withRange(phys, length);
    if (!memory) return false;
    region.keep = OSSharedPtr<IOMemoryMap>(memory->map(kIOMapAnywhere | cache), OSNoRetain);
    memory->release();
    region.va = region.keep ? static_cast<uintptr_t>(region.keep->getVirtualAddress()) : 0;
    return region.va != 0;
}
uint32_t rd32(uintptr_t base, uint32_t reg) {return *reinterpret_cast<volatile uint32_t *>(base + reg);}
void wr32(uintptr_t base, uint32_t reg, uint32_t value) {
    *reinterpret_cast<volatile uint32_t *>(base + reg) = value;
    __asm__ volatile("dsb sy" ::: "memory");
}
void cacheClean(const void *p, size_t len) {
    uint64_t ctr;
    __asm__ volatile("mrs %0, ctr_el0" : "=r"(ctr));
    const size_t line = 4u << ((ctr >> 16) & 15);
    for (uintptr_t a = reinterpret_cast<uintptr_t>(p) & ~(line - 1), end = reinterpret_cast<uintptr_t>(p) + len; a < end; a += line)
        __asm__ volatile("dc cvac, %0" :: "r"(a) : "memory");
    __asm__ volatile("dsb sy" ::: "memory");
}
uint32_t ioRd(void *ctx, uint32_t reg) {return rd32(reinterpret_cast<uintptr_t>(ctx), reg);}
void ioWr(void *ctx, uint32_t reg, uint32_t value) {wr32(reinterpret_cast<uintptr_t>(ctx), reg, value);}
uint32_t smmuRd(void *ctx, unsigned o) {return rd32(reinterpret_cast<uintptr_t>(ctx), o);}
void smmuWr(void *ctx, unsigned o, uint32_t v) {wr32(reinterpret_cast<uintptr_t>(ctx), o, v);}
int32_t smmuErrorCode(const char *e) {
    static const char *const names[] = {"SMMU AArch64 stage 1 unsupported", "DMA exceeds SMMU address width", "unsupported SMMU geometry",
        "conflicting stream mapping", "reserved context bank is not a bypass bank", "no free stream-match group",
        "SMMU bypass bank readback", "SMMU stream context readback", "SMMU stream mapping readback", "no free context bank", "bad window",
        "SMMU window bank readback", "SMMU window stream readback"};
    for (unsigned i = 0; i < sizeof names / sizeof *names; ++i) if (strcmp(e, names[i]) == 0) return static_cast<int32_t>(i) + 1;
    return 99;
}
uint64_t nowMs() {uint64_t ns; absolutetime_to_nanoseconds(mach_absolute_time(), &ns); return ns / 1000000;}
void fail(int step) {native_audio_stats[0] = -step;}
void stage(int step) {native_audio_stats[0] = step;}

struct Audio {
    Region gccLow, gcc1e, gcc23, gccVote, smmu, se;
    Region pins[8];
    unsigned pinCount;
    GeniI2c bus;
    uint8_t *bufferVa;
    uint64_t bufferPa;
    Region low;
    uint64_t dspAddr;   // the buffer as the DSP takes it
    ShengAdspChannel *channel;
    IOLock *lock;
    IOSimpleLock *ringLock;
    struct {uint32_t length; uint8_t data[kMessage];} ring[kRing];
    uint32_t ringHead, ringCount, token, handle;
    volatile uint32_t up;
    uint32_t toneMs, atten, mode, prm, micWant;
    ArI2s i2s;
    uint16_t deviceBits;
    // The shared memory of the service (ShengAudioAbi.h).
    OSSharedPtr<IOBufferMemoryDescriptor> control, outRing, inRing;
    ShengAudioControl *ctl;
    int16_t *out;
    volatile uint32_t want;            // ShengAudioOutputRunning / ShengAudioInputRunning as the plug-in asks
    volatile uint32_t serving;         // the service accepts requests
    bool running, statusRead, tone;
    uint64_t next, done, first;        // playback periods: next to send, returned so far, the first of this run (all absolute)
    uint64_t stopAsked;                // ms when the plug-in stopped output; the path stays up kKeepMs longer
    uint32_t inFlight;
    // Capture for the plug-in's input stream, once the microphone reached stage 8.
    int16_t *in;
    bool micReady, capturing;
    uint64_t capNext, capDone, capPos;   // capture periods asked and returned; the timeline frame the next captured sample goes to
    uint32_t capInFlight;
};
Audio gAudio;

// DSP thread: copy and wake, nothing else.
void dspState(void *, int up) {
    __atomic_store_n(&gAudio.up, up ? 1u : 0u, __ATOMIC_RELEASE);
    IOLockWakeup(gAudio.lock, &gAudio.ringCount, false);
}
void dspReceive(void *, const uint8_t *data, uint32_t length) {
    IOSimpleLockLock(gAudio.ringLock);
    if (gAudio.ringCount < kRing) {
        auto &slot = gAudio.ring[(gAudio.ringHead + gAudio.ringCount) % kRing];
        slot.length = length < kMessage ? length : kMessage;
        memcpy(slot.data, data, slot.length);
        gAudio.ringCount++;
    }
    IOSimpleLockUnlock(gAudio.ringLock);
    IOLockWakeup(gAudio.lock, &gAudio.ringCount, false);
}
// The next answer, waiting up to ms; false on timeout.
bool nextAnswer(ArAnswer &a, uint32_t ms) {
    const uint64_t end = nowMs() + ms;
    for (;;) {
        uint8_t data[kMessage];
        uint32_t length = 0;
        IOSimpleLockLock(gAudio.ringLock);
        if (gAudio.ringCount) {
            auto &slot = gAudio.ring[gAudio.ringHead];
            length = slot.length;
            memcpy(data, slot.data, length);
            gAudio.ringHead = (gAudio.ringHead + 1) % kRing;
            gAudio.ringCount--;
        }
        IOSimpleLockUnlock(gAudio.ringLock);
        if (length) {
            if (ar_parse(data, length, &a) == ArBad) continue;
            native_audio_stats[2] = native_audio_stats[2] + 1;
            native_audio_stats[12] = (int32_t)a.opcode;
            return true;
        }
        const uint64_t now = nowMs();
        if (now >= end) return false;
        uint64_t deadline;
        clock_interval_to_deadline((uint32_t)(end - now), kMillisecondScale, &deadline);
        IOLockLock(gAudio.lock);
        if (!gAudio.ringCount) IOLockSleepDeadline(gAudio.lock, &gAudio.ringCount, deadline, THREAD_UNINT);
        IOLockUnlock(gAudio.lock);
    }
}
bool send(const uint8_t *packet, uint32_t length) {return length && sheng_adsp_send(gAudio.channel, packet, length) == 0;}
// Sends an APM or PRM command and waits for its result (or, for the map, the handle); *status gets the DSP's status (-1 no answer).
// Buffer answers that arrive meanwhile belong to the running stream and go to handle() once the service runs (sheng's i537).
void handle(const ArAnswer &a);
bool command(uint32_t length, const uint8_t *packet, uint32_t opcode, int32_t *status = nullptr) {
    if (status) *status = -1;
    if (!send(packet, length)) return false;
    ArAnswer a;
    const uint64_t end = nowMs() + 2000;
    while (nowMs() < end) {
        if (!nextAnswer(a, 2000)) break;
        if (a.opcode == AR_OP_MEM_MAP_DONE && opcode == AR_OP_MEM_MAP) {gAudio.handle = a.handle; native_audio_stats[3] = (int32_t)a.handle; return true;}
        if ((a.opcode == GPR_OP_BASIC_RESULT && a.result_opcode == opcode) ||
            (a.opcode == AR_OP_PRM_REQUEST_DONE && opcode == AR_OP_PRM_REQUEST) || (a.opcode == AR_OP_PRM_RELEASE_DONE && opcode == AR_OP_PRM_RELEASE)) {
            native_audio_stats[1] = (int32_t)a.status;
            if (status) *status = (int32_t)a.status;
            return a.status == 0;
        }
        if (gAudio.serving && (a.opcode == AR_OP_WRITE_DONE || a.opcode == AR_OP_READ_DONE)) {
            handle(a);
            native_audio_passed = native_audio_passed + 1;
        }
    }
    native_audio_stats[1] = -60;
    return false;
}

// One I2C transfer to the amplifier at address, waiting by polling (20 us steps, 20 ms per stage of the timeout ladder).
bool transfer(uint8_t address, const TfaAction &act, uint16_t *value) {
    GeniI2c &g = gAudio.bus;
    g.address = address;
    const int started = act.read_length ? geni_i2c_start_read_bytes(&g, act.bytes, act.length, act.read_length)
                                        : geni_i2c_start_write_bytes(&g, act.bytes, act.length);
    if (started) return false;
    for (int stageLeft = 3; stageLeft > 0; ) {
        for (unsigned i = 0; i < 1000; ++i) {
            const int r = geni_i2c_poll(&g);
            if (r == GeniDone) {if (value) *value = tfa_wire_value(g.rx); return true;}
            if (r == GeniFailed) {native_audio_stats[13] = native_audio_stats[13] + 1; return false;}
            IODelay(20);
        }
        const int r = geni_i2c_timeout(&g);
        if (r != GeniPending) break;
        --stageLeft;
    }
    native_audio_stats[13] = native_audio_stats[13] + 1;
    return false;
}
// Runs a program on one amplifier; false on the first failed transfer.
bool runProgram(unsigned amp, const TfaStep *steps, unsigned count, TfaRun &run) {
    tfa_run_start(&run, steps, count);
    for (;;) {
        TfaAction act;
        const int k = tfa_run_next(&run, &act);
        if (k == TfaFinished) return true;
        if (k == TfaWait) {IOSleep(act.delay_ms); continue;}
        uint16_t value = 0;
        if (!transfer(tfa_address(amp), act, &value)) return false;
        tfa_run_result(&run, value);
    }
}

// The DSP's audio streams in pass-through (the bypass bank the storage and touch drivers share); sheng found the hypervisor keeps the
// translating context banks for them.
bool setupSmmu() {
    if (!mapRegion(kSmmu, kSmmuBytes, gAudio.smmu)) return false;
    struct smmu_io io = {reinterpret_cast<void *>(gAudio.smmu.va), smmuRd, smmuWr};
    int32_t result = 0;
    for (unsigned i = 0; i < sizeof kStreams / sizeof *kStreams; ++i) {
        struct smmu_plan plan = smmu_plan_bypass(&io, kStreams[i], gAudio.bufferPa, kBufferBytes);
        const char *e = plan.error ? plan.error : smmu_configure_bypass(&io, &plan, kStreams[i]);
        if (!e) result |= 1 << i;
        else if (!(result >> 8)) result |= smmuErrorCode(e) << 8;
    }
    native_audio_stats[9] = result;
    return (result & 1) != 0;
}

// The DSP takes 32-bit buffer addresses (sheng's i491 to i495) with the stream index (0x1001 & 0xf) in the upper word. The buffer is
// 64 KiB of low DRAM nothing uses after boot, mapped write-combined: on the 3a the tail of the splash region (sheng's 0xd0000000 lies in
// the UEFI DXE heap here). sheng-audio-pa moves it.
constexpr uint64_t kLowBuffer = 0xe6200000ULL;
bool setupBuffer() {
    uint64_t pa = kLowBuffer;
    PE_parse_boot_argn("sheng-audio-pa", &pa, sizeof pa);
    if ((pa & 0xfff) || pa + kBufferBytes > 0x100000000ULL || !mapRegion(pa, kBufferBytes, gAudio.low, kIOMapWriteCombineCache)) return false;
    gAudio.bufferPa = pa;
    gAudio.dspAddr = pa | 1ull << 32;
    gAudio.bufferVa = reinterpret_cast<uint8_t *>(gAudio.low.va);
    native_audio_pa = gAudio.bufferPa;
    memset(gAudio.bufferVa, 0, kBufferBytes);
    return true;
}

bool setupPins() {
    auto pin = [](unsigned gpio) -> uintptr_t {
        if (gAudio.pinCount >= 8 || !mapRegion(kTlmm + gpio * 0x1000ull, 0x1000, gAudio.pins[gAudio.pinCount])) return 0;
        return gAudio.pins[gAudio.pinCount++].va;
    };
    for (unsigned g : kI2sPins) {uintptr_t p = pin(g); if (!p) return false; wr32(p, 0, 2u << 2 | 3u << 6);}
    for (unsigned g : kI2cPins) {uintptr_t p = pin(g); if (!p) return false; wr32(p, 0, 1u << 2 | 0u << 6 | 3u);}
    uintptr_t reset[TFA_AMPS];
    for (unsigned amp = 0; amp < TFA_AMPS; ++amp) {
        reset[amp] = pin(tfa_reset_gpio(amp));
        if (!reset[amp]) return false;
        wr32(reset[amp], 4, 1u << 1);        // reset asserted (the TFA9873's RST is active high)
        wr32(reset[amp], 0, 1u << 9);        // GPIO, output enabled, 2 mA, no pull
    }
    IOSleep(1);
    for (unsigned amp = 0; amp < TFA_AMPS; ++amp) wr32(reset[amp], 4, 0);
    IOSleep(2);
    return true;
}

bool setupClocks() {
    if (!mapRegion(kGcc, 0x1000, gAudio.gccLow) || !mapRegion(kGcc + 0x1e000, 0x1000, gAudio.gcc1e) ||
        !mapRegion(kGcc + 0x23000, 0x1000, gAudio.gcc23) || !mapRegion(kGcc + 0x52000, 0x1000, gAudio.gccVote)) return false;
    wr32(gAudio.gccVote.va, kGpll0Vote & 0xfff, rd32(gAudio.gccVote.va, kGpll0Vote & 0xfff) | 1u);
    for (unsigned i = 0; i < 10000 && !(rd32(gAudio.gccLow.va, kGpll0Mode) & (1u << 31)); ++i) IODelay(10);
    wr32(gAudio.gccVote.va, kVote & 0xfff, rd32(gAudio.gccVote.va, kVote & 0xfff) | kVoteBits);
    bool ok = false;
    for (unsigned i = 0; i < 2000 && !ok; ++i) {
        ok = true;
        for (uint32_t reg : kHalts) {
            const uint32_t v = rd32((reg >> 12) == 0x1e ? gAudio.gcc1e.va : gAudio.gcc23.va, reg & 0xfff);
            if ((v & kCbcrOff) && !(v & 2u)) ok = false;
        }
        if (!ok) IODelay(10);
    }
    return ok;   // never touch the engine without running clocks
}

// The DFS level that runs the engine from TCXO undivided (source 0, no M/N, half-integer divider 0 or 1).
unsigned tcxoLevel(uintptr_t rcg) {
    for (unsigned l = 0; l < kDfsLevels; ++l) {
        const uint32_t cfg = rd32(rcg, kRcgPerfCfg + 4 * l);
        if (((cfg >> 8) & 7) == 0 && ((cfg >> 12) & 3) == 0 && (cfg & 0x1f) <= 1) return l;
    }
    return 0xff;
}

// Brings the engine to I2C at 400 kHz if the boot loader left it unconfigured, then hands it to the transfer core.
bool setupBus() {
    if (!mapRegion(kSe, 0x4000, gAudio.se)) return false;
    const uintptr_t se = gAudio.se.va;
    const unsigned level = tcxoLevel(gAudio.gcc1e.va + (kRcg & 0xfff));
    const uint32_t proto = (rd32(se, GENI_FW_REVISION) >> 8) & 0xff;
    native_audio_stats[10] = (int32_t)(proto << 24 | (rd32(se, GENI_CLK_CFG) & 0xffffff));
    if (proto != GENI_PROTO_I2C) return false;
    if (!(rd32(se, GENI_CLK_CFG) & 1u) || !rd32(se, GENI_SCL_COUNTERS)) {
        if (level == 0xff) return false;
        wr32(se, 0x28, rd32(se, 0x28) | 0x7fu);             // clock gating controls on
        wr32(se, 0xe30, rd32(se, 0xe30) | 0xfu);            // DMA clock gating controls on
        wr32(se, 0x24, 0x7fu);                              // default output control
        wr32(se, 0x20, 1u);                                 // force default
        wr32(se, 0xe18, 0);                                 // no GSI events
        wr32(se, 0x7c, level);
        wr32(se, GENI_CLK_CFG, kClkCfg);
        wr32(se, GENI_SCL_COUNTERS, kSclCounters);
    }
    const GeniIo io = {ioRd, ioWr, reinterpret_cast<void *>(se)};
    uint32_t config = 0;
    if (geni_i2c_prepare(&gAudio.bus, &io, TFA_ADDRESS_P, &config)) return false;
    native_audio_stats[10] = (int32_t)config;
    return true;
}

bool setupAmps() {
    TfaStep steps[TFA_MAX_STEPS];
    TfaRun run;
    int32_t found = 0;
    for (unsigned amp = 0; amp < TFA_AMPS; ++amp) {
        if (!runProgram(amp, steps, tfa_identify_program(steps), run)) continue;
        const uint16_t revision = run.reads[0];
        native_audio_amps[amp][0] = revision;
        if (!TFA_REVISIONS_KNOWN(revision)) continue;
        const unsigned n = tfa_init_program(steps, amp, revision);
        if (n > TFA_MAX_STEPS || !runProgram(amp, steps, n, run)) continue;
        found |= 1 << amp;
    }
    native_audio_stats[6] = found;
    return found != 0;
}

void readAmpStatus() {
    TfaStep steps[TFA_MAX_STEPS];
    TfaRun run;
    int32_t clocked = 0, errors = 0;
    for (unsigned amp = 0; amp < TFA_AMPS; ++amp) {
        if (!(native_audio_stats[6] & (1 << amp))) continue;
        if (!runProgram(amp, steps, tfa_status_program(steps), run)) continue;
        native_audio_amps[amp][2] = run.reads[0];
        native_audio_amps[amp][3] = run.reads[1];
        if (run.reads[0] & TFA_STATUS0_CLKS) clocked |= 1 << amp;
        if (TFA_STATUS0_FAULT(run.reads[0])) errors |= 1 << amp;
    }
    native_audio_stats[7] = clocked;
    native_audio_stats[8] = errors;
}

void powerUpAmps() {
    TfaStep steps[TFA_MAX_STEPS];
    TfaRun run;
    for (unsigned amp = 0; amp < TFA_AMPS; ++amp) {
        if (!(native_audio_stats[6] & (1 << amp))) continue;
        if (runProgram(amp, steps, tfa_power_up_program(steps), run)) native_audio_amps[amp][1] = run.failed;
        else native_audio_amps[amp][1] = 0xff;
    }
}

void powerDownAmps() {
    TfaStep steps[TFA_MAX_STEPS];
    TfaRun run;
    for (unsigned amp = 0; amp < TFA_AMPS; ++amp) {
        if (!(native_audio_stats[6] & (1 << amp))) continue;
        runProgram(amp, steps, tfa_power_down_program(steps), run);
    }
}

bool waitUp(uint32_t ms) {
    const uint64_t end = nowMs() + ms;
    while (!__atomic_load_n(&gAudio.up, __ATOMIC_ACQUIRE)) {
        if (nowMs() >= end) return false;
        IOSleep(10);
    }
    return true;
}

// Asks until the framework manager says ready (status 1), 100 ms apart, at most 100 times.
bool dspReady() {
    uint8_t b[GPR_MAX_PACKET];
    for (unsigned query = 0; query < 100; ++query) {
        const GprPacket q = {.dst_domain = GPR_DOMAIN_ADSP, .src_domain = GPR_DOMAIN_APPS, .src_port = kPort, .dst_port = GPR_PORT_APM,
                             .token = ++gAudio.token, .opcode = GPR_OP_APM_GET_SPF_STATE};
        if (send(b, gpr_encode(b, sizeof b, &q, nullptr, 0))) {
            const uint64_t end = nowMs() + 1000;
            ArAnswer a;
            while (nowMs() < end && nextAnswer(a, 1000)) {
                if (a.opcode == GPR_OP_APM_SPF_STATE) {native_audio_stats[1] = (int32_t)a.status; if (a.status == 1) return true; break;}
                if (a.opcode == GPR_OP_BASIC_RESULT && a.result_opcode == GPR_OP_APM_GET_SPF_STATE) {native_audio_stats[1] = (int32_t)a.status; break;}
            }
        }
        IOSleep(100);
    }
    fail(51);
    return false;
}

// The PRM votes the vendor's machine driver makes for MI2S (sheng's PRM commands go out from the PRM's own port). Not fatal: the I2S
// endpoint may take its clock itself, as sheng's secondary MI2S did.
void prmVotes() {
    uint8_t b[GPR_MAX_PACKET];
    int32_t core = 0xff, clock = 0xff, status;
    if (gAudio.prm & 1) {
        command(ar_prm_vote(b, sizeof b, AR_PRM_IID, ++gAudio.token, AR_HW_MACRO, 1), b, AR_OP_PRM_REQUEST, &status);
        core = status < 0 ? 0xff : status & 0xff;
    }
    if (gAudio.prm & 2) {
        const uint32_t hz = kRate * 2 * gAudio.deviceBits;
        command(ar_prm_clock_attr(b, sizeof b, AR_PRM_IID, ++gAudio.token, kPrmClkPriMi2sIbit + 2 * gAudio.i2s.intf, hz, kPrmClkAttrCoupleNo),
                b, AR_OP_PRM_REQUEST, &status);
        clock = status < 0 ? 0xff : status & 0xff;
    }
    native_audio_stats[11] = core | clock << 8;
}

bool setupShared() {
    const IOOptionBits shared = static_cast<IOOptionBits>(kIODirectionInOut) | static_cast<IOOptionBits>(kIOMemoryKernelUserShared);
    gAudio.control = OSSharedPtr<IOBufferMemoryDescriptor>(IOBufferMemoryDescriptor::inTaskWithOptions(kernel_task, shared, 4096, 4096), OSNoRetain);
    gAudio.outRing = OSSharedPtr<IOBufferMemoryDescriptor>(IOBufferMemoryDescriptor::inTaskWithOptions(kernel_task, shared,
        kRingFrames * SHENG_AUDIO_OUT_CHANNELS * 2, 4096), OSNoRetain);
    gAudio.inRing = OSSharedPtr<IOBufferMemoryDescriptor>(IOBufferMemoryDescriptor::inTaskWithOptions(kernel_task, shared,
        kRingFrames * SHENG_AUDIO_IN_CHANNELS * 2, 4096), OSNoRetain);
    if (!gAudio.control || !gAudio.outRing || !gAudio.inRing) return false;
    gAudio.ctl = static_cast<ShengAudioControl *>(gAudio.control->getBytesNoCopy());
    gAudio.out = static_cast<int16_t *>(gAudio.outRing->getBytesNoCopy());
    gAudio.in = static_cast<int16_t *>(gAudio.inRing->getBytesNoCopy());
    memset(gAudio.ctl, 0, 4096);
    memset(gAudio.out, 0, kRingFrames * SHENG_AUDIO_OUT_CHANNELS * 2);
    memset(gAudio.inRing->getBytesNoCopy(), 0, kRingFrames * SHENG_AUDIO_IN_CHANNELS * 2);
    mach_timebase_info_data_t tb;
    clock_timebase_info(&tb);
    ShengAudioControl *c = gAudio.ctl;
    c->rate = kRate; c->out_channels = SHENG_AUDIO_OUT_CHANNELS; c->in_channels = SHENG_AUDIO_IN_CHANNELS;
    c->period_frames = kPeriodFrames; c->ring_frames = kRingFrames; c->lead_frames = kLeadFrames;
    // 128-bit: with the 19.2 MHz timer (timebase 625/12) 1e9 * 12 << 32 overflows 64 bits (sheng's i535).
    c->ticks_per_frame_q32 = (uint64_t)(((__uint128_t)1000000000ULL * tb.denom << 32) / ((uint64_t)kRate * tb.numer));
    c->out_anchor_host = mach_absolute_time();
    c->in_anchor_host = c->out_anchor_host;
    c->version = SHENG_AUDIO_VERSION;
    __atomic_store_n(&c->magic, SHENG_AUDIO_MAGIC, __ATOMIC_RELEASE);
    return true;
}

// [audio-service] peak=: the largest sample magnitude the plug-in's playback (low half) and capture (high half) carried; the boot's test
// tone is not counted.
extern "C" volatile int32_t native_audio_service[8];
void notePeak(const int16_t *p, uint32_t count, unsigned shift) {
    int32_t peak = 0;
    for (uint32_t i = 0; i < count; ++i) {const int32_t v = p[i] < 0 ? -p[i] : p[i]; if (v > peak) peak = v;}
    if (peak > 0x7fff) peak = 0x7fff;
    const uint32_t now = (uint32_t)native_audio_service[7], mine = (now >> shift) & 0xffffu;
    if ((uint32_t)peak > mine) native_audio_service[7] = (int32_t)((now & ~(0xffffu << shift)) | (uint32_t)peak << shift);
}

// The frame of the shared timeline that plays now.
uint64_t framesNow() {
    const uint64_t now = mach_absolute_time(), anchor = gAudio.ctl->out_anchor_host;
    return now > anchor ? (uint64_t)(((__uint128_t)(now - anchor) << 32) / gAudio.ctl->ticks_per_frame_q32) : 0;
}

// [audio-sync]: playback resyncs, the least and most drift of playback from its steady lead (frames), capture resyncs, the
// steady lead. The DSP answers a buffer once it has copied it into its own queue, so each run takes the steady distance once the queue
// has filled; when the DSP's clock and the host's drift three periods from it, the next period is set back to it (one click).
extern "C" {volatile int32_t native_audio_sync[5] = {0, 0x7fffffff, -0x7fffffff, 0, 0};}
constexpr int64_t kDriftLimit = 3 * kPeriodFrames;
int64_t gBaseLead;
void syncPlayback() {
    const uint64_t now = framesNow(), played = gAudio.done - gAudio.first;
    const int64_t lead = (int64_t)(gAudio.next * kPeriodFrames) - (int64_t)now;
    if (played < 2 * kInFlight) return;
    if (played == 2 * kInFlight) {
        gBaseLead = lead;
        native_audio_sync[4] = (int32_t)lead;
        const int64_t want = lead + 2 * (int64_t)kPeriodFrames;
        gAudio.ctl->lead_frames = (uint32_t)(want < (int64_t)kLeadFrames ? kLeadFrames : want > 24000 ? 24000 : want);
        return;
    }
    const int64_t drift = lead - gBaseLead;
    if (drift < native_audio_sync[1]) native_audio_sync[1] = (int32_t)drift;
    if (drift > native_audio_sync[2]) native_audio_sync[2] = (int32_t)drift;
    if (drift > kDriftLimit || drift < -kDriftLimit) {
        gAudio.next = (uint64_t)((int64_t)now + gBaseLead + kPeriodFrames / 2) / kPeriodFrames;
        native_audio_sync[0] = native_audio_sync[0] + 1;
    }
}

// Sends period n (timeline frames n * kPeriodFrames on) from the playback ring through DSP slot n % kSlots. Outside the test tone the
// frames taken are cleared, so a frame the plug-in did not write in time plays as silence (sheng's i531).
bool submit(uint64_t n) {
    uint8_t b[GPR_MAX_PACKET];
    const uint32_t slot = (uint32_t)(n % kSlots);
    int16_t *dst = reinterpret_cast<int16_t *>(gAudio.bufferVa + slot * kPeriodBytes);
    const uint32_t at = (uint32_t)((n * kPeriodFrames) % kRingFrames), head = kRingFrames - at < kPeriodFrames ? kRingFrames - at : kPeriodFrames;
    memcpy(dst, gAudio.out + at * kChannels, head * kChannels * 2);
    if (head < kPeriodFrames) memcpy(dst + head * kChannels, gAudio.out, (kPeriodFrames - head) * kChannels * 2);
    if (!gAudio.tone) {
        memset(gAudio.out + at * kChannels, 0, head * kChannels * 2);
        if (head < kPeriodFrames) memset(gAudio.out, 0, (kPeriodFrames - head) * kChannels * 2);
        uint64_t recent = 0;
        nanoseconds_to_absolutetime(100000000ULL, &recent);
        if ((gAudio.want & ShengAudioOutputRunning) && mach_absolute_time() - gAudio.ctl->out_written_host < recent &&
            gAudio.ctl->out_written < (n + 1) * kPeriodFrames) {
            gAudio.ctl->out_late = gAudio.ctl->out_late + 1;
        } else if ((gAudio.want & ShengAudioOutputRunning) && gAudio.ctl->out_written >= (n + 1) * kPeriodFrames) {
            uint32_t run = 0, longest = 0, loud = 0;
            for (uint32_t i = 0; i < kPeriodFrames; ++i) {
                const bool quiet = dst[2 * i] == 0 && dst[2 * i + 1] == 0;
                run = quiet ? run + 1 : 0;
                if (run > longest) longest = run;
                if (!quiet) loud++;
            }
            if (longest >= 64 && loud) gAudio.ctl->out_holes = gAudio.ctl->out_holes + 1;
        }
        notePeak(dst, kPeriodFrames * kChannels, 0);
    }
    cacheClean(dst, kPeriodBytes);
    if (!send(b, ar_write(b, sizeof b, kPort, (uint32_t)n, gAudio.dspAddr + slot * kPeriodBytes, gAudio.handle, kPeriodBytes))) return false;
    native_audio_stats[4] = native_audio_stats[4] + 1;
    gAudio.inFlight++;
    return true;
}

bool graphs(uint32_t opcode) {
    uint8_t b[GPR_MAX_PACKET];
    const uint32_t first = opcode == AR_OP_GRAPH_STOP ? AR_SG_PLAY : AR_SG_SPEAKER, second = first == AR_SG_PLAY ? AR_SG_SPEAKER : AR_SG_PLAY;
    const bool a = command(ar_graph_command(b, sizeof b, kPort, ++gAudio.token, opcode, first), b, opcode);
    const bool c = command(ar_graph_command(b, sizeof b, kPort, ++gAudio.token, opcode, second), b, opcode);
    return a && c;
}

void startOutput() {
    if (gAudio.running) return;
    if (!graphs(AR_OP_GRAPH_PREPARE) || !graphs(AR_OP_GRAPH_START)) return;
    ShengAudioControl *c = gAudio.ctl;
    if (!gAudio.tone) memset(gAudio.out, 0, kRingFrames * kChannels * 2);
    gAudio.next = gAudio.done = gAudio.first = (framesNow() + kPeriodFrames - 1) / kPeriodFrames;
    c->out_frames = 0;
    gAudio.inFlight = 0;
    gAudio.statusRead = false;
    gAudio.running = true;
    while (gAudio.inFlight < kInFlight && submit(gAudio.next)) gAudio.next++;
    // The bit clock runs now: the amplifiers' power-up waits for it.
    powerUpAmps();
    __atomic_or_fetch(&c->state, ShengAudioOutputRunning, __ATOMIC_ACQ_REL);
    native_audio_stats[14] = native_audio_stats[14] + 1;
}

void stopOutput() {
    if (!gAudio.running) return;
    gAudio.running = false;
    powerDownAmps();
    graphs(AR_OP_GRAPH_STOP);
    __atomic_and_fetch(&gAudio.ctl->state, ~ShengAudioOutputRunning, __ATOMIC_ACQ_REL);
    native_audio_stats[14] = native_audio_stats[14] + (1 << 16);
}

// ---- the microphone (asteroids-mic=<stage>, with sheng-audio; T41) ----
// The 3a's main microphone is AMIC1 of the WCD9378 codec (vendor mixer_paths "handset-mic": amic1 on ADC1, MIC BIAS1 by the card's
// routing), low power ADC mode, gain 6; ADC1 leaves the codec's TX SoundWire slave on data port 1 and enters master port 1 channel 1
// (SWR_MIC0) of the VA macro's SoundWire master (0x33b0000, v1.7), then TX decimator 0 and codec DMA TX3 (resourcemanager: handset mic
// on CODEC_DMA-LPAIF_RXTX-TX-3, sheng's capture graphs). The codec's registers are the 32-bit paged SDCA space of the July 2026 LKML
// WCD9378 TX driver (wcd9378.c v2, from Qualcomm's GPL driver); its built-in sequencers do the analog power-up. Stages, each one
// including the ones before:
//   1  the codec's 1.8 V rails (ldob7 io/rxtx, ldob8 buck) on active TCS 2 of the apps RSC, then its reset (gpio162) pulsed low
//   2  PRM: LPASS macro and digital codec votes, the VA and TX core clocks at 19.2 MHz
//   3  LPI pins of the VA SoundWire bus (gpio0 clock, 1/2/14 data), VA macro frame sync and SoundWire clock, TX macro clock
//   4  the SoundWire master (vendor swrm_master_init for v1.6+), enumeration: the codec's device number and id
//   5  the codec: SCP clock and interrupt setup, vendor interrupt types, io_init, the SDCA function class load, part id
//   6  the capture path: ports in both banks, micbias 1, SmartMIC0 sequencer up (ACT_PS 0), TX decimator 0
//   7  the DSP's capture graphs and two seconds of capture ([mic capture])
//   8  capture serves the plug-in's input stream
// asteroids-mic-port = master sample interval | offset1 << 8 | lane << 16 (default 1 | 1 << 8: the vendor's first TX use case for ADC1,
// sheng's interval) moves the port timing; asteroids-mic-mux=1 sets the VA macro's clock source switch (kVaMuxSel).
extern "C" {volatile int32_t native_mic_stats[16];}
// [0] stage (positive progress x10, negative the failing step), [1] PRM: macro | dcodec << 8 | VA clock << 16 | TX clock << 24 (0xff no
// answer), [2] rails (bit 0 ldob7 found, 1 ldob8 found, 2 voted), [3] master version, [4] COMP_STATUS, [5] slave status, [6]/[7] the
// codec's device id low/high word, [8] its part id, [9] SmartMIC0 ACT_PS after the power-up, [10] TX macro MCLK control, [11] VA macro
// MCLK control, [12] master interrupt status, [13] COMP_PARAMS, [14] reads that timed out, [15] ms the bring-up took
extern "C" {volatile uint32_t native_mic_regs[16];}
// tx 0x0/0x4/0x100/0x400, va 0x0/0x4/0x80 (from stage 6; up to stage 5 the first seven are tx 0x0/0x4/0x8, the VA clock source switch,
// va 0x0/0x4/0x8, read when the master is up or fails), swr 0x4/0x101c/0x1044/0x104c/0x200/0x14/0x1124, LPI gpio0 control, slew
extern "C" {volatile int32_t native_mic_capture[8];}
// The capture's length (asteroids-mic-capture-s, default 2, at most 30) and its peak in each second, with the uptime (ms) it started at:
// a voice at a known moment shows in its second (T47-mic7: two seconds at boot peaked at 34 of 32767).
extern "C" {volatile int32_t native_mic_peaks[30];volatile uint32_t native_mic_capture_at, native_mic_capture_s;}
// T42 diagnostics for a master whose link never comes up: [0] the codec reset pin's IN_OUT register after release (bit 0 the level read
// back), [1]/[3] LDO B7/B8 (the codec's rails, PM7550 at SPMI SID 1, peripherals 0x46/0x47): APID | STATUS1 << 12 | EN_CTL << 20, read
// only with asteroids-mic-pmic=1 (T42: no APID for 0x146, and 0x147 is TYPE 9 SUBTYPE 2, not an LDO; recovery's regmap cannot read SID 1
// 0x42xx-0x46xx either: the rails are the RPMh's alone, ldob7 already on for the eUSB2 repeater), [2]/[4] VSET low | VSET high << 8 | MODE << 16 | read status << 24 (0xff not read), [5]/[6]/[7] the
// master's COMP_STATUS 1 ms, 100 ms and 1 s after it was enabled.
extern "C" {volatile uint32_t native_mic_extra[8];}
// [0] stage, [1] last DSP status, [2] periods asked, [3] returned, [4] bytes, [5] peak, [6] mean absolute value, [7] non-zero samples
// The VA macro's clock source switch (DT qcom,va-island-mode-muxsel; va-clk-mux-select = 1, use-clk-id 3 = VA core clock): the vendor's
// clock manager sets it to 1 once the VA core clock runs. asteroids-mic-mux=1 does the same (T41: the master's link never came up with
// it left at 0).
constexpr uint64_t kVaMuxSel = 0x03420000ULL;
constexpr uint64_t kLpi = 0x03440000ULL, kLpiSlew = 0x034da000ULL, kTxMacro = 0x03220000ULL, kVaMacro = 0x033f0000ULL, kSwr = 0x033b0000ULL;
constexpr unsigned kWcdResetPin = 162;
constexpr uint64_t kCmdDb = 0x81c60000ULL, kRsc = 0x17a20000ULL;
constexpr uint32_t kCmdDbBytes = 0x20000, kTcsOffset = 0xd00;
constexpr uint32_t kCapBytes = kPeriodFrames * 2, kCapOffset = 32 * 1024, kCapSlots = 8;
const uint32_t kMicUp[2] = {AR_SG_MIC, AR_SG_CAPTURE}, kMicDown[2] = {AR_SG_CAPTURE, AR_SG_MIC};
// WCD9378 registers (wcd9378.h of the LKML driver).
constexpr uint32_t W_ANA_BIAS = 0x40180001, W_ANA_TX_CH1 = 0x4018000e, W_ANA_TX_CH2 = 0x4018000f, W_BIAS_VBG_FINE = 0x40180029,
    W_MBHC_SPARE_1 = 0x40180058, W_TXFE_DIV = 0x4018007b, W_MICB1_TEST2 = 0x4018006c, W_MICB2_TEST2 = 0x4018006f, W_MICB3_TEST2 = 0x40180072,
    W_SLEEP_CTL = 0x40180103, W_TX_CH12_MUX = 0x4018012e, W_TX_CLK_CTL = 0x40180417, W_PDM_WD0 = 0x40180465, W_PDM_WD1 = 0x40180466,
    W_EFUSE16 = 0x401804c0, W_SYS_USAGE = 0x40180501, W_SM0_MB_SEL = 0x401805b0, W_SM1_MB_SEL = 0x401805b1, W_SM2_MB_SEL = 0x401805b2,
    W_PART_ID0 = 0x40100068, W_PART_ID1 = 0x40100069, W_AMP_ACT = 0x40880008, W_AMP_STAT = 0x40880000, W_JACK_ACT = 0x40c80008,
    W_JACK_STAT = 0x40c80000, W_JACK_GRP = 0x40c00008, W_SM0_MICB = 0x41000098, W_SM0_USAGE = 0x410000a0, W_SM0_REQ_PS = 0x41000108,
    W_SM0_ACT_PS = 0x41080100;
constexpr uint32_t wSmAct(unsigned n) {return 0x41080008 + 0x400000 * n;}
constexpr uint32_t wSmStat(unsigned n) {return 0x41080000 + 0x400000 * n;}
void micStage(int s) {native_mic_stats[0] = s;}
void micFail(int s) {native_mic_stats[0] = -s;}

// An RPMh regulator on through active TCS 2 (sheng's micRail; the touch probe uses the same TCS earlier in the boot).
bool micRails() {
    uint8_t *db = static_cast<uint8_t *>(IOMallocData(kCmdDbBytes));
    Region dbw, rsc;
    if (!db) return false;
    bool ok = mapRegion(kCmdDb, kCmdDbBytes, dbw) && mapRegion(kRsc, 0x10000, rsc);
    QcomRpmhResource res[2];
    if (ok) {
        for (uint32_t i = 0; i < kCmdDbBytes; i += 4) {const uint32_t w = rd32(dbw.va, i); memcpy(db + i, &w, 4);}
        if (!qcom_cmd_db_find(db, kCmdDbBytes, "ldob7", &res[0])) native_mic_stats[2] = native_mic_stats[2] | 1;
        if (!qcom_cmd_db_find(db, kCmdDbBytes, "ldob8", &res[1])) native_mic_stats[2] = native_mic_stats[2] | 2;
        ok = (native_mic_stats[2] & 3) == 3;
    }
    IOFreeData(db, kCmdDbBytes);
    if (!ok) return false;
    const QcomRscLayout layout = qcom_rsc_layout(rd32(rsc.va, 0));
    auto vote = [&](uint32_t address, uint32_t value) {
        QcomRpmhWrite writes[16];
        const unsigned n = qcom_rpmh_active_request(&layout, 2, &address, &value, 1, writes, 16);
        for (unsigned i = 0; i < n; ++i) wr32(rsc.va, kTcsOffset + writes[i].offset, writes[i].value);
        IOSleep(2);
        const unsigned m = qcom_rpmh_finish(&layout, 2, writes, 16);
        for (unsigned i = 0; i < m; ++i) wr32(rsc.va, kTcsOffset + writes[i].offset, writes[i].value);
    };
    for (auto &r : res) {
        vote(r.address + 8, 7);       // high power mode
        vote(r.address, 1800);        // mV
        vote(r.address + 4, 1);       // enable
    }
    native_mic_stats[2] = native_mic_stats[2] | 4;
    return true;
}

// The codec's two LDOs as the PMIC sees them (asteroids-mic-pmic=1): read through the SPMI arbiter's observer channel, as ButtonProbe
// reads the power key. Off by default: an observer read of a peripheral another EE owns may end in a synchronous external abort.
constexpr uint64_t kSpmiCore = 0x0c400000ULL, kSpmiObsrvr = 0x0c440000ULL, kSpmiCnfg = 0x0c42d000ULL;
void micPmic() {
    native_mic_extra[2] = 0xffu << 24;
    native_mic_extra[4] = 0xffu << 24;
    uint32_t want = 0;
    PE_parse_boot_argn("asteroids-mic-pmic", &want, sizeof want);
    if (want != 1) return;
    Region core, cnfg, obs;
    if (!mapRegion(kSpmiCore, 0x3000, core) || !mapRegion(kSpmiCnfg, 0x4000, cnfg)) return;
    br_arbiter arb = {reinterpret_cast<volatile uint32_t *>(core.va), reinterpret_cast<volatile uint32_t *>(cnfg.va), 0};
    const uint32_t version = arb.core[0], apids = arb.core[BR_FEATURES / 4] & BR_FEATURES_APIDS_MASK;
    if (version < BR_VERSION_V7_MIN || version >= BR_VERSION_V8_MIN || !apids || apids > 1024) return;
    if (!mapRegion(kSpmiObsrvr, ((uint64_t)BR_OBS_APID_STRIDE * apids + 0xfff) & ~0xfffull, obs)) return;
    static const uint32_t ppids[2] = {0x146, 0x147};
    for (unsigned k = 0; k < 2; ++k) {
        int apid = -1;
        for (uint32_t i = 0; i < apids; ++i) {
            const uint32_t map = arb.core[(BR_APID_MAP + 4 * i) / 4];
            if (map && ((map >> 8) & 0xfff) == ppids[k]) {apid = (int)i; break;}
        }
        if (apid < 0) continue;
        uint8_t status = 0, en = 0, lb = 0, ub = 0, mode = 0;
        volatile uint32_t *o = reinterpret_cast<volatile uint32_t *>(obs.va);
        int rc = br_read8(o, &arb, apid, 0x08, &status);
        if (!rc) rc = br_read8(o, &arb, apid, 0x46, &en);
        if (!rc) rc = br_read8(o, &arb, apid, 0x40, &lb);
        if (!rc) rc = br_read8(o, &arb, apid, 0x41, &ub);
        if (!rc) rc = br_read8(o, &arb, apid, 0x45, &mode);
        native_mic_extra[1 + 2 * k] = (uint32_t)apid | (uint32_t)status << 12 | (uint32_t)en << 20;
        native_mic_extra[2 + 2 * k] = lb | (uint32_t)ub << 8 | (uint32_t)mode << 16 | (uint32_t)(rc & 0xff) << 24;
    }
}

// PRM: [1] macro | dcodec << 8 | VA core clock << 16 | TX core clock << 24, each the DSP's status (0xff no answer).
bool micPrm() {
    uint8_t b[GPR_MAX_PACKET];
    int32_t s;
    auto run = [&](uint32_t n) {command(n, b, AR_OP_PRM_REQUEST, &s); return s < 0 ? 0xffu : (uint32_t)s & 0xffu;};
    uint32_t r = run(ar_prm_vote(b, sizeof b, AR_PRM_IID, ++gAudio.token, AR_HW_MACRO, 1));
    r |= run(ar_prm_vote(b, sizeof b, AR_PRM_IID, ++gAudio.token, AR_HW_DCODEC, 1)) << 8;
    r |= run(ar_prm_clock_attr(b, sizeof b, AR_PRM_IID, ++gAudio.token, AR_CLK_VA_CORE_MCLK, 19200000, 1)) << 16;
    r |= run(ar_prm_clock_attr(b, sizeof b, AR_PRM_IID, ++gAudio.token, AR_CLK_TX_CORE_MCLK, 19200000, 1)) << 24;
    native_mic_stats[1] = (int32_t)r;
    return (r & 0xffffu) == 0;   // register access needs the two votes
}

// The SoundWire master's command FIFO: the apps (CPU1) FIFO from v1.6 on, the shared one before (swr-mstr-registers.h).
struct Swr {Region r; uint8_t id, dev, page1, page2; uint32_t wr, rd, status, fifo;};
Swr gSwr;
uint32_t swrPack(uint8_t data, uint8_t dev, uint8_t id, uint16_t reg) {return reg | (uint32_t)id << 16 | (uint32_t)dev << 20 | (uint32_t)data << 24;}
uint8_t nextId() {gSwr.id = gSwr.id < 14 ? gSwr.id + 1 : 0; return gSwr.id;}
bool swrWrite(uint8_t dev, uint16_t reg, uint8_t value) {
    for (unsigned i = 0; i < 100 && ((rd32(gSwr.r.va, gSwr.status) >> 8) & 0x1f) >= 8; ++i) IODelay(50);
    wr32(gSwr.r.va, gSwr.wr, swrPack(value, dev, nextId(), reg));
    IODelay(50);
    return true;
}
bool swrRead(uint8_t dev, uint16_t reg, uint8_t *value) {
    const uint8_t id = nextId();
    wr32(gSwr.r.va, gSwr.rd, swrPack(1, dev, id, reg));
    for (unsigned i = 0; i < 200; ++i) {
        if ((rd32(gSwr.r.va, gSwr.status) >> 16) & 0x1f) {
            const uint32_t v = rd32(gSwr.r.va, gSwr.fifo);
            if (((v >> 8) & 0xf) == id) {*value = (uint8_t)v; return true;}
            wr32(gSwr.r.va, 0x308, 1);   // SWRM_CMD_FIFO_CMD: flush, give up on a stale answer
            break;
        }
        IODelay(50);
    }
    native_mic_stats[14] = native_mic_stats[14] + 1;
    return false;
}
// A codec register: at 0x8000 and above through SCP paging (Page1 0x48 = bits 22:15, Page2 0x49 = bits 30:23), kept while unchanged.
uint16_t wcdPage(uint32_t reg) {
    if (reg < 0x8000) return (uint16_t)reg;
    const uint8_t p1 = (uint8_t)(reg >> 15), p2 = (uint8_t)(reg >> 23);
    if (p1 != gSwr.page1) {swrWrite(gSwr.dev, 0x48, p1); gSwr.page1 = p1;}
    if (p2 != gSwr.page2) {swrWrite(gSwr.dev, 0x49, p2); gSwr.page2 = p2;}
    return (uint16_t)((reg & 0x7fff) | 0x8000);
}
bool wcdWrite(uint32_t reg, uint8_t value) {return swrWrite(gSwr.dev, wcdPage(reg), value);}
bool wcdRead(uint32_t reg, uint8_t *value) {return swrRead(gSwr.dev, wcdPage(reg), value);}
bool wcdUpdate(uint32_t reg, uint8_t mask, uint8_t value) {
    uint8_t v = 0;
    return wcdRead(reg, &v) && wcdWrite(reg, (uint8_t)((v & ~mask) | (value & mask)));
}

bool micPins() {
    Region lpi, slew;
    if (!mapRegion(kLpi, 0xf000, lpi) || !mapRegion(kLpiSlew, 0x1000, slew)) return false;
    // DT tx_swr_*_active: function 1, 2 mA, slew 1; the clock without pull, the three data lines bus hold (the sleep states pull down).
    // Slew offsets from qcom,lpi-slew-offset-tbl.
    static const struct {unsigned pin, slew; uint32_t pull;} pins[] = {{0, 0, 0}, {1, 2, 2}, {2, 4, 2}, {14, 6, 2}};
    uint32_t sv = rd32(slew.va, 0);
    for (auto &p : pins) {
        wr32(lpi.va, p.pin * 0x1000, 1u << 2 | p.pull);
        sv = (sv & ~(3u << p.slew)) | 1u << p.slew;
    }
    wr32(slew.va, 0, sv);
    native_mic_regs[14] = rd32(lpi.va, 0);
    native_mic_regs[15] = rd32(slew.va, 0);
    return true;
}

// The VA macro's frame sync generator (the DT's qcom,fs-gen-sequence: 0x3000 |= 1, 0x3004 |= 3, 0x3004 = 1, 0x3080 |= 2 at the VA macro)
// and its SoundWire clock with a reset pulse; the TX macro's clock for decimator 0.
Region gTx, gVa, gVaMuxMap;
uintptr_t gVaMux;
bool micMacros() {
    if (!mapRegion(kVaMacro, 0x1000, gVa) || !mapRegion(kTxMacro, 0x1000, gTx)) return false;
    const uintptr_t va = gVa.va, tx = gTx.va;
    uint32_t mux = 0;
    PE_parse_boot_argn("asteroids-mic-mux", &mux, sizeof mux);
    if (mapRegion(kVaMuxSel, 0x1000, gVaMuxMap)) {
        if (mux == 1) wr32(gVaMuxMap.va, 0, 1);
        gVaMux = gVaMuxMap.va;
    }
    wr32(va, 0x0, rd32(va, 0x0) | 1);
    wr32(va, 0x4, rd32(va, 0x4) | 3);
    wr32(va, 0x4, (rd32(va, 0x4) & ~2u) | 1);
    wr32(va, 0x80, rd32(va, 0x80) | 2);
    for (uint32_t r = 0xd0; r <= 0xd8; r += 4) wr32(va, r, (rd32(va, r) & ~0xeeu) | 0xccu);   // SWR mic clock dividers 1 (sheng)
    wr32(va, 0x8, rd32(va, 0x8) | 2);
    wr32(va, 0x8, rd32(va, 0x8) | 1);
    wr32(va, 0x8, rd32(va, 0x8) & ~2u);
    wr32(tx, 0x0, rd32(tx, 0x0) | 1);
    wr32(tx, 0x4, rd32(tx, 0x4) | 1);
    native_mic_stats[10] = (int32_t)rd32(tx, 0x0);
    native_mic_stats[11] = (int32_t)rd32(va, 0x0);
    return true;
}

// The master as mainline's qcom_swrm_init starts a v1.7 one: 50 rows x 16 columns, auto enumeration, 31 pings, the link manager and
// bus clock for the CPU, 7 command retries, pulse interrupts, link up (COMP_STATUS bit 0, the frame generator), then the codec found among the attached devices (manufacturer 0x0217, part 0x0110, as the driver's id table).
bool micMaster() {
    if (!mapRegion(kSwr, 0x10000, gSwr.r)) return false;
    const uintptr_t m = gSwr.r.va;
    const uint32_t version = rd32(m, 0x0);
    native_mic_stats[3] = (int32_t)version;
    native_mic_stats[13] = (int32_t)rd32(m, 0x100);
    if ((version >> 24) != 1) return false;   // the v2 layout (sheng) is not this one
    const bool cpu1 = version >= 0x01060000;
    gSwr.wr = cpu1 ? 0x31c : 0x300; gSwr.rd = cpu1 ? 0x320 : 0x304; gSwr.status = cpu1 ? 0x328 : 0x30c; gSwr.fifo = cpu1 ? 0x334 : 0x318;
    gSwr.page1 = gSwr.page2 = 0xff;
    // Two soft resets, then mainline's qcom_swrm_init (drivers/soundwire/qcom.c) step for step. A v1.7 master has a link manager per
    // execution environment: the CPU (EE 1) must take it (LINK_MANAGER_EE 0x018 = 1) and start its own bus clock (MCP_BUS_CTRL
    // CLK_START << EE = 4). T41-T46 wrote 2 there (EE 0's clock start) and never claimed the link manager: COMP_STATUS stayed 0x40002
    // (frame generator off), no interrupt, no slave.
    wr32(m, 0x008, 1);
    wr32(m, 0x008, 1);
    IOSleep(1);
    wr32(m, 0x101c, 1u << 3 | 7u);                              // frame bank 0: 50 rows, 16 columns
    wr32(m, 0x500, 1);                                          // auto enumeration
    wr32(m, 0x204, 0x1ffff);                                    // interrupts masked (SWRM_INTERRUPT_STATUS_RMSK)
    wr32(m, 0x1048, (rd32(m, 0x1048) & ~0x3e0000u) | 0x1fu << 17);   // 31 pings without a command
    wr32(m, 0x018, 1);                                          // link manager: the CPU's
    wr32(m, 0x1044, 1u << 1 << 1);                              // bus clock start for EE 1
    wr32(m, 0x314, 0x80000007u);                                // 7 retries, go on after an ignored command
    wr32(m, 0x004, 1);                                          // component on
    wr32(m, 0x004, 2);                                          // pulse interrupts
    wr32(m, 0x208, 0xffffffffu);
    wr32(m, 0x210, 0x1ffff);                                    // CPU interrupts on
    wr32(m, 0x004, 3);
    const uint64_t enabled = nowMs();
    IODelay(1000);
    native_mic_extra[5] = rd32(m, 0x14);
    for (unsigned i = 0; i < 200 && !(rd32(m, 0x14) & 1); ++i) IODelay(500);
    native_mic_stats[4] = (int32_t)rd32(m, 0x14);
    uint32_t slaves = 0;
    for (unsigned i = 0; i < 200 && !((slaves = rd32(m, 0x1090)) & 0x3fffcu); ++i) {
        IOSleep(5);
        if (!native_mic_extra[6] && nowMs() - enabled >= 100) native_mic_extra[6] = rd32(m, 0x14) | 0x80000000u;
    }
    native_mic_extra[7] = rd32(m, 0x14);
    native_mic_stats[5] = (int32_t)slaves;
    native_mic_stats[12] = (int32_t)rd32(m, 0x200);
    gSwr.dev = 0;
    for (unsigned d = 1; d < 12; ++d) {
        if (!((slaves >> (2 * d)) & 3)) continue;
        const uint32_t lo = rd32(m, 0x530 + 8 * d), hi = rd32(m, 0x534 + 8 * d);
        const uint64_t id = (uint64_t)(hi & 0xffff) << 32 | lo;
        if (!gSwr.dev || ((id >> 24) & 0xffff) == 0x0217) {
            gSwr.dev = (uint8_t)d;
            native_mic_stats[6] = (int32_t)lo;
            native_mic_stats[7] = (int32_t)hi;
        }
    }
    const uint32_t mr[7] = {0x4, 0x101c, 0x1044, 0x104c, 0x200, 0x14, 0x1124};
    for (unsigned k = 0; k < 7; ++k) native_mic_regs[7 + k] = rd32(m, mr[k]);
    native_mic_regs[0] = rd32(gTx.va, 0x0); native_mic_regs[1] = rd32(gTx.va, 0x4); native_mic_regs[2] = rd32(gTx.va, 0x8);
    native_mic_regs[3] = gVaMux ? rd32(gVaMux, 0) : 0xdead;
    native_mic_regs[4] = rd32(gVa.va, 0x0); native_mic_regs[5] = rd32(gVa.va, 0x4); native_mic_regs[6] = rd32(gVa.va, 0x8);
    return gSwr.dev != 0;
}

// The SoundWire core's attach writes, then the driver's probe: interrupt types, io_init, class load.
bool micCodec() {
    const uint8_t d = gSwr.dev;
    bool ok = swrWrite(d, 0x4d, 0x01) && swrWrite(d, 0x62, 0x02) && swrWrite(d, 0x72, 0x02);   // bus clock: 19.2 MHz base, scale /2
    uint8_t mask = 0;
    ok = ok && swrRead(d, 0x41, &mask) && swrWrite(d, 0x41, mask | 3);                         // SCP_IntMask1: parity, bus clash
    ok = ok && swrWrite(d, 0xf4, 0xff) && swrWrite(d, 0xf8, 0x0b) && swrWrite(d, 0xfc, 0xff);
    if (!ok) return false;
    uint8_t id0 = 0, id1 = 0, efuse = 0;
    if (wcdRead(W_PART_ID0, &id0) && wcdRead(W_PART_ID1, &id1)) native_mic_stats[8] = id0 | id1 << 8;
    wcdRead(W_EFUSE16, &efuse);
    ok = wcdUpdate(W_MBHC_SPARE_1, 0x03, efuse ? 0x01 : 0x03) && wcdUpdate(W_SLEEP_CTL, 0x0e, 0x0e) && wcdUpdate(W_SLEEP_CTL, 0x80, 0x80);
    IOSleep(1);
    ok = ok && wcdUpdate(W_SLEEP_CTL, 0x40, 0x40);
    IOSleep(1);
    ok = ok && wcdUpdate(W_BIAS_VBG_FINE, 0xf0, 0xb0) && wcdUpdate(W_ANA_BIAS, 0x80, 0x80) && wcdUpdate(W_ANA_BIAS, 0x40, 0x40);
    IOSleep(10);
    ok = ok && wcdUpdate(W_ANA_BIAS, 0x40, 0) && wcdUpdate(W_TX_CLK_CTL, 0x01, 0x01) && wcdUpdate(W_TXFE_DIV, 0x80, 0x80) &&
         wcdUpdate(W_PDM_WD0, 0x18, 0x10) && wcdUpdate(W_PDM_WD1, 0x18, 0x10) && wcdUpdate(W_MICB1_TEST2, 0x07, 0x01) &&
         wcdUpdate(W_MICB2_TEST2, 0x07, 0x01) && wcdUpdate(W_MICB3_TEST2, 0x07, 0x01) && wcdUpdate(W_SM0_MB_SEL, 0x03, 0x01) &&
         wcdUpdate(W_SM1_MB_SEL, 0x03, 0x02) && wcdUpdate(W_SM2_MB_SEL, 0x03, 0x03) && wcdUpdate(W_SYS_USAGE, 0x0f, 0);
    if (!ok) return false;
    // Class load: each SDCA function off, then on, settle, its sticky status cleared.
    ok = wcdWrite(W_AMP_ACT, 0) && wcdWrite(W_AMP_ACT, 1);
    IOSleep(20);
    ok = ok && wcdWrite(W_AMP_STAT, 0x7f) && wcdWrite(W_JACK_ACT, 0) && wcdWrite(W_JACK_ACT, 1);
    IOSleep(30);
    ok = ok && wcdUpdate(W_JACK_GRP, 0xff, 0x02) && wcdWrite(W_JACK_STAT, 0x7f);
    for (unsigned n = 0; n < 3 && ok; ++n) {
        ok = wcdWrite(wSmAct(n), 0) && wcdWrite(wSmAct(n), 1);
        IOSleep(5);
        ok = ok && wcdWrite(wSmStat(n), 0x7f);
    }
    return ok;
}

// Data port 1 of the codec and port 1 of the master in both banks (as sheng's WCD938x path, without bank switches), then micbias 1
// (2.75 V usage code 0x05), ADC1 from AMIC1, low power mode, gain 6, the SmartMIC0 sequencer to PS0, and TX decimator 0 from SWR_MIC0.
bool micPath() {
    const uint8_t d = gSwr.dev;
    uint32_t port = 1u | 1u << 8;
    PE_parse_boot_argn("asteroids-mic-port", &port, sizeof port);
    const uint8_t si = (uint8_t)port, off1 = (uint8_t)(port >> 8), lane = (uint8_t)((port >> 16) & 0xff);
    bool ok = swrWrite(d, 0x102, 0) && swrWrite(d, 0x103, 0);
    for (uint16_t bank = 0; bank <= 0x10 && ok; bank += 0x10)
        ok = swrWrite(d, (uint16_t)(0x122 + bank), si) && swrWrite(d, (uint16_t)(0x123 + bank), 0) && swrWrite(d, (uint16_t)(0x124 + bank), off1) &&
             swrWrite(d, (uint16_t)(0x125 + bank), 0) && swrWrite(d, (uint16_t)(0x126 + bank), 0xff) && swrWrite(d, (uint16_t)(0x128 + bank), lane) &&
             swrWrite(d, (uint16_t)(0xe0 + bank), 0x01);
    for (uint16_t bank = 0; bank <= 0x10 && ok; bank += 0x10) ok = swrWrite(d, (uint16_t)(0x120 + bank), 0x01);
    if (!ok) return false;
    const uintptr_t m = gSwr.r.va;
    for (uint32_t bank = 0; bank < 2; ++bank) {
        wr32(m, 0x1124 + 0x40 * bank, si | (uint32_t)off1 << 8 | 0x01u << 24);   // channel 1 enabled
        wr32(m, 0x1128 + 0x40 * bank, lane);
        wr32(m, 0x1134 + 0x40 * bank, 0xf0);
    }
    wr32(m, 0x112c, 0);
    ok = wcdUpdate(W_TX_CH12_MUX, 0x07, 0x01) && wcdWrite(W_SM0_MICB, 0x05);
    IOSleep(1);
    ok = ok && wcdUpdate(W_ANA_TX_CH1, 0x1f, 6) && wcdUpdate(W_SYS_USAGE, 0x0f, 0) && wcdWrite(W_SM0_USAGE, 0x05) &&
         wcdUpdate(W_ANA_TX_CH2, 0x40, 0x40) && wcdWrite(W_SM0_REQ_PS, 0x00);
    IODelay(800);
    ok = ok && wcdUpdate(W_ANA_TX_CH2, 0x40, 0);
    uint8_t act = 0xff;
    for (unsigned i = 0; i < 20 && ok; ++i) {
        if (wcdRead(W_SM0_ACT_PS, &act) && act == 0) break;
        IODelay(500);
    }
    native_mic_stats[9] = act;
    if (!ok || act != 0) return false;
    // TX decimator 0: SMIC mux 0 = SWR_MIC0, DEC0 mux = SWR_MIC, 48 kHz, 150 Hz high-pass, clock on, muted until the path settles.
    const uintptr_t tx = gTx.va;
    wr32(tx, 0x100, (rd32(tx, 0x100) & ~0xfu) | 1);
    wr32(tx, 0x104, (rd32(tx, 0x104) & ~3u) | 1);
    wr32(tx, 0x404, (rd32(tx, 0x404) & ~(3u << 5 | 3u << 1)) | 2u << 5);
    wr32(tx, 0x400, (rd32(tx, 0x400) & ~0x3fu) | 4 | 0x10);
    wr32(tx, 0x400, rd32(tx, 0x400) | 0x20);
    const uint32_t txr[4] = {0x0, 0x4, 0x100, 0x400}, var[3] = {0x0, 0x4, 0x80};
    for (unsigned k = 0; k < 4; ++k) native_mic_regs[k] = rd32(tx, txr[k]);
    for (unsigned k = 0; k < 3; ++k) native_mic_regs[4 + k] = rd32(gVa.va, var[k]);
    return true;
}

void capStage(int s) {native_mic_capture[0] = s;}
// sheng's capture test: the graphs, then 200 periods (2 s) through four buffers in the second half of the DSP buffer.
bool micCaptureTest() {
    capStage(1);
    uint8_t b[GPR_MAX_PACKET];
    const ArFormat f = {kRate, 16, 1};
    if (!command(ar_open_mic_device(b, sizeof b, kPort, ++gAudio.token), b, AR_OP_GRAPH_OPEN)) {capStage(-6); return false;}
    if (!command(ar_config_mic_device(b, sizeof b, kPort, ++gAudio.token, &f, 1), b, AR_OP_SET_CFG)) {capStage(-7); return false;}
    if (!command(ar_open_capture_stream(b, sizeof b, kPort, ++gAudio.token), b, AR_OP_GRAPH_OPEN)) {capStage(-8); return false;}
    if (!command(ar_config_capture_stream(b, sizeof b, kPort, ++gAudio.token, &f), b, AR_OP_SET_CFG)) {capStage(-9); return false;}
    for (uint32_t sg : kMicUp)
        if (!command(ar_graph_command(b, sizeof b, kPort, ++gAudio.token, AR_OP_GRAPH_PREPARE, sg), b, AR_OP_GRAPH_PREPARE)) {capStage(-10); return false;}
    for (uint32_t sg : kMicUp)
        if (!command(ar_graph_command(b, sizeof b, kPort, ++gAudio.token, AR_OP_GRAPH_START, sg), b, AR_OP_GRAPH_START)) {capStage(-11); return false;}
    capStage(6);
    IOSleep(300);
    wr32(gTx.va, 0x400, rd32(gTx.va, 0x400) & ~0x10u);   // unmute after the analog path's settling time
    uint64_t sum = 0, samples = 0;
    int32_t peak = 0, nonzero = 0;
    uint32_t asked = 0, back = 0;
    auto ask = [&](uint32_t n) {
        const uint32_t slot = n % kCapSlots;
        if (send(b, ar_read(b, sizeof b, kPort, n, gAudio.dspAddr + kCapOffset + slot * kCapBytes, gAudio.handle, kCapBytes))) {++asked; native_mic_capture[2] = (int32_t)asked;}
    };
    uint32_t seconds = 2;
    PE_parse_boot_argn("asteroids-mic-capture-s", &seconds, sizeof seconds);
    if (seconds < 1) seconds = 1;
    if (seconds > 30) seconds = 30;
    const uint32_t periods = seconds * 100;
    native_mic_capture_s = seconds;
    native_mic_capture_at = (uint32_t)nowMs();
    for (uint32_t n = 0; n < 4; ++n) ask(n);
    while (back < periods) {
        ArAnswer a;
        if (!nextAnswer(a, 1000)) {capStage(-12); break;}
        if (a.opcode == AR_OP_READ_DONE) {
            native_mic_capture[1] = (int32_t)a.status;
            const uint32_t slot = (uint32_t)((a.address - (gAudio.dspAddr + kCapOffset)) / kCapBytes) % kCapSlots;
            const int16_t *p = reinterpret_cast<const int16_t *>(gAudio.bufferVa + kCapOffset + slot * kCapBytes);
            const uint32_t count = (a.data_bytes ? a.data_bytes : kCapBytes) / 2;
            for (uint32_t i = 0; i < count && i < kPeriodFrames; ++i) {
                const int32_t v = p[i] < 0 ? -p[i] : p[i];
                sum += (uint32_t)v; ++samples;
                if (v > peak) peak = v;
                if (v > native_mic_peaks[back / 100]) native_mic_peaks[back / 100] = v;
                if (v) ++nonzero;
            }
            native_mic_capture[3] = (int32_t)++back;
            native_mic_capture[4] = native_mic_capture[4] + (int32_t)a.data_bytes;
            native_mic_capture[5] = peak;
            native_mic_capture[6] = samples ? (int32_t)(sum / samples) : 0;
            native_mic_capture[7] = nonzero;
            if (asked < periods) ask(asked);
        } else if (a.opcode == GPR_OP_BASIC_RESULT && a.status) {
            native_mic_capture[1] = (int32_t)a.status;
        }
    }
    for (uint32_t sg : kMicDown) command(ar_graph_command(b, sizeof b, kPort, ++gAudio.token, AR_OP_GRAPH_STOP, sg), b, AR_OP_GRAPH_STOP);
    if (native_mic_capture[0] > 0) capStage(7);
    return native_mic_capture[0] == 7;
}

void micBringUp(unsigned want) {
    const uint64_t t0 = nowMs();
    micStage(1);
    if (!micRails()) {micFail(1); return;}
    Region reset;
    if (!mapRegion(kTlmm + kWcdResetPin * 0x1000ull, 0x1000, reset)) {micFail(1); return;}
    wr32(reset.va, 4, 0);
    // GPIO, output, 16 mA (DT wcd_reset_active). gpio162 is one of the TLMM's eGPIOs (132-166): without EGPIO_ENABLE (bit 12) the pad
    // belongs to the other owner and stays low (T42: GPIO_IN_OUT 2, output high read back low, the codec held in reset and the bus empty).
    // pinctrl-msm sets the bit for every function other than egpio.
    wr32(reset.va, 0, 1u << 12 | 1u << 9 | 7u << 6);
    IOSleep(1);
    wr32(reset.va, 4, 1u << 1);             // out of reset (active state: output high)
    IOSleep(5);
    native_mic_extra[0] = rd32(reset.va, 4);
    micPmic();
    micStage(10);
    if (want < 2) return;
    if (!micPrm()) {micFail(2); return;}
    micStage(20);
    if (want < 3) return;
    if (!micPins() || !micMacros()) {micFail(3); return;}
    micStage(30);
    if (want < 4) return;
    if (!micMaster()) {micFail(4); native_mic_stats[15] = (int32_t)(nowMs() - t0); return;}
    micStage(40);
    if (want < 5) return;
    if (!micCodec()) {micFail(5); native_mic_stats[15] = (int32_t)(nowMs() - t0); return;}
    micStage(50);
    if (want < 6) return;
    if (!micPath()) {micFail(6); native_mic_stats[15] = (int32_t)(nowMs() - t0); return;}
    micStage(60);
    native_mic_stats[15] = (int32_t)(nowMs() - t0);
    if (want < 7) return;
    if (!micCaptureTest()) {micFail(7); return;}
    micStage(70);
    if (want < 8) return;
    gAudio.micReady = true;
    micStage(80);
}

// Capture for the plug-in's input stream (sheng's, once the microphone test passed): periods read from the DSP land in the capture ring
// at capPos, which advances by the frames each one carries; syncCapture moves it when the DSP's clock drifts from the host's.
bool askCapture(uint64_t n) {
    uint8_t b[GPR_MAX_PACKET];
    const uint32_t slot = (uint32_t)(n % kCapSlots);
    if (!send(b, ar_read(b, sizeof b, kPort, (uint32_t)n, gAudio.dspAddr + kCapOffset + slot * kCapBytes, gAudio.handle, kCapBytes))) return false;
    gAudio.capInFlight++;
    return true;
}
int64_t gBaseCapture;
void syncCapture() {
    const int64_t gap = (int64_t)framesNow() - (int64_t)gAudio.capPos;
    if (gAudio.capDone < 2 * kInFlight) return;
    if (gAudio.capDone == 2 * kInFlight) {gBaseCapture = gap; return;}
    if (gap - gBaseCapture > kDriftLimit || gBaseCapture - gap > kDriftLimit) {
        gAudio.capPos = (uint64_t)((int64_t)gAudio.capPos + gap - gBaseCapture);
        native_audio_sync[3] = native_audio_sync[3] + 1;
    }
}
void startInput() {
    if (gAudio.capturing || !gAudio.micReady) return;
    uint8_t b[GPR_MAX_PACKET];
    for (uint32_t sg : kMicUp) command(ar_graph_command(b, sizeof b, kPort, ++gAudio.token, AR_OP_GRAPH_PREPARE, sg), b, AR_OP_GRAPH_PREPARE);
    for (uint32_t sg : kMicUp)
        if (!command(ar_graph_command(b, sizeof b, kPort, ++gAudio.token, AR_OP_GRAPH_START, sg), b, AR_OP_GRAPH_START)) return;
    ShengAudioControl *c = gAudio.ctl;
    gAudio.capPos = framesNow();
    c->in_written = gAudio.capPos;
    c->in_frames = 0;
    memset(gAudio.in, 0, kRingFrames * 2);
    gAudio.capNext = gAudio.capDone = 0;
    gAudio.capInFlight = 0;
    gAudio.capturing = true;
    while (gAudio.capInFlight < kInFlight && askCapture(gAudio.capNext)) gAudio.capNext++;
    __atomic_or_fetch(&c->state, ShengAudioInputRunning, __ATOMIC_ACQ_REL);
    native_audio_stats[14] = native_audio_stats[14] + (1 << 8);
}
void stopInput() {
    if (!gAudio.capturing) return;
    gAudio.capturing = false;
    uint8_t b[GPR_MAX_PACKET];
    for (uint32_t sg : kMicDown) command(ar_graph_command(b, sizeof b, kPort, ++gAudio.token, AR_OP_GRAPH_STOP, sg), b, AR_OP_GRAPH_STOP);
    __atomic_and_fetch(&gAudio.ctl->state, ~ShengAudioInputRunning, __ATOMIC_ACQ_REL);
    native_audio_stats[14] = native_audio_stats[14] + (1 << 24);
}
// A returned capture buffer: only the bytes the DSP filled, from its offset (sheng's i514), go to the ring at the capture position.
void captured(const ArAnswer &a) {
    if (gAudio.capInFlight) gAudio.capInFlight--;
    const uint32_t slot = (uint32_t)((a.address - (gAudio.dspAddr + kCapOffset)) / kCapBytes) % kCapSlots;
    const uint32_t offset = a.offset < kCapBytes ? a.offset & ~1u : 0, bytes = a.data_bytes < kCapBytes - offset ? a.data_bytes : kCapBytes - offset;
    const int16_t *src = reinterpret_cast<const int16_t *>(gAudio.bufferVa + kCapOffset + slot * kCapBytes + offset);
    const uint32_t count = bytes / 2;
    syncCapture();
    const uint64_t t = gAudio.capPos;
    for (uint32_t i = 0; i < count; ++i) gAudio.in[(t + i) % kRingFrames] = src[i];
    if (count) notePeak(src, count, 16);
    gAudio.capPos = t + count;
    gAudio.ctl->in_written = gAudio.capPos;
    if (count < kPeriodFrames) native_audio_sync[3] = native_audio_sync[3] + (1 << 16);
    gAudio.capDone++;
    gAudio.ctl->in_frames = gAudio.ctl->in_frames + count;
    native_mic_capture[3] = native_mic_capture[3] + 1;
    if (gAudio.capturing) while (gAudio.capInFlight < kInFlight && askCapture(gAudio.capNext)) gAudio.capNext++;
}

void handle(const ArAnswer &a) {
    if (a.opcode == AR_OP_WRITE_DONE) {
        native_audio_stats[5] = native_audio_stats[5] + 1;
        if (a.status) native_audio_stats[1] = (int32_t)a.status;
        if (gAudio.inFlight) gAudio.inFlight--;
        gAudio.done++;
        gAudio.ctl->out_frames = (gAudio.done - gAudio.first) * kPeriodFrames;
        if (gAudio.running) {
            if (!gAudio.tone) syncPlayback();
            while (gAudio.inFlight < kInFlight && submit(gAudio.next)) gAudio.next++;
        }
        if (gAudio.running && !gAudio.statusRead && gAudio.done - gAudio.first == 10) {readAmpStatus(); gAudio.statusRead = true;}
    } else if (a.opcode == AR_OP_READ_DONE) {
        captured(a);
    } else if (a.opcode == GPR_OP_BASIC_RESULT && a.status) {
        native_audio_stats[1] = (int32_t)a.status;
    }
}

// Serves requests and answers until the end of time; with ms > 0 it returns after ms (the test tone).
void serve(uint32_t ms) {
    const uint64_t end = ms ? nowMs() + ms : 0;
    for (;;) {
        const uint32_t want = __atomic_load_n(&gAudio.want, __ATOMIC_ACQUIRE);
        if (!ms) {
            // The path stays up kKeepMs after the plug-in stops: the amplifiers' power-up would cut the start of the next sound.
            if (want & ShengAudioOutputRunning) {gAudio.stopAsked = 0; if (!gAudio.running) startOutput();}
            else if (gAudio.running && !gAudio.stopAsked) gAudio.stopAsked = nowMs();
            else if (gAudio.running && nowMs() - gAudio.stopAsked >= kKeepMs) {gAudio.stopAsked = 0; stopOutput();}
            if ((want & ShengAudioInputRunning) && !gAudio.capturing) startInput();
            if (!(want & ShengAudioInputRunning) && gAudio.capturing) stopInput();
        }
        ArAnswer a;
        if (nextAnswer(a, gAudio.running || gAudio.capturing ? 100 : 1000)) handle(a);
        if (end && nowMs() >= end) return;
    }
}

void playTone() {
    const unsigned shift = gAudio.atten / 6 > 15 ? 15 : gAudio.atten / 6;
    for (uint32_t i = 0; i < kRingFrames; ++i) gAudio.out[2 * i] = gAudio.out[2 * i + 1] = (int16_t)(kSine[i % 64] >> shift);
    gAudio.tone = true;
    startOutput();
    serve(gAudio.toneMs);
    // Silence through the DSP's queue before the amplifiers power down, so the tone does not end in a click.
    memset(gAudio.out, 0, kRingFrames * kChannels * 2);
    serve(80);
    stopOutput();
    gAudio.tone = false;
    memset(gAudio.out, 0, kRingFrames * kChannels * 2);
    native_audio_service[7] = 0;
    stage(57);
}

// sheng-audio bit 8: at 150 s of uptime one synthetic power button press locks the screen, and SpringBoard plays its lock sound through
// the audio server, the plug-in and this service; [audio-service] then counts the requests.
extern "C" bool sheng_buttons_press(uint32_t mask, uint32_t ms) __attribute__((weak_import));
void lockSoundThread(void *, wait_result_t) {
    uint64_t up = 0;
    absolutetime_to_nanoseconds(mach_absolute_time(), &up);
    const uint64_t at = 150ull * 1000000000ull;
    if (up < at) IOSleep((uint32_t)((at - up) / 1000000ull));
    native_audio_service[6] = sheng_buttons_press && sheng_buttons_press(1u << 1, 150) ? 1 : -1;
}

bool openGraphs() {
    uint8_t b[GPR_MAX_PACKET];
    const ArFormat f = {kRate, 16, kChannels}, device = {kRate, gAudio.deviceBits, kChannels};
    if (!command(ar_open_speaker_device(b, sizeof b, kPort, ++gAudio.token), b, AR_OP_GRAPH_OPEN)) {fail(54); return false;}
    if (!command(ar_config_speaker_device(b, sizeof b, kPort, ++gAudio.token, &device, &gAudio.i2s), b, AR_OP_SET_CFG)) {fail(55); return false;}
    if (!command(ar_open_playback_stream(b, sizeof b, kPort, ++gAudio.token), b, AR_OP_GRAPH_OPEN)) {fail(56); return false;}
    if (!command(ar_config_playback_stream(b, sizeof b, kPort, ++gAudio.token, &f, &device), b, AR_OP_SET_CFG)) {fail(57); return false;}
    if (!command(ar_config_playback_shmem(b, sizeof b, kPort, ++gAudio.token, &f), b, AR_OP_SET_CFG)) {fail(58); return false;}
    if (!command(ar_map(b, sizeof b, kPort, ++gAudio.token, gAudio.dspAddr, kBufferBytes), b, AR_OP_MEM_MAP)) {fail(53); return false;}
    return true;
}

bool publishService() {
    void *handle = OSMetaClass::preModLoad(g_kernel_kmod_info.name);
    if (!handle) return false;
    native_audio_service_static_init();
    if (OSMetaClass::postModLoad(handle)) return false;
    sheng_audio_service_start();
    return true;
}

void audioThread(void *, wait_result_t) {
    thread_set_thread_name(current_thread(), "asteroids-audio");
    const uint64_t start = nowMs();
    stage(10);
    if (!setupBuffer() || !setupShared()) {fail(10); return;}
    if (!setupSmmu()) {fail(11); return;}
    stage(20);
    if (!setupPins()) {fail(20); return;}
    stage(30);
    if (!setupClocks()) {fail(30); return;}
    if (!setupBus()) {fail(31); return;}
    stage(40);
    if (!setupAmps()) {fail(40); return;}
    stage(50);
    static const ShengAdspClient client = {dspState, dspReceive};
    gAudio.channel = sheng_adsp_channel("adsp_apps", kIntentSize, kIntentCount, 0, &client, nullptr);
    if (!gAudio.channel) {fail(50); return;}
    if (!waitUp(60000)) {fail(51); return;}
    stage(51);
    if (!dspReady()) return;
    stage(52);
    prmVotes();
    if (!openGraphs()) return;
    stage(56);
    if (gAudio.toneMs) playTone();
    if (gAudio.micWant) micBringUp(gAudio.micWant);
    if (!publishService()) {fail(58); return;}
    __atomic_store_n(&gAudio.ctl->state, gAudio.ctl->state | ShengAudioOutputReady, __ATOMIC_RELEASE);
    __atomic_store_n(&gAudio.serving, 1u, __ATOMIC_RELEASE);
    native_audio_stats[15] = (int32_t)(nowMs() - start);
    stage(60);
    if (gAudio.mode & 8) {
        thread_t thread;
        if (kernel_thread_start(lockSoundThread, nullptr, &thread) == KERN_SUCCESS) thread_deallocate(thread);
    }
    serve(0);
}
}  // namespace

// The plug-in's record of unanswered properties (ShengAudioDiag in the control page), for the log.
extern "C" const ShengAudioDiag *sheng_audio_diag(void) {
    return gAudio.ctl ? reinterpret_cast<const ShengAudioDiag *>(reinterpret_cast<const uint8_t *>(gAudio.ctl) + SHENG_AUDIO_DIAG_OFFSET) : nullptr;
}

extern "C" IOBufferMemoryDescriptor *sheng_audio_memory(uint32_t type) {
    if (!__atomic_load_n(&gAudio.serving, __ATOMIC_ACQUIRE)) return nullptr;
    IOBufferMemoryDescriptor *m = type == ShengAudioMemoryControl ? gAudio.control.get() : type == ShengAudioMemoryOutput ? gAudio.outRing.get() :
                                  type == ShengAudioMemoryInput ? gAudio.inRing.get() : nullptr;
    if (m) m->retain();
    return m;
}

// [audio-requests]: the plug-in's last 16 requests, each ms since boot << 4 | selector, oldest first from [16] (the next slot). Input
// requests start capture only once the microphone reached stage 8 (asteroids-mic=8); before that the input ring stays silent.
extern "C" {volatile uint64_t native_audio_requests[17];}
extern "C" int sheng_audio_request(uint32_t selector) {
    if (!__atomic_load_n(&gAudio.serving, __ATOMIC_ACQUIRE)) return -1;
    const uint64_t slot = __atomic_fetch_add(&native_audio_requests[16], 1, __ATOMIC_RELAXED) % 16;
    native_audio_requests[slot] = nowMs() << 4 | (selector & 15);
    switch (selector) {
    case ShengAudioStartOutput: __atomic_or_fetch(&gAudio.want, ShengAudioOutputRunning, __ATOMIC_ACQ_REL); break;
    case ShengAudioStopOutput: __atomic_and_fetch(&gAudio.want, ~ShengAudioOutputRunning, __ATOMIC_ACQ_REL); break;
    case ShengAudioStartInput: __atomic_or_fetch(&gAudio.want, ShengAudioInputRunning, __ATOMIC_ACQ_REL); break;
    case ShengAudioStopInput: __atomic_and_fetch(&gAudio.want, ~ShengAudioInputRunning, __ATOMIC_ACQ_REL); break;
    default: return -1;
    }
    IOLockWakeup(gAudio.lock, &gAudio.ringCount, false);
    return 0;
}

extern "C" void sheng_audio_probe(void) {
    unsigned mode = 0;
    if (!PE_parse_boot_argn("sheng-audio", &mode, sizeof mode) || !mode || mode > 15 || native_audio_stats[0]) return;
    gAudio.mode = mode;
    gAudio.toneMs = (mode & 2) ? 1500 : 0;
    gAudio.atten = 30;
    gAudio.prm = 3;
    uint32_t port = AR_I2S_PRIMARY | AR_I2S_SD1 << 8 | 32u << 16;
    PE_parse_boot_argn("sheng-audio-tone-ms", &gAudio.toneMs, sizeof gAudio.toneMs);
    PE_parse_boot_argn("sheng-audio-atten", &gAudio.atten, sizeof gAudio.atten);
    PE_parse_boot_argn("asteroids-audio-prm", &gAudio.prm, sizeof gAudio.prm);
    PE_parse_boot_argn("asteroids-audio-i2s", &port, sizeof port);
    PE_parse_boot_argn("asteroids-mic", &gAudio.micWant, sizeof gAudio.micWant);
    if (gAudio.micWant > 8) gAudio.micWant = 8;
    gAudio.i2s.intf = port & 0xff;
    gAudio.i2s.lines = (uint16_t)((port >> 8) & 0xff);
    gAudio.deviceBits = (port >> 16) == 16 ? 16 : 32;
    native_audio_stats[0] = 1;
    if (!sheng_adsp_channel || !sheng_adsp_send) {fail(1); return;}
    gAudio.lock = IOLockAlloc();
    gAudio.ringLock = IOSimpleLockAlloc();
    if (!gAudio.lock || !gAudio.ringLock) {fail(2); return;}
    thread_t thread;
    if (kernel_thread_start(audioThread, nullptr, &thread) != KERN_SUCCESS) {fail(3); return;}
    thread_deallocate(thread);
}
