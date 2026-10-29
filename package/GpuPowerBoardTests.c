// SPDX-License-Identifier: BSD-2-Clause-Patent
// Host test of GpuPowerBoard.h (asteroids, modes 1-13) over fake register windows: which registers each mode reads, that mode 1
// never writes, that mode 2 writes only the gcc_gpu_cfg_ahb_clk enable and only when it is off, the gating, and that every access
// stays inside its window. Build: scripts/check-asteroids-gpu-power.py.
#include "GpuPowerBoard.h"
#ifdef GPB_HAVE_SHENG_SOURCE
#include "ShengGmuSource.h"   // build/sheng-gmu, generated from the pinned linux XML: mode 8's offsets must match it
#endif
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#define CHECK(x) do {if(!(x)){fprintf(stderr,"line %d: %s\n",__LINE__,#x);exit(1);}}while(0)
static const uint32_t sizes[GPB_WINDOWS] = {0x1000, 0x1000, (uint32_t)GPB_GPUCC_SIZE, (uint32_t)GPB_SMMU_GR0_SIZE,
                                            (uint32_t)GPB_GPU_WINDOW};
static uint32_t mem[GPB_WINDOWS][0x9800 / 4];
static struct {unsigned window; uint32_t offset; uint32_t value;} reads[2048], writes[64];
static int hw7 = 0, pll_locks = 1, rcg_updates = 1;   // fake PLL0 / RCGs for mode 7
static int cx_on_after = -1;   // fake GDSC: polls of the hw-ctrl register left until PWR_ON, once SW_COLLAPSE is cleared
static unsigned delayed;
static unsigned nreads, nwrites, fail_read_window = 99;
static int rd(void *cookie, unsigned w, uint32_t off, uint32_t *out) {
    CHECK(cookie == mem && w < GPB_WINDOWS && !(off & 3) && off + 4 <= sizes[w]);
    CHECK(nreads < 2048); reads[nreads].window = w; reads[nreads].offset = off; ++nreads;
    if (w == fail_read_window) return 1;
    if (w == GPB_WIN_GPUCC && off == GPB_CC_CX_GDS_HW_CTRL && cx_on_after >= 0 &&
        !(mem[GPB_WIN_GPUCC][GPB_CC_CX_GDSCR / 4] & GPB_GDSC_SW_COLLAPSE) && cx_on_after-- == 0)
        mem[GPB_WIN_GPUCC][GPB_CC_CX_GDS_HW_CTRL / 4] |= GPB_GDSC_PWR_ON;
    *out = mem[w][off / 4]; return 0;
}
static int wr(void *cookie, unsigned w, uint32_t off, uint32_t v) {
    CHECK(cookie == mem && w < GPB_WINDOWS && !(off & 3) && off + 4 <= sizes[w]);
    CHECK(nwrites < 64); writes[nwrites].window = w; writes[nwrites].offset = off; writes[nwrites].value = v; ++nwrites;
    mem[w][off / 4] = v;
    if (w == GPB_WIN_GPUCC && off == GPB_CC_SMMU_VOTE_CLK && (v & GPB_CBCR_ENABLE)) mem[w][off / 4] &= ~GPB_CBCR_CLK_OFF;
    if (hw7 && w == GPB_WIN_GPUCC) {
        if (off == GPB_PLL_OPMODE && v == 1 && pll_locks) mem[w][0] |= GPB_PLL_LOCK_DET;
        if ((off == GPB_CC_HUB_RCGR || off == GPB_CC_GMU_RCGR) && rcg_updates) mem[w][off / 4] &= ~(GPB_RCG_UPDATE | GPB_RCG_ROOT_OFF);
    }
    return 0;
}
static unsigned stages[64], nstages;
static void stage(void *cookie, unsigned w) {CHECK(cookie == mem && nstages < 64); stages[nstages++] = w;}
static void delay(void *cookie, unsigned us) {CHECK(cookie == mem); delayed += us;}
static const struct gpb_io io = {mem, rd, wr, stage, delay};
static const unsigned all = (1u << GPB_WINDOWS) - 1;
static void reset(void) {memset(mem, 0, sizeof mem); hw7 = 0; pll_locks = rcg_updates = 1; nreads = nwrites = nstages = delayed = 0; fail_read_window = 99; cx_on_after = -1;}
static int was_read(unsigned w, uint32_t off) {
    for (unsigned i = 0; i < nreads; ++i) if (reads[i].window == w && reads[i].offset == off) return 1;
    return 0;
}
static unsigned reads_in(unsigned w) {unsigned n = 0; for (unsigned i = 0; i < nreads; ++i) n += reads[i].window == w; return n;}
// Mode 8's fake GMU and RSCC windows: any in-bounds offset reads its value from a short list, else 0.
static struct {unsigned w; uint32_t off, v;} gmu_vals[24];
static unsigned ngmu_vals, gmu_fail_at = 99, ao_tick;
static int rd8(void *cookie, unsigned w, uint32_t off, uint32_t *out) {
    CHECK(cookie == mem && (w == GPB_WIN_GMU || w == GPB_WIN_RSCC) && !(off & 3));
    CHECK(off + 4 <= (w == GPB_WIN_GMU ? GPB_GMU_SIZE : GPB_RSCC_SIZE));
    CHECK(nreads < 2048); reads[nreads].window = w; reads[nreads].offset = off;
    if (nreads++ == gmu_fail_at) return 1;
    *out = 0;
    for (unsigned i = 0; i < ngmu_vals; ++i) if (gmu_vals[i].w == w && gmu_vals[i].off == off) *out = gmu_vals[i].v;
    if (w == GPB_WIN_GMU && off == GPB_GMU_AO_COUNTER_L) *out += ao_tick++ * 192;
    return 0;
}
static int wr8(void *cookie, unsigned w, uint32_t off, uint32_t v) {(void)cookie; (void)w; (void)off; (void)v; CHECK(0); return 1;}
static void gmu_set(unsigned w, uint32_t off, uint32_t v) {gmu_vals[ngmu_vals].w = w; gmu_vals[ngmu_vals].off = off; gmu_vals[ngmu_vals++].v = v;}
// Mode 11's fake GMU: the OOB ack and GX power follow the request, as configured per case.
static struct {unsigned w; uint32_t off, v;} gx_writes[16];
static unsigned ngx_writes, gx_reads, gx_ack = 1, gx_powers = 1, gx_vote_fail, gx_votes, gx_fail_read = ~0u;
static uint32_t gx_info, gx_status = 3, gx_vote_perf, gx_vote_bw, gx_delayed;
static int gx_rd(void *c, unsigned w, uint32_t off, uint32_t *out) {
    CHECK(c == mem && (w == GPB_WIN_GMU || w == GPB_WIN_GPU) && !(off & 3));
    CHECK(off + 4 <= (w == GPB_WIN_GMU ? GPB_GMU_SIZE : GPB_GPU_WINDOW));
    if (gx_reads++ == gx_fail_read) return 1;
    *out = w == GPB_WIN_GPU ? 0x1000 + off : off == GPB_GMU_PWR_CLK_STATUS ? gx_status :
           off == GPB_GMU_GMU2HOST_INTR_INFO ? gx_info : 0;
    return 0;
}
static int gx_wr(void *c, unsigned w, uint32_t off, uint32_t v) {
    CHECK(c == mem && w == GPB_WIN_GMU && ngx_writes < 16);   // never the GPU block
    gx_writes[ngx_writes].w = w; gx_writes[ngx_writes].off = off; gx_writes[ngx_writes++].v = v;
    if (off == GPB_GMU_HOST2GMU_INTR_SET && v == GPB_OOB_GPU_SET_REQUEST) {
        if (gx_ack) gx_info |= GPB_OOB_GPU_SET_ACK;
        if (gx_powers) gx_status = 0;
    }
    if (off == GPB_GMU_GMU2HOST_INTR_CLR) gx_info &= ~v;
    return 0;
}
static int gx_vote(void *c, uint32_t perf, uint32_t bw, uint32_t *fw) {
    CHECK(c == mem); ++gx_votes; gx_vote_perf = perf; gx_vote_bw = bw; *fw = gx_vote_fail == 2 ? 7 : 0;
    return gx_vote_fail == 1;
}
static void gx_delay(void *c, unsigned us) {CHECK(c == mem); gx_delayed += us;}
static void gx_reset(void) {ngx_writes = gx_reads = gx_votes = gx_vote_fail = 0; gx_ack = gx_powers = 1; gx_info = 0x1;
    gx_status = 3; gx_delayed = 0; gx_fail_read = ~0u;}
// Modes 12-13: GPU, GMU and CX_MISC as plain register files; every write is logged with the aperture it went through.
static uint32_t hw_gpu[0x40000 / 4], hw_gmu[GPB_GMU_SIZE / 4], hw_cx[0x2000 / 4];
static struct {unsigned w; uint32_t off, v, aperture;} hw_log[1024];
static unsigned nhw, hw_fail_at = ~0u, hw_ops, hw_cx_writes;
static uint32_t *hw_reg(unsigned w, uint32_t off) {
    CHECK(!(off & 3));
    if (w == GPB_WIN_GPU) {CHECK(off < sizeof hw_gpu); return &hw_gpu[off / 4];}
    if (w == GPB_WIN_GMU) {CHECK(off < sizeof hw_gmu); return &hw_gmu[off / 4];}
    CHECK(w == GPB_WIN_CX && off < sizeof hw_cx); return &hw_cx[off / 4];
}
static int hw_rd(void *c, unsigned w, uint32_t off, uint32_t *out) {
    CHECK(c == hw_gpu);
    if (hw_ops++ == hw_fail_at) return 1;
    *out = *hw_reg(w, off); return 0;
}
static int hw_wr(void *c, unsigned w, uint32_t off, uint32_t v) {
    CHECK(c == hw_gpu && nhw < 1024);
    if (hw_ops++ == hw_fail_at) return 1;
    if (w == GPB_WIN_CX) ++hw_cx_writes;
    hw_log[nhw].w = w; hw_log[nhw].off = off; hw_log[nhw].v = v;
    hw_log[nhw++].aperture = hw_gpu[A810_CP_APERTURE_CNTL_HOST];
    *hw_reg(w, off) = v; return 0;
}
static void hw_reset(void) {memset(hw_gpu, 0, sizeof hw_gpu); memset(hw_gmu, 0, sizeof hw_gmu); memset(hw_cx, 0, sizeof hw_cx);
    nhw = hw_ops = hw_cx_writes = 0; hw_fail_at = ~0u;}
// How many logged writes match {window, offset, value, aperture pipe}; value ~0u matches any.
static unsigned hw_count(unsigned w, uint32_t dword_or_off, uint32_t v, unsigned pipe) {
    const uint32_t off = w == GPB_WIN_GPU ? GPB_GPU_REG(dword_or_off) : dword_or_off;
    unsigned n = 0;
    for (unsigned i = 0; i < nhw; ++i)
        if (hw_log[i].w == w && hw_log[i].off == off && (v == ~0u || hw_log[i].v == v) &&
            hw_log[i].aperture == pipe << 12) ++n;
    return n;
}
static int hw_written(uint32_t dword) {
    for (unsigned i = 0; i < nhw; ++i) if (hw_log[i].w == GPB_WIN_GPU && hw_log[i].off == GPB_GPU_REG(dword)) return 1;
    return 0;
}
int main(void) {
    struct gpb_result r;
    // The facts every read relies on.
    CHECK(GPB_GCC_GPU_PAGE == GPB_GCC_BASE + 0x71000 && GPB_GCC_VOTE_PAGE == GPB_GCC_BASE + 0x52000);
    CHECK(GPB_CC_CX_GDSCR == 0x9080 && GPB_CC_CX_GDS_HW_CTRL == 0x9094 && GPB_CC_GX_GDSCR == 0x905c);
    CHECK(GPB_CC_AHB_CLK < GPB_GPUCC_SIZE && GPB_CC_CX_GMU_CLK < GPB_GPUCC_SIZE && GPB_RBBM_STATUS_BYTES == 0x48);

    // Mode 1, the boot state Linux leaves (cfg_ahb enabled), CX off: GCC and GPUCC read, SMMU and GPU skipped, no writes.
    reset(); mem[GPB_WIN_GCC_GPU][GPB_GCC_GPU_CFG_AHB / 4] = 0x80000003; mem[GPB_WIN_GPUCC][GPB_CC_CX_GDSCR / 4] = 0x00000001;   // SW_COLLAPSE set, PWR_ON clear
    gpb_run(&io, 1, all, &r);
    CHECK(r.gcc == GPB_DONE && r.cc == GPB_DONE && r.smmu == GPB_SKIPPED_PREREQ && r.gpu == GPB_SKIPPED_PREREQ);
    CHECK(!nwrites && !r.writes && !r.io_error && r.cfg_ahb == 0x80000003 && r.cx_gdscr == 0x00000001);
    CHECK(was_read(GPB_WIN_GCC_GPU, 0x004) && was_read(GPB_WIN_GCC_GPU, 0x010) && was_read(GPB_WIN_GCC_GPU, 0x018));
    CHECK(was_read(GPB_WIN_GCC_GPU, 0x15c) && was_read(GPB_WIN_GCC_VOTE, 0x000));
    CHECK(was_read(GPB_WIN_GPUCC, 0x9080) && was_read(GPB_WIN_GPUCC, 0x9094) && was_read(GPB_WIN_GPUCC, 0x905c));
    CHECK(was_read(GPB_WIN_GPUCC, 0x90bc) && was_read(GPB_WIN_GPUCC, 0x7000) && was_read(GPB_WIN_GPUCC, 0x90d4));
    CHECK(!reads_in(GPB_WIN_SMMU) && !reads_in(GPB_WIN_GPU));
    CHECK(nstages == 2 && stages[0] == GPB_WIN_GCC_GPU && stages[1] == GPB_WIN_GPUCC);   // announced before each block

    // Mode 1 with cfg_ahb off: nothing in GPUCC is touched, still no writes.
    reset(); gpb_run(&io, 1, all, &r);
    CHECK(r.gcc == GPB_DONE && r.cc == GPB_SKIPPED_PREREQ && r.smmu == GPB_SKIPPED_PREREQ && r.gpu == GPB_SKIPPED_PREREQ);
    CHECK(!nwrites && !reads_in(GPB_WIN_GPUCC) && !reads_in(GPB_WIN_SMMU) && !reads_in(GPB_WIN_GPU));
    CHECK(nstages == 1 && stages[0] == GPB_WIN_GCC_GPU);

    // Mode 2 with cfg_ahb off: exactly one write, the enable bit on the value read, then GPUCC is read.
    reset(); mem[GPB_WIN_GCC_GPU][GPB_GCC_GPU_CFG_AHB / 4] = 0x80000002;
    gpb_run(&io, 2, all, &r);
    CHECK(nwrites == 1 && writes[0].window == GPB_WIN_GCC_GPU && writes[0].offset == GPB_GCC_GPU_CFG_AHB && r.writes == 1);
    CHECK(mem[GPB_WIN_GCC_GPU][GPB_GCC_GPU_CFG_AHB / 4] == 0x80000003 && r.enabled_ahb && r.cfg_ahb_after == 0x80000003);
    CHECK(r.cc == GPB_DONE && reads_in(GPB_WIN_GPUCC) == 6);
    // Mode 2 with cfg_ahb already on: no write.
    reset(); mem[GPB_WIN_GCC_GPU][GPB_GCC_GPU_CFG_AHB / 4] = 0x00000001;
    gpb_run(&io, 2, all, &r);
    CHECK(!nwrites && !r.enabled_ahb && r.cc == GPB_DONE);

    // CX on and the SMMU vote clock running: global space 0 is read; GX still off, so the GPU window is not.
    reset(); mem[GPB_WIN_GCC_GPU][GPB_GCC_GPU_CFG_AHB / 4] = 1; mem[GPB_WIN_GPUCC][GPB_CC_CX_GDSCR / 4] = GPB_GDSC_PWR_ON;
    mem[GPB_WIN_SMMU][GPB_SMMU_ID0 / 4] = 0x4c017e18;   // the phone's ID0: NUMSMRG 24
    gpb_run(&io, 1, all, &r);
    mem[GPB_WIN_SMMU][GPB_SMMU_SMR(2) / 4] = 0x80000005; mem[GPB_WIN_SMMU][GPB_SMMU_S2CR(2) / 4] = 0x00000001;
    nreads = 0; gpb_run(&io, 1, all, &r);
    // The stream-match survey: ID0's NUMSMRG (24 on the phone) entries, SMR and S2CR each, read only.
    CHECK(r.smmu == GPB_DONE && r.id0 == 0x4c017e18 && reads_in(GPB_WIN_SMMU) == 5 + 2 * 24 && r.gpu == GPB_SKIPPED_PREREQ);
    CHECK(r.num_smr == 24 && r.smr_valid == 1 && r.smr[2] == 0x80000005 && r.s2cr[2] == 1 && !nwrites);
    CHECK(was_read(GPB_WIN_SMMU, 0x800) && was_read(GPB_WIN_SMMU, 0x85c) && was_read(GPB_WIN_SMMU, 0xc5c) &&
          !was_read(GPB_WIN_SMMU, 0x860));
    mem[GPB_WIN_SMMU][GPB_SMMU_ID0 / 4] = 0xff; nreads = 0; gpb_run(&io, 1, all, &r);   // a large NUMSMRG is capped
    CHECK(r.num_smr == GPB_SMMU_MAX_SMR && reads_in(GPB_WIN_SMMU) == 5 + 2 * GPB_SMMU_MAX_SMR);
    mem[GPB_WIN_SMMU][GPB_SMMU_ID0 / 4] = 0x4c017e18; mem[GPB_WIN_SMMU][GPB_SMMU_SMR(2) / 4] = 0; nreads = 0;
    gpb_run(&io, 1, all, &r);
    // GX on as well: RBBM_STATUS at the gen8 offset.
    mem[GPB_WIN_GPUCC][GPB_CC_GX_GDSCR / 4] = GPB_GDSC_PWR_ON; mem[GPB_WIN_GPU][0x48 / 4] = 0x12345678; nreads = 0;
    gpb_run(&io, 1, all, &r);
    CHECK(r.gpu == GPB_DONE && r.rbbm_status == 0x12345678 && was_read(GPB_WIN_GPU, 0x48) && !nwrites);

    // Mode 3 from the value T2 read on the phone (0x0022f001): the downstream gdsc-regulator.c writes, in order, then the clocks.
    reset(); mem[GPB_WIN_GCC_GPU][GPB_GCC_GPU_CFG_AHB / 4] = 1; mem[GPB_WIN_GPUCC][GPB_CC_CX_GDSCR / 4] = 0x0022f001;
    mem[GPB_WIN_GPUCC][GPB_CC_SMMU_VOTE_CLK / 4] = GPB_CBCR_CLK_OFF; mem[GPB_WIN_SMMU][GPB_SMMU_ID0 / 4] = 0x4c017e18;
    cx_on_after = 3;
    gpb_run(&io, 3, all, &r);
    CHECK(r.cx_before == 0x0022f001 && r.cx_init == 0x00228001 && r.cx_after == 0x00228800 && !r.cx_refused);
    CHECK(nwrites == 5 && r.writes == 5);
    CHECK(writes[0].offset == GPB_CC_CX_GDSCR && writes[0].value == 0x00228001);   // probe init: clk-dis-wait 8
    CHECK(writes[1].offset == GPB_CC_CX_GDSCR && writes[1].value == 0x00228000);   // enable: SW_COLLAPSE cleared
    CHECK(writes[2].offset == GPB_CC_CX_GDSCR && writes[2].value == 0x00228800);   // RETAIN_FF after PWR_ON
    CHECK(writes[3].offset == GPB_CC_AHB_CLK && (writes[3].value & GPB_CBCR_ENABLE));
    CHECK(writes[4].offset == GPB_CC_SMMU_VOTE_CLK && (writes[4].value & GPB_CBCR_ENABLE));
    for (unsigned i = 0; i < nwrites; ++i) CHECK(writes[i].window == GPB_WIN_GPUCC && writes[i].offset != GPB_CC_GX_GDSCR);
    CHECK(r.cx_tries == 4 && r.powered_cx && r.powered_clocks && gpb_gdsc_on(r.cx_hwctrl_after));
    {unsigned announced = 0; for (unsigned i = 0; i < nstages; ++i) announced += stages[i] == GPB_STAGE_CX_GDSC; CHECK(announced == 1);}
    CHECK(r.smmu == GPB_DONE && r.id0 == 0x4c017e18 && r.gpu == GPB_SKIPPED_PREREQ && !r.io_error);
    CHECK(delayed >= 1 + 3 + 2 + GPB_BRANCH_HALT_DELAY_US);   // 1 us before polling, 1 us per poll, 2 us after, halt delay
    // The exact value T2 read on the phone, 0x0022f801: RETAIN_FF (bit 11) is already set, so there are two GDSC writes, no third.
    reset(); mem[GPB_WIN_GCC_GPU][GPB_GCC_GPU_CFG_AHB / 4] = 1; mem[GPB_WIN_GPUCC][GPB_CC_CX_GDSCR / 4] = 0x0022f801;
    mem[GPB_WIN_GPUCC][GPB_CC_CX_GDS_HW_CTRL / 4] = 0x2800010a; mem[GPB_WIN_GPUCC][GPB_CC_SMMU_VOTE_CLK / 4] = 0x80000000;
    cx_on_after = 0;
    gpb_run(&io, 3, all, &r);
    CHECK(r.cx_before == 0x0022f801 && r.cx_init == 0x00228801 && r.cx_after == 0x00228800);
    CHECK(nwrites == 4 && writes[0].value == 0x00228801 && writes[1].value == 0x00228800);
    CHECK(writes[2].offset == GPB_CC_AHB_CLK && writes[3].offset == GPB_CC_SMMU_VOTE_CLK);
    CHECK(r.powered_cx && r.powered_clocks && r.smmu == GPB_DONE);
    // Under hardware control: refused, nothing written.
    reset(); mem[GPB_WIN_GCC_GPU][GPB_GCC_GPU_CFG_AHB / 4] = 1; mem[GPB_WIN_GPUCC][GPB_CC_CX_GDSCR / 4] = 0x0022f003;
    gpb_run(&io, 3, all, &r);
    CHECK(r.cx_refused && !nwrites && !r.powered_cx && r.smmu == GPB_SKIPPED_PREREQ);
    // CX already on (GDSCR or hw-ctrl): mode 3 writes nothing.
    reset(); mem[GPB_WIN_GCC_GPU][GPB_GCC_GPU_CFG_AHB / 4] = 1; mem[GPB_WIN_GPUCC][GPB_CC_CX_GDS_HW_CTRL / 4] = GPB_GDSC_PWR_ON;
    gpb_run(&io, 3, all, &r);
    CHECK(!nwrites && !r.powered_cx);
    // PWR_ON never comes: 1500 tries, then no RETAIN_FF and no clock writes; the SMMU stays unread.
    reset(); mem[GPB_WIN_GCC_GPU][GPB_GCC_GPU_CFG_AHB / 4] = 1; mem[GPB_WIN_GPUCC][GPB_CC_CX_GDSCR / 4] = 0x0022f001;
    gpb_run(&io, 3, all, &r);
    CHECK(nwrites == 2 && !r.powered_cx && !r.powered_clocks && r.cx_tries == GPB_GDSC_POLL_TRIES + 1);
    CHECK(r.smmu == GPB_SKIPPED_PREREQ && !reads_in(GPB_WIN_SMMU));
    // Modes 1 and 2 never run the mode-3 writes on a collapsed CX.
    for (unsigned m = 1; m <= 2; ++m) {
        reset(); mem[GPB_WIN_GCC_GPU][GPB_GCC_GPU_CFG_AHB / 4] = 1; mem[GPB_WIN_GPUCC][GPB_CC_CX_GDSCR / 4] = 0x0022f001;
        gpb_run(&io, m, all, &r);
        CHECK(!nwrites && !r.powered_cx);
    }

    // Mode 4 runs exactly what mode 3 does in gpb_run (the pool test is separate).
    reset(); mem[GPB_WIN_GCC_GPU][GPB_GCC_GPU_CFG_AHB / 4] = 1; mem[GPB_WIN_GPUCC][GPB_CC_CX_GDSCR / 4] = 0x0022f801;
    cx_on_after = 0; gpb_run(&io, 4, all, &r);
    CHECK(r.mode == 4 && r.powered_cx && nwrites == 4);
    // The pool round trip: a clean pool, then one stuck bit, then a word that will not clear.
    {
        static uint64_t pool[4096];
        struct gpb_pool_result p;
        gpb_pool_test(pool, sizeof pool, &p);
        CHECK(p.words == 4096 && !p.pattern_bad && !p.zero_bad && p.first_bad == ~0ULL);
        for (unsigned i = 0; i < 4096; ++i) CHECK(pool[i] == 0);
        CHECK(gpb_pool_pattern(0) != 0 && gpb_pool_pattern(1) != gpb_pool_pattern(0));
        CHECK(GPB_GMU_POOL_BASE == 0xE6440000ULL && GPB_GMU_POOL_BYTES == 0x02000000ULL && !(GPB_GMU_POOL_BASE & 0xfff));
    }

    // Mode 5's gate, from the T5b survey (24 SMRs invalid, S2CR FAULT) and a clean pool round trip.
    {
        CHECK(gpb_smmu_cb_offset(0x40000016, 1) == 0x21000 && gpb_smmu_cb_offset(0x40000016, 0) == 0x20000);
        CHECK(gpb_smmu_cb_offset(0xc0000016, 1) == 0x210000);   // 64 KiB pages
        struct gpb_result g;
        __builtin_memset(&g, 0, sizeof g);
        g.smmu = GPB_DONE; g.cx_gdscr = 0xf8228800; g.cc_smmu = 0x00000001; g.id1 = 0x40000016; g.num_smr = 24;
        for (unsigned i = 0; i < 24; ++i) g.s2cr[i] = 0x00020000;
        struct gpb_pool_result p = {GPB_GMU_POOL_BYTES / 8, 0, 0, ~0ULL};
        CHECK(gpb_gmu_context_gate(&g, &p) == 0 && !__builtin_strcmp(gpb_gmu_gate_text(0), "ok"));
        struct gpb_result b = g; b.io_error = 1; CHECK(gpb_gmu_context_gate(&b, &p) == 1);
        b = g; b.smmu = GPB_SKIPPED_PREREQ; CHECK(gpb_gmu_context_gate(&b, &p) == 1);
        b = g; b.cx_gdscr = 0x0022f801; CHECK(gpb_gmu_context_gate(&b, &p) == 2);
        b = g; b.cx_gdscr = 0x0022f801; b.cx_hwctrl = 0x80000202; CHECK(gpb_gmu_context_gate(&b, &p) == 0);
        b = g; b.cc_smmu = 0x80000000; CHECK(gpb_gmu_context_gate(&b, &p) == 2);
        b = g; b.smr_valid = 1; CHECK(gpb_gmu_context_gate(&b, &p) == 3);
        b = g; b.smr[2] = 0x5; CHECK(gpb_gmu_context_gate(&b, &p) == 4);
        b = g; b.s2cr[2] = 0x00010000; CHECK(gpb_gmu_context_gate(&b, &p) == 4);
        b = g; b.num_smr = 2; CHECK(gpb_gmu_context_gate(&b, &p) == 4);
        b = g; b.id1 = 0x40000001; CHECK(gpb_gmu_context_gate(&b, &p) == 5);   // one context bank only
        b = g; b.id1 = 0xc0000016; CHECK(gpb_gmu_context_gate(&b, &p) == 5);   // 64 KiB pages: bank 1 outside 0x40000
        CHECK(gpb_gmu_context_gate(&g, 0) == 6);
        struct gpb_pool_result q = p; q.words = 4096; CHECK(gpb_gmu_context_gate(&g, &q) == 6);
        q = p; q.zero_bad = 1; CHECK(gpb_gmu_context_gate(&g, &q) == 7);
        q = p; q.pattern_bad = 3; CHECK(gpb_gmu_context_gate(&g, &q) == 7);
        CHECK(GPB_GMU_BANK == 1 && GPB_GMU_ASID == 1 && GPB_GMU_SLOT == 2 && GPB_GMU_SID == 5 && GPB_SMMU_WINDOW == 0x40000);
    }

    // Mode 6's clock snapshot: reads only, the gpucc-milos.c offsets, inside the GPUCC window, and only after stage 2.
    {
        CHECK(GPB_CC_GMU_RCGR == 0x9318 && GPB_CC_HUB_RCGR == 0x93ec && GPB_CC_HUB_DIV == 0x942c && GPB_CC_FF_RCGR == 0x9474);
        CHECK(gpb_gmu_branch[0] == GPB_CC_CX_GMU_CLK && gpb_gmu_branch[2] == GPB_CC_AHB_CLK);
        reset(); mem[GPB_WIN_GCC_GPU][GPB_GCC_GPU_CFG_AHB / 4] = 0x00000001; mem[GPB_WIN_GPUCC][GPB_CC_CX_GDSCR / 4] = 0x0022f801;
        mem[GPB_WIN_GPUCC][GPB_CC_GMU_RCGR / 4] = 0x80000000; mem[GPB_WIN_GPUCC][GPB_CC_CX_GMU_CLK / 4] = 0x80002001;
        mem[GPB_WIN_GPUCC][0x10 / 4] = 0x24;
        gpb_run(&io, 1, all, &r);
        const unsigned before = nreads;
        struct gpb_gmu_clocks k;
        gpb_read_gmu_clocks(&io, &r, &k);
        CHECK(k.done && !nwrites && !r.io_error && nreads - before == 6 + 7 + GPB_GMU_BRANCHES + GPB_CRITICAL_BRANCHES + 3);
        CHECK(k.pll0_l == 0x24 && k.gmu_cmd == 0x80000000 && k.branch[0] == 0x80002001 && gpb_branch_digit(k.branch[0]) == 3);
        for (unsigned i = before; i < nreads; ++i)
            CHECK(reads[i].window == GPB_WIN_GPUCC || reads[i].window == GPB_WIN_GCC_GPU || reads[i].window == GPB_WIN_GCC_VOTE);
        CHECK(was_read(GPB_WIN_GPUCC, 0x9318) && was_read(GPB_WIN_GPUCC, 0x931c) && was_read(GPB_WIN_GPUCC, 0x93ec) &&
              was_read(GPB_WIN_GPUCC, 0x942c) && was_read(GPB_WIN_GPUCC, 0x93a4) && was_read(GPB_WIN_GPUCC, 0x90cc));
        CHECK(gpb_branch_digit(0x00000001) == 1 && gpb_branch_digit(0x80000000) == 2);
        // Not read without stage 2 (GPUCC unmapped).
        reset(); mem[GPB_WIN_GCC_GPU][GPB_GCC_GPU_CFG_AHB / 4] = 0x00000001;
        gpb_run(&io, 1, all & ~(1u << GPB_WIN_GPUCC), &r);
        const unsigned skipped = nreads;
        gpb_read_gmu_clocks(&io, &r, &k);
        CHECK(!k.done && nreads == skipped);
    }

    // Mode 7's bring-up from T8's phone state: the exact write sequence, all of it allowed by gpb_mode7_writable.
    {
        struct gpb_gmu_clocks k; struct gpb_gmu_clock_on o;
#define T8_STATE() do { reset(); hw7 = 1; \
        mem[GPB_WIN_GCC_GPU][GPB_GCC_GPU_CFG_AHB / 4] = 1; mem[GPB_WIN_GPUCC][GPB_CC_CX_GDSCR / 4] = 0x00228800; \
        mem[GPB_WIN_GPUCC][GPB_CC_CX_GDS_HW_CTRL / 4] = GPB_GDSC_PWR_ON; mem[GPB_WIN_GCC_VOTE][0] = 0x0060257f; \
        mem[GPB_WIN_GPUCC][0] = 0x100; mem[GPB_WIN_GPUCC][0x18 / 4] = 1; \
        mem[GPB_WIN_GPUCC][GPB_CC_GMU_RCGR / 4 + 1] = 0x00100000; mem[GPB_WIN_GPUCC][GPB_CC_HUB_RCGR / 4 + 1] = 0x00100000; \
        mem[GPB_WIN_GPUCC][0x93a4 / 4] = 1; mem[GPB_WIN_GPUCC][0x9008 / 4] = 0; mem[GPB_WIN_GPUCC][0x9010 / 4] = 0; \
        mem[GPB_WIN_GPUCC][0x9064 / 4] = 1; mem[GPB_WIN_GPUCC][0x93a8 / 4] = 1; mem[GPB_WIN_GPUCC][0x9004 / 4] = 1; \
        mem[GPB_WIN_GPUCC][0x90cc / 4] = 1; \
        gpb_run(&io, 1, all, &r); gpb_read_gmu_clocks(&io, &r, &k); nwrites = 0; delayed = 0; } while (0)
        T8_STATE();
        CHECK(k.done && r.cc == GPB_DONE);
        gpb_gmu_clocks_on(&io, &r, &k, &o);
        CHECK(!o.failed && o.step == 8 && !o.pll_was_running && o.writes == nwrites);
        static const uint32_t want[][3] = {
            {GPB_WIN_GCC_VOTE, 0, 0x0060257f | (3u << 15)},
            {GPB_WIN_GPUCC, 0x9008, 1}, {GPB_WIN_GPUCC, 0x9010, 1},
            {GPB_WIN_GPUCC, 0x10, 0x44440024}, {GPB_WIN_GPUCC, 0x14, 0x7555}, {GPB_WIN_GPUCC, 0x20, 0x20485699},
            {GPB_WIN_GPUCC, 0x24, 0x00182261}, {GPB_WIN_GPUCC, 0x28, 0x82aa299c}, {GPB_WIN_GPUCC, 0x18, 0x400},
            {GPB_WIN_GPUCC, 0x1c, 5}, {GPB_WIN_GPUCC, 0x30, 3}, {GPB_WIN_GPUCC, 0x34, 0x9000}, {GPB_WIN_GPUCC, 0x38, 0x34},
            {GPB_WIN_GPUCC, 0x0, 0x100}, {GPB_WIN_GPUCC, 0x4, 0}, {GPB_WIN_GPUCC, 0x0, 0x104},
            {GPB_WIN_GPUCC, 0x0, 0x104}, {GPB_WIN_GPUCC, 0x4, 1}, {GPB_WIN_GPUCC, 0x18, 0x407},
            {GPB_WIN_GPUCC, 0x0, 0x80000105},
            {GPB_WIN_GPUCC, 0x93ec, 2}, {GPB_WIN_GPUCC, 0x93f0, 0x505}, {GPB_WIN_GPUCC, 0x93ec, 3}, {GPB_WIN_GPUCC, 0x93ec, 0},
            {GPB_WIN_GPUCC, 0x9318, 2}, {GPB_WIN_GPUCC, 0x931c, 0x201}, {GPB_WIN_GPUCC, 0x9318, 3}, {GPB_WIN_GPUCC, 0x9318, 0},
            {GPB_WIN_GPUCC, 0x90bc, 1},
        };
        const unsigned nwant = sizeof want / sizeof want[0];
        CHECK(nwrites == nwant);
        for (unsigned i = 0; i < nwant; ++i) {
            CHECK(writes[i].window == want[i][0] && writes[i].offset == want[i][1]);
            if (writes[i].value != want[i][2]) {fprintf(stderr, "write %u: %08x want %08x\n", i, writes[i].value, want[i][2]); exit(1);}
            CHECK(gpb_mode7_writable(writes[i].window, writes[i].offset));
        }
        CHECK(delayed == 2 * GPB_BRANCH_HALT_DELAY_US && o.hub_cfg == 0x505 && o.gmu_cfg == 0x201 && o.ahb == 1 &&
              (o.pll0_mode & GPB_PLL_LOCK_DET) && o.pll0_opmode == 1 && (o.pll0_user & 7) == 7);
        // The allow-list refuses everything else in the windows mode 7 touches.
        CHECK(!gpb_mode7_writable(GPB_WIN_GPUCC, GPB_CC_CX_GDSCR) && !gpb_mode7_writable(GPB_WIN_GPUCC, 0x3c) &&
              !gpb_mode7_writable(GPB_WIN_GPUCC, GPB_CC_HUB_DIV) && !gpb_mode7_writable(GPB_WIN_GPUCC, GPB_CC_FF_RCGR) &&
              !gpb_mode7_writable(GPB_WIN_GCC_VOTE, 4) && !gpb_mode7_writable(GPB_WIN_GCC_GPU, 0) &&
              !gpb_mode7_writable(GPB_WIN_SMMU, 0) && !gpb_mode7_writable(GPB_WIN_GPU, 0));
        // A PLL that never locks: stops at step 4 after 1500 polls, no RCG or branch write.
        T8_STATE(); pll_locks = 0; gpb_gmu_clocks_on(&io, &r, &k, &o);
        CHECK(o.failed == 2 && o.step == 4 && o.lock_polls == GPB_PLL_LOCK_TRIES && nwrites == 18);
        // An RCG update that never clears: stops at step 5.
        T8_STATE(); rcg_updates = 0; gpb_gmu_clocks_on(&io, &r, &k, &o);
        CHECK(o.failed == 2 && o.step == 5 && o.hub_root_polls == 0 && o.hub_polls == GPB_RCG_TRIES && nwrites == 23);
        // A PLL already running is left alone: no configure and no enable writes.
        T8_STATE(); mem[GPB_WIN_GPUCC][0] = 0x80000107; mem[GPB_WIN_GPUCC][1] = 1;
        gpb_gmu_clocks_on(&io, &r, &k, &o);
        CHECK(!o.failed && o.pll_was_running && nwrites == 3 + 8 + 1);
        for (unsigned i = 0; i < nwrites; ++i) CHECK(writes[i].offset > 0x38 || writes[i].window != GPB_WIN_GPUCC);
        // An FSM-voted PLL is refused before it is enabled.
        T8_STATE(); mem[GPB_WIN_GPUCC][0x18 / 4] = GPB_PLL_VOTE_RUN; gpb_gmu_clocks_on(&io, &r, &k, &o);
        CHECK(o.failed == 1 && o.step == 3 && nwrites == 3);
        // Refused with nothing written: no snapshot, CX off, an I/O error, or a root reporting off.
        T8_STATE(); k.done = 0; gpb_gmu_clocks_on(&io, &r, &k, &o); CHECK(o.failed == 1 && !nwrites);
        T8_STATE(); r.cx_gdscr = r.cx_hwctrl = 0; gpb_gmu_clocks_on(&io, &r, &k, &o); CHECK(o.failed == 1 && !nwrites);
        T8_STATE(); r.io_error = 1; gpb_gmu_clocks_on(&io, &r, &k, &o); CHECK(o.failed == 1 && !nwrites);
        T8_STATE(); k.gmu_cmd = GPB_RCG_ROOT_OFF; gpb_gmu_clocks_on(&io, &r, &k, &o); CHECK(o.failed == 1 && !nwrites);
#undef T8_STATE
    }

    // Mode 8's GMU state read: reads only, inside the two new windows, offsets as ShengGmuSource.h has them.
    {
        CHECK(GPB_GMU_CM3_SYSRESET == 0x47000 && GPB_GMU_HFI_CTRL_STATUS == 0x47600 && GPB_GMU_AO_COUNTER_L == 0x47100 &&
              GPB_GMU_AO_HOST_INTERRUPT_STATUS == 0x57c14 && GPB_GMU_DTCM_VERSION == 0x3dfe0 && GPB_RSCC_SEQ_BUSY_DRV0 == 0x404);
#ifdef GPB_HAVE_SHENG_SOURCE
#define SAME(mine, word) CHECK((mine) == (word) * 4u - SHENG_GMU_CORE_GPU_DELTA)
        CHECK(GPB_GMU_DELTA == SHENG_GMU_CORE_GPU_DELTA && GPB_GMU_BASE == sheng_gmu_window_physical[0] &&
              GPB_GMU_SIZE == sheng_gmu_window_bytes[0] && GPB_RSCC_BASE == sheng_gmu_window_physical[1] &&
              GPB_RSCC_SIZE == sheng_gmu_window_bytes[1] && GPB_RSCC_SEQ_BUSY_DRV0 == REG_RSCC_SEQ_BUSY_DRV0 * 4u &&
              GPB_GMU_AO_FATAL == SHENG_GMU_AO_FATAL_MASK && GPB_GMU_CM3_FATAL == SHENG_GMU_CM3_FATAL_MASK);
        SAME(GPB_GMU_CM3_SYSRESET, REG_GMU_CM3_SYSRESET); SAME(GPB_GMU_CM3_BOOT_CONFIG, REG_GMU_CM3_BOOT_CONFIG);
        SAME(GPB_GMU_CM3_FW_INIT_RESULT, REG_GMU_CM3_FW_INIT_RESULT); SAME(GPB_GMU_CM3_CFG, REG_GMU_CM3_CFG);
        SAME(GPB_GMU_HFI_CTRL_STATUS, REG_GMU_HFI_CTRL_STATUS); SAME(GPB_GMU_GMU2HOST_INTR_INFO, REG_GMU_GMU2HOST_INTR_INFO);
        SAME(GPB_GMU_GENERAL_8, REG_GMU_GENERAL_8); SAME(GPB_GMU_GENERAL_9, REG_GMU_GENERAL_9);
        SAME(GPB_GMU_GENERAL_10, REG_GMU_GENERAL_10); SAME(GPB_GMU_AHB_FENCE_RANGE_0, REG_GMU_AHB_FENCE_RANGE_0);
        SAME(GPB_GMU_AO_HOST_INTERRUPT_STATUS, REG_GMU_AO_HOST_INTERRUPT_STATUS); SAME(GPB_GMU_ITCM, REG_GMU_CM3_ITCM_START);
        SAME(GPB_GMU_DTCM_VERSION, REG_GMU_CM3_DTCM_START + 0xff8);
#undef SAME
#endif
        CHECK(GPB_STAGE_CX_GDSC >= GPB_ALL_WINDOWS);   // the CX power-up marker names no window (T10a logged it for the GMU)
        const struct gpb_io io8 = {mem, rd8, wr8, stage, delay};
        struct gpb_gmu_state g;
        reset(); ngmu_vals = 0; gmu_fail_at = 99; ao_tick = 0;
        gpb_read_gmu_state(&io8, &r, 0, &g);
        CHECK(!g.done && g.gate == 1 && !nreads);
        gmu_set(GPB_WIN_GMU, GPB_GMU_CM3_SYSRESET, 1);
        reset(); gpb_read_gmu_state(&io8, &r, 1, &g);
        CHECK(g.done && g.gate == 0 && !nwrites && nreads == GPB_GMU_STATE_READS + 1 && delayed == 10 && nstages == 2 &&
              stages[0] == GPB_WIN_GMU && stages[1] == GPB_WIN_RSCC && reads[0].offset == GPB_GMU_CM3_SYSRESET &&
              reads[nreads - 1].window == GPB_WIN_RSCC && g.v[15] - g.v[13] == 192);
        const struct {unsigned w; uint32_t off, v; unsigned gate;} bad[] = {
            {GPB_WIN_GMU, GPB_GMU_CM3_SYSRESET, 0, 3}, {GPB_WIN_RSCC, GPB_RSCC_SEQ_BUSY_DRV0, 1, 4},
            {GPB_WIN_GMU, GPB_GMU_HFI_CTRL_STATUS, 1, 5}, {GPB_WIN_GMU, GPB_GMU_AO_HOST_INTERRUPT_STATUS, 0x20, 6},
            {GPB_WIN_GMU, GPB_GMU_GMU2HOST_INTR_INFO, 0x800000, 7}};
        for (unsigned i = 0; i < 5; ++i) {
            ngmu_vals = 1; gmu_set(bad[i].w, bad[i].off, bad[i].v);
            if (bad[i].gate == 3) ngmu_vals = 0;
            reset(); gpb_read_gmu_state(&io8, &r, 1, &g);
            CHECK(g.gate == bad[i].gate && !strcmp(gpb_gmu_state_text(g.gate), gpb_gmu_state_text(bad[i].gate)));
        }
        ngmu_vals = 1; reset(); gmu_fail_at = 0; gpb_read_gmu_state(&io8, &r, 1, &g);
        CHECK(g.gate == 2 && g.io_error && nreads == 1);
        gmu_fail_at = 99;
    }

    // Mode 11's GX request: the writes, the vote, the OOB handshake and the RBBM reads only once GX is on.
    {
        CHECK(GPB_GMU_PWR_CLK_STATUS == 0x46fa0 && GPB_GMU_GMU2HOST_INTR_CLR == 0x47644 &&
              GPB_GMU_GMU2HOST_INTR_MASK == 0x4764c && GPB_GMU_HOST2GMU_INTR_SET == 0x47650 && GPB_RBBM_STATUS == 0x48);
#ifdef GPB_HAVE_SHENG_SOURCE
        CHECK(GPB_GMU_HOST2GMU_INTR_SET == REG_GMU_HOST2GMU_INTR_SET * 4u - SHENG_GMU_CORE_GPU_DELTA &&
              GPB_GMU_GMU2HOST_INTR_INFO == REG_GMU_GMU2HOST_INTR_INFO * 4u - SHENG_GMU_CORE_GPU_DELTA);
#endif
        const struct gpb_gx_io gio = {mem, gx_rd, gx_wr, gx_vote, gx_delay};
        struct gpb_gx x;
        gx_reset(); gpb_gx_on(&gio, 1, 1, &x);
        CHECK(!x.failed && x.step == 6 && x.gx_before == 3 && x.gx_after == 0 && x.gx_later == 0 && x.oob_polls == 1);
        CHECK(gx_votes == 1 && gx_vote_perf == 1 && gx_vote_bw == 1 && x.rbbm == 0x1048 && x.gfx == 0x1054 &&
              x.gfx_br == 0x1068 && x.gbif_halt == 0x1028 && gx_delayed == 100000);
        CHECK(ngx_writes == 4 && x.writes == 4 &&
              gx_writes[0].off == GPB_GMU_GMU2HOST_INTR_CLR && gx_writes[0].v == ~0u &&
              gx_writes[1].off == GPB_GMU_GMU2HOST_INTR_MASK && gx_writes[1].v == ~(1u << 23) &&
              gx_writes[2].off == GPB_GMU_HOST2GMU_INTR_SET && gx_writes[2].v == (1u << 30) &&
              gx_writes[3].off == GPB_GMU_GMU2HOST_INTR_CLR && gx_writes[3].v == (1u << 31));
        // A refused vote or a firmware error: no OOB request.
        gx_reset(); gx_vote_fail = 1; gpb_gx_on(&gio, 1, 1, &x); CHECK(x.failed == 1 && x.step == 3 && ngx_writes == 2);
        gx_reset(); gx_vote_fail = 2; gpb_gx_on(&gio, 1, 1, &x); CHECK(x.failed == 1 && x.fw_error == 7 && ngx_writes == 2);
        // No ack within 100 polls: no clear, no RBBM read; GX status still read for the log.
        gx_reset(); gx_ack = 0; gx_powers = 0; gpb_gx_on(&gio, 1, 1, &x);
        CHECK(x.failed == 2 && x.oob_polls == GPB_OOB_TRIES && ngx_writes == 3 && x.gx_after == 3 && !x.rbbm &&
              gx_delayed == 100 * (GPB_OOB_TRIES - 1));
        // Acked but GX stays off: no RBBM read.
        gx_reset(); gx_powers = 0; gpb_gx_on(&gio, 1, 1, &x); CHECK(x.failed == 4 && x.step == 5 && !x.rbbm);
        // A failed read stops it.
        gx_reset(); gx_fail_read = 0; gpb_gx_on(&gio, 1, 1, &x); CHECK(x.failed == 3 && x.step == 2 && !gx_votes);
    }

    {   // Mode 12: the survey reads CX_MISC first, then the GPU block, and writes nothing.
        const struct gpb_gx_io hio = {hw_gpu, hw_rd, hw_wr, gx_vote, gx_delay};
        struct gpb_survey v;
        CHECK(GPB_CX_SW_FUSE_VALUE == 0x1000 && GPB_CX_SW_FUSE_FREQ_LIMIT == 0x1014 && GPB_WIN_CX == GPB_ALL_WINDOWS);
        hw_reset(); hw_cx[0x400] = 0x7; hw_cx[0x405] = 0x55; hw_gpu[A810_RBBM_STATUS] = 1; hw_gpu[A810_CP_SQE_CNTL] = 9;
        gpb_gpu_survey(&hio, &v);
        CHECK(v.done && v.fuse == 7 && v.freq_limit == 0x55 && v.gpu[0] == 1 && v.gpu[6] == 9 && !nhw &&
              hw_ops == 2 + GPB_SURVEY_GPU && gpb_survey_gpu[7] == 0x816 && gpb_survey_gpu[12] == 0x81c);
        hw_reset(); hw_fail_at = 0; gpb_gpu_survey(&hio, &v); CHECK(!v.done && hw_ops == 1);
    }
    {   // Mode 13: a8xx hw_init's writes, in order, through the cached aperture.
        const struct gpb_gx_io hio = {hw_gpu, hw_rd, hw_wr, gx_vote, gx_delay};
        struct gpb_hw h;
        // The UBWC words from milos_data by a8xx_set_ubwc_config's formulas: hbb 15 - 13, swizzle LVL2 only.
        const uint32_t hbb = 15 - 13, lo = hbb & 3, hi = hbb >> 2, l2 = 0, l3 = 1, mode = 2;
        CHECK(GPB_UBWC_GRAS == (hbb << 5 | l3 << 4 | l2 << 3) &&
              GPB_UBWC_RB_CCU == (l3 << 5 | l2 << 4 | hi << 3 | lo << 1) &&
              GPB_UBWC_RB_CMP == (mode << 15 | 1u << 4 | 1u << 3 | 1u << 2 | 1u << 1) &&
              GPB_UBWC_SP == (l3 << 13 | l2 << 12 | hi << 10 | 2u << 4 | lo << 1) &&
              GPB_UBWC_TPL1 == (l3 << 7 | l2 << 6 | hi << 4 | lo << 1));
        hw_reset(); hw_gpu[A810_GBIF_HALT] = 1; hw_gmu[GPB_GMU_REG(A810_GMU_CX_GMU_POWER_COUNTER_SELECT_XOCLK_0) / 4] = 0xabcd12;
        gpb_hw_init(&hio, &h);
        CHECK(!h.failed && h.step == 9 && !h.mismatches && h.writes == nhw && !hw_cx_writes && !h.gbif_halt &&
              !h.rbbm_gbif_halt && h.protect_last == NATIVE_ADRENO_A810_PROTECT_LAST && h.protect_writes == 47);
        // The first write clears the aperture (cached_aperture starts at UINT_MAX); then GBIF_HALT and RBBM_GBIF_HALT.
        CHECK(hw_log[0].off == GPB_GPU_REG(A810_CP_APERTURE_CNTL_HOST) && hw_log[0].v == 0 &&
              hw_log[1].off == GPB_GPU_REG(A810_GBIF_HALT) && hw_log[2].off == GPB_GPU_REG(A810_RBBM_GBIF_HALT));
        // Never the CP start, the ring, the trust control or the SQE base: those are mode 14's (or later).
        CHECK(!hw_written(A810_CP_SQE_CNTL) && !hw_written(A810_CP_SQE_INSTR_BASE) && !hw_written(A810_CP_RB_BASE) &&
              !hw_written(A810_CP_RB_CNTL) && !hw_written(A810_CP_RB_WPTR) && !hw_written(A810_RBBM_SECVID_TRUST_CNTL) &&
              !hw_written(A810_CP_RB_RPTR_ADDR_BV));
        // 64-bit writes, low word first; the trap base in all four.
        CHECK(hw_gpu[A810_UCHE_TRAP_BASE] == 0xfffff000u && hw_gpu[A810_UCHE_TRAP_BASE + 1] == 0x1ffff &&
              hw_gpu[A810_UCHE_CCHE_WRITE_THRU_BASE + 1] == 0x1ffff && hw_gpu[A810_SP_HLSQ_GC_GMEM_RANGE_MIN] == 0x4000000 &&
              hw_gpu[A810_UCHE_CCHE_GC_GMEM_RANGE_MIN + 1] == 0);
        // UBWC: GRAS on BV then BR, RB words on BR, SP and TPL1 with the aperture cleared.
        CHECK(hw_count(GPB_WIN_GPU, A810_GRAS_NC_MODE_CNTL, 0x50, A810_PIPE_BV) == 1 &&
              hw_count(GPB_WIN_GPU, A810_GRAS_NC_MODE_CNTL, 0x50, A810_PIPE_BR) == 1 &&
              hw_count(GPB_WIN_GPU, A810_RB_CCU_NC_MODE_CNTL, 0x24, A810_PIPE_BR) == 1 &&
              hw_count(GPB_WIN_GPU, A810_RB_CMP_NC_MODE_CNTL, 0x1001e, A810_PIPE_BR) == 1 &&
              hw_count(GPB_WIN_GPU, A810_SP_NC_MODE_CNTL, 0x2024, A810_PIPE_NONE) == 1 &&
              hw_count(GPB_WIN_GPU, A810_TPL1_NC_MODE_CNTL, 0x84, A810_PIPE_NONE) == 1);
        // Non-context registers: PC_CHICKEN_BITS_1 differs per pipe; LPAC is skipped; GMEM_PROTECT on BR only.
        CHECK(hw_count(GPB_WIN_GPU, A810_PC_CHICKEN_BITS_1, 0x3, A810_PIPE_BR) == 1 &&
              hw_count(GPB_WIN_GPU, A810_PC_CHICKEN_BITS_1, 0x23, A810_PIPE_BV) == 1 &&
              hw_count(GPB_WIN_GPU, A810_CP_SMMU_STREAM_ID_LPAC, 0x101, A810_PIPE_NONE) == 1 &&
              hw_count(GPB_WIN_GPU, A810_RB_GC_GMEM_PROTECT, 0x00900000, A810_PIPE_BR) == 1 &&
              hw_count(GPB_WIN_GPU, A810_RB_GC_GMEM_PROTECT, ~0u, A810_PIPE_NONE) == 0);
        unsigned nonctxt = 0;
        for (unsigned i = 0; i < NATIVE_ADRENO_A810_NONCTXT_COUNT; ++i)
            nonctxt += __builtin_popcount(NativeAdrenoA810NonCtxt[i][2] & 0xf7u);
        CHECK(h.nonctxt_writes == nonctxt && nonctxt == 52);
        // Hang detection, the GMU counters (XOCLK_0 keeps its upper bits) and the protect list.
        CHECK(hw_gpu[A810_RBBM_INTERFACE_HANG_INT_CNTL] == 0x40cfffff && hw_gpu[A810_RBBM_SLICE_INTERFACE_HANG_INT_CNTL] == 1u << 30 &&
              hw_gmu[GPB_GMU_REG(A810_GMU_CX_GMU_POWER_COUNTER_SELECT_XOCLK_0) / 4] == 0xabcd20 &&
              hw_gmu[GPB_GMU_REG(A810_GMU_CX_GMU_POWER_COUNTER_SELECT_XOCLK_1) / 4] == 4 &&
              hw_gmu[GPB_GMU_REG(A810_GMU_CX_GMU_POWER_COUNTER_ENABLE) / 4] == 1 &&
              hw_gmu[GPB_GMU_REG(A810_GPU_GMU_AO_GPU_CX_BUSY_MASK) / 4] == 0xff000000u);
        CHECK(hw_count(GPB_WIN_GPU, A810_CP_PROTECT_CNTL_PIPE, 0xffff000b, A810_PIPE_BR) == 1 &&
              hw_count(GPB_WIN_GPU, A810_CP_PROTECT_CNTL_PIPE, 0xffff000b, A810_PIPE_BV) == 1 &&
              hw_count(GPB_WIN_GPU, A810_CP_PROTECT_PIPE(15), 0x80027c02, A810_PIPE_BR) == 1 &&
              hw_count(GPB_WIN_GPU, A810_CP_PROTECT_PIPE(15), 0x80027c02, A810_PIPE_BV) == 1 &&
              hw_count(GPB_WIN_GPU, A810_CP_PROTECT_GLOBAL(46), 0x80027c02, A810_PIPE_NONE) == 1 &&
              hw_count(GPB_WIN_GPU, A810_RB_CONTEXT_SWITCH_GMEM_SAVE_RESTORE_ENABLE, 1, A810_PIPE_BR) == 1);
        // APRIV and the CP masks on the six pipes, none on LPAC; the global masks last, with the aperture cleared.
        for (unsigned pipe = A810_PIPE_BR; pipe <= A810_PIPE_DDE_BV; ++pipe)
            CHECK(hw_count(GPB_WIN_GPU, A810_CP_APRIV_CNTL_PIPE, pipe == A810_PIPE_BR ? 0x6f : 0xf, pipe) ==
                      (pipe != A810_PIPE_LPAC) &&
                  hw_count(GPB_WIN_GPU, A810_CP_INTERRUPT_STATUS_MASK_PIPE, 0x07e83f7f, pipe) == (pipe != A810_PIPE_LPAC) &&
                  hw_count(GPB_WIN_GPU, A810_CP_HW_FAULT_STATUS_MASK_PIPE, 0x77f, pipe) == (pipe != A810_PIPE_LPAC));
        CHECK(hw_log[nhw - 1].off == GPB_GPU_REG(A810_RBBM_INT_0_MASK) && hw_log[nhw - 1].v == 0x33d283c2 &&
              hw_log[nhw - 1].aperture == 0 && hw_log[nhw - 2].off == GPB_GPU_REG(A810_CP_INTERRUPT_STATUS_MASK_GLOBAL) &&
              hw_gpu[A810_CP_APERTURE_CNTL_HOST] == 0);
        // The aperture is written only on a change: 24 times.
        unsigned apertures = 0;
        for (unsigned i = 0; i < nhw; ++i) apertures += hw_log[i].off == GPB_GPU_REG(A810_CP_APERTURE_CNTL_HOST);
        CHECK(apertures == h.aperture_writes && apertures == 24);
        const unsigned total = nhw;
        // Steps 1-8 without the aperture: 6, 17, 6, 20 + 1 + 52, 5, 2 + 47 + 2 + 1, 18, 2.
        CHECK(total == 6 + 17 + 6 + 21 + nonctxt + 5 + 52 + 18 + 2 + apertures);
        // A failed access stops it and says so.
        hw_reset(); hw_fail_at = 5; gpb_hw_init(&hio, &h); CHECK(h.failed == 3 && h.step == 1 && nhw == 3);   // op 5 is the TSB_CNTL write
    }
    // Mode 15's SCM encodings and the mbn split, on an image laid out as gen80300_zap.mbn is: three program headers (the
    // ELF headers as segment type 7, one relocatable PT_LOAD of 0x378 bytes at 0x1000, the hash segment at 0x2000).
    {
        CHECK(gpb_scm_fn(GPB_SCM_SVC_PIL, GPB_SCM_PAS_INIT_IMAGE) == 0x42000201u);
        CHECK(gpb_scm_fn(GPB_SCM_SVC_PIL, GPB_SCM_PAS_AUTH_AND_RESET) == 0x42000205u);
        CHECK(gpb_scm_fn(GPB_SCM_SVC_INFO, GPB_SCM_INFO_IS_CALL_AVAIL) == 0x42000601u);
        CHECK(gpb_scm_query(GPB_SCM_SVC_PIL, GPB_SCM_PAS_IS_SUPPORTED) == 0x02000207u);
        static uint8_t img[0x2f38], meta[0x1000], region[GPB_ZAP_REGION_BYTES];
        struct gpb_mdt m;
#define PUT32(at, v) do {uint32_t v_ = (v); img[(at)] = v_; img[(at) + 1] = v_ >> 8; img[(at) + 2] = v_ >> 16; img[(at) + 3] = v_ >> 24;} while (0)
#define PHDR(i, type, off, addr, file, mem, flags) do {const uint32_t b_ = 0x34 + 32 * (i); PUT32(b_, type); PUT32(b_ + 4, off); \
        PUT32(b_ + 8, addr); PUT32(b_ + 12, addr); PUT32(b_ + 16, file); PUT32(b_ + 20, mem); PUT32(b_ + 24, flags);} while (0)
        memset(img, 0, sizeof img);
        img[0] = 0x7f; img[1] = 'E'; img[2] = 'L'; img[3] = 'F'; img[4] = 1; img[5] = 1;
        PUT32(0x1c, 0x34); img[0x2a] = 32; img[0x2c] = 3;
        PHDR(0, 0, 0, 0, 0x94, 0, 0x07000000u);
        PHDR(1, 1, 0x1000, 0x1000, 0x378, 0x378, 0x08000007u);
        PHDR(2, 0, 0x2000, 0, 0xf38, 0xf38, 0x02000000u);
        for (uint32_t i = 0; i < 0x378; ++i) img[0x1000 + i] = (uint8_t)(i * 7 + 1);
        for (uint32_t i = 0; i < 0xf38; ++i) img[0x2000 + i] = (uint8_t)(i * 3 + 5);
        CHECK(gpb_mdt_parse(img, sizeof img, GPB_ZAP_REGION_BYTES, &m) == 0);
        CHECK(m.header_bytes == 0x94 && m.hash_offset == 0x2000 && m.hash_bytes == 0xf38 && m.meta_bytes == 0xfcc);
        CHECK(m.loads == 1 && m.relocatable && m.min_addr == 0x1000 && m.max_addr == 0x2000 && m.offset[0] == 0x1000);
        gpb_mdt_metadata(img, &m, meta);
        CHECK(!memcmp(meta, img, 0x94) && !memcmp(meta + 0x94, img + 0x2000, 0xf38));
        memset(region, 0xa5, sizeof region);
        gpb_mdt_place(img, &m, region);
        CHECK(!memcmp(region, img + 0x1000, 0x378) && region[0x378] == 0xa5);
        // A shorter file part is zero-filled up to the memory size.
        PHDR(1, 1, 0x1000, 0x1000, 0x300, 0x378, 0x08000007u);
        CHECK(gpb_mdt_parse(img, sizeof img, GPB_ZAP_REGION_BYTES, &m) == 0);
        gpb_mdt_place(img, &m, region);
        CHECK(!memcmp(region, img + 0x1000, 0x300) && region[0x300] == 0 && region[0x377] == 0 && region[0x378] == 0xa5);
        PHDR(1, 1, 0x1000, 0x1000, 0x378, 0x378, 0x08000007u);
        // Each rule refuses.
        CHECK(gpb_mdt_parse(img, 0x30, GPB_ZAP_REGION_BYTES, &m) == 1);
        img[4] = 2; CHECK(gpb_mdt_parse(img, sizeof img, GPB_ZAP_REGION_BYTES, &m) == 2); img[4] = 1;
        img[0x2c] = 200; CHECK(gpb_mdt_parse(img, sizeof img, GPB_ZAP_REGION_BYTES, &m) == 3); img[0x2c] = 3;
        CHECK(gpb_mdt_parse(img, 0x2e00, GPB_ZAP_REGION_BYTES, &m) == 4);
        PHDR(0, 0, 0, 0, 0x94, 0, 0x02000000u); CHECK(gpb_mdt_parse(img, sizeof img, GPB_ZAP_REGION_BYTES, &m) == 5);
        PHDR(0, 0, 0, 0, 0x94, 0, 0x07000000u);
        PHDR(1, 1, 0x1000, 0x1000, 0x400, 0x378, 0x08000007u); CHECK(gpb_mdt_parse(img, sizeof img, GPB_ZAP_REGION_BYTES, &m) == 6);
        PHDR(1, 1, 0x1000, 0x1000, 0x378, 0x378, 0x08000007u);
        PHDR(2, 0, 0x2000, 0, 0xf38, 0xf38, 0); CHECK(gpb_mdt_parse(img, sizeof img, GPB_ZAP_REGION_BYTES, &m) == 7);
        PHDR(2, 0, 0x2000, 0, 0xf38, 0xf38, 0x02000000u);
        PHDR(1, 0, 0x1000, 0x1000, 0x378, 0x378, 0x08000007u); CHECK(gpb_mdt_parse(img, sizeof img, GPB_ZAP_REGION_BYTES, &m) == 8);
        PHDR(1, 1, 0x1000, 0x1000, 0x378, 0x378, 0x08000007u);
        CHECK(gpb_mdt_parse(img, sizeof img, 0x300, &m) == 9);
#undef PHDR
#undef PUT32
    }
    // Mode 16's bar: the 3a console (1080 x 2392, 4320-byte rows) and the refusals.
    {
        struct gpb_bar b;
        CHECK(gpb_bar_plan(0xe1000000ULL, 1080, 2392, 4320, 32, &b) == 0);
        CHECK(b.y0 == 1594 && b.rows == 48 && b.page == ((0xe1000000ULL + 1594ULL * 4320) & ~0xfffULL));
        CHECK(b.offset == (0xe1000000ULL + 1594ULL * 4320) - b.page && !(b.page & 0xfff) && !(b.bytes & 0xfff));
        CHECK(b.page + b.bytes >= 0xe1000000ULL + 1641ULL * 4320 + 4320 && b.bytes - b.offset < 49ULL * 4320 + 0x1000);
        CHECK(b.ib_dwords == 48 * 1083 && b.ib_bytes == 0x33000);
        static uint32_t ib[48 * 1083];
        gpb_bar_fill(&b, 0x7abc0000u, GPB_FB_IOVA, ib);
        const uint64_t row5 = GPB_FB_IOVA + b.offset + 5 * 4320ULL;
        CHECK(ib[0] == 0x7abc0000u && ib[1] == (uint32_t)(GPB_FB_IOVA + b.offset) && ib[2] == 0);
        CHECK(ib[5 * 1083] == 0x7abc0000u && ib[5 * 1083 + 1] == (uint32_t)row5);
        CHECK(ib[3] == 0x00ffffffu && ib[5 * 1083 + 3] == 0x00ff0000u && ib[5 * 1083 + 3 + 539] == 0x0000ff00u);
        CHECK(ib[5 * 1083 + 3 + 1079] == 0x000000ffu && ib[47 * 1083 + 3 + 540] == 0x00ffffffu);
        CHECK(gpb_bar_pixel(&b, 359, 10) == 0x00ff0000u && gpb_bar_pixel(&b, 360, 10) == 0x0000ff00u &&
              gpb_bar_pixel(&b, 720, 10) == 0x000000ffu && gpb_bar_pixel(&b, 0, 44) == 0x00ffffffu);
        CHECK(gpb_bar_plan(0, 1080, 2392, 4320, 32, &b) == 1 && gpb_bar_plan(0xe1000000ULL, 1080, 2392, 4320, 16, &b) == 1);
        CHECK(gpb_bar_plan(0xe1000000ULL, 2, 2392, 4320, 32, &b) == 2 && gpb_bar_plan(0xe1000000ULL, 16382, 2392, 65536, 32, &b) == 2);
        CHECK(gpb_bar_plan(0xe1000000ULL, 1080, 100, 4320, 32, &b) == 3 && gpb_bar_plan(0xe1000000ULL, 1080, 2392, 4316, 32, &b) == 4);
        CHECK(gpb_bar_plan(0xe1000000ULL, 1080, 2392, 200000, 32, &b) == 5 && gpb_bar_plan(0xe1000000ULL, 8000, 2392, 32000, 32, &b) == 6);
    }
    CHECK(GPB_CLIENT_VA_START >= GPB_GMEM_RANGE_MIN + GPB_A810_GMEM_BYTES && GPB_CLIENT_VA_START >= GPB_FB_IOVA + GPB_FB_MAX &&
          GPB_CLIENT_VA_START + GPB_CLIENT_VA_BYTES <= 0x80000000ULL);
    // Modes 0 and 20+ run nothing; an unmapped GCC window reads nothing; a failed first read stops the sequence.
    for (unsigned m = 0; m < 23; m += (m == 0 ? 20 : 1)) {
        reset(); gpb_run(&io, m, all, &r);
        CHECK(!nreads && !nwrites && r.gcc == GPB_SKIPPED_MODE && r.gpu == GPB_SKIPPED_MODE);
    }
    reset(); gpb_run(&io, 1, all & ~(1u << GPB_WIN_GCC_GPU), &r);
    CHECK(!nreads && r.gcc == GPB_NOMAP && r.cc == GPB_NOMAP);
    reset(); fail_read_window = GPB_WIN_GCC_GPU; gpb_run(&io, 2, all, &r);
    CHECK(nreads == 1 && !nwrites && r.io_error && r.cc == GPB_SKIPPED_PREREQ);
    // An unmapped GPUCC window: stage 2 is nomap and later stages are skipped.
    reset(); mem[GPB_WIN_GCC_GPU][GPB_GCC_GPU_CFG_AHB / 4] = 1; gpb_run(&io, 1, all & ~(1u << GPB_WIN_GPUCC), &r);
    CHECK(r.cc == GPB_NOMAP && r.smmu == GPB_SKIPPED_PREREQ && !reads_in(GPB_WIN_GPUCC));
    printf("PASS: asteroids GPU power modes 1-3: exact reads per mode, mode 1 never writes, mode 2 only the cfg_ahb enable, "
           "mode 3 the downstream CX GDSC sequence (phone 0x0022f801 -> 0x00228801 -> 0x00228800; 0x0022f001 adds RETAIN_FF) and the two SMMU "
           "clocks, refusal under HW control, poll timeout, gating, window bounds, the pool round trip, the mode 5 gate (7 refusals), "
           "the mode 6 clock snapshot (reads only), the mode 7 clock bring-up (29 writes in Linux order, lock and update timeouts, "
           "a running PLL left alone, an FSM PLL and 4 preconditions refused), the mode 8 GMU state read (reads only, 5 gate refusals), the mode 11 GX request (4 GMU writes, vote, OOB, RBBM only with GX on; 5 failure cases), the mode 12 survey (reads only), mode 13's hw_init writes (Linux order, cached aperture, no CP start), mode 15's SCM encodings and mbn split (headers + hash as metadata, the relocatable load placed and zero-filled, 9 refusals), mode 16's bar plan and IB (6 refusals), mode 17's VA window bounds, modes 0 and 20+ refused (9-10 and 14-19 are the probe's)\n");
    return 0;
}
