// SPDX-License-Identifier: BSD-2-Clause-Patent
// Host test of GpuPowerBoard.h (asteroids, modes 1-3) over fake register windows: which registers each mode reads, that mode 1
// never writes, that mode 2 writes only the gcc_gpu_cfg_ahb_clk enable and only when it is off, the gating, and that every access
// stays inside its window. Build: scripts/check-asteroids-gpu-power.py.
#include "GpuPowerBoard.h"
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

    // Modes 0 and 8+ run nothing; an unmapped GCC window reads nothing; a failed first read stops the sequence.
    for (unsigned m = 0; m < 10; m += (m == 0 ? 8 : 1)) {
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
           "a running PLL left alone, an FSM PLL and 4 preconditions refused), modes 0 and 8+ refused\n");
    return 0;
}
