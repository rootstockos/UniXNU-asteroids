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
static struct {unsigned window; uint32_t offset; uint32_t value;} reads[2048], writes[16];
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
    CHECK(nwrites < 16); writes[nwrites].window = w; writes[nwrites].offset = off; writes[nwrites].value = v; ++nwrites;
    mem[w][off / 4] = v;
    if (w == GPB_WIN_GPUCC && off == GPB_CC_SMMU_VOTE_CLK && (v & GPB_CBCR_ENABLE)) mem[w][off / 4] &= ~GPB_CBCR_CLK_OFF;
    return 0;
}
static unsigned stages[8], nstages;
static void stage(void *cookie, unsigned w) {CHECK(cookie == mem && nstages < 8); stages[nstages++] = w;}
static void delay(void *cookie, unsigned us) {CHECK(cookie == mem); delayed += us;}
static const struct gpb_io io = {mem, rd, wr, stage, delay};
static const unsigned all = (1u << GPB_WINDOWS) - 1;
static void reset(void) {memset(mem, 0, sizeof mem); nreads = nwrites = nstages = delayed = 0; fail_read_window = 99; cx_on_after = -1;}
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
    mem[GPB_WIN_SMMU][GPB_SMMU_ID0 / 4] = 0x00290406;
    gpb_run(&io, 1, all, &r);
    CHECK(r.smmu == GPB_DONE && r.id0 == 0x00290406 && reads_in(GPB_WIN_SMMU) == 5 && r.gpu == GPB_SKIPPED_PREREQ);
    // GX on as well: RBBM_STATUS at the gen8 offset.
    mem[GPB_WIN_GPUCC][GPB_CC_GX_GDSCR / 4] = GPB_GDSC_PWR_ON; mem[GPB_WIN_GPU][0x48 / 4] = 0x12345678; nreads = 0;
    gpb_run(&io, 1, all, &r);
    CHECK(r.gpu == GPB_DONE && r.rbbm_status == 0x12345678 && was_read(GPB_WIN_GPU, 0x48) && !nwrites);

    // Mode 3 from the value T2 read on the phone (0x0022f001): the downstream gdsc-regulator.c writes, in order, then the clocks.
    reset(); mem[GPB_WIN_GCC_GPU][GPB_GCC_GPU_CFG_AHB / 4] = 1; mem[GPB_WIN_GPUCC][GPB_CC_CX_GDSCR / 4] = 0x0022f001;
    mem[GPB_WIN_GPUCC][GPB_CC_SMMU_VOTE_CLK / 4] = GPB_CBCR_CLK_OFF; mem[GPB_WIN_SMMU][GPB_SMMU_ID0 / 4] = 0x00290406;
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
    CHECK(r.smmu == GPB_DONE && r.id0 == 0x00290406 && r.gpu == GPB_SKIPPED_PREREQ && !r.io_error);
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

    // Modes 0 and 4+ run nothing; an unmapped GCC window reads nothing; a failed first read stops the sequence.
    for (unsigned m = 0; m < 8; m += (m == 0 ? 4 : 1)) {
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
           "clocks, refusal under HW control, poll timeout, gating, window bounds, modes 0 and 4+ refused\n");
    return 0;
}
