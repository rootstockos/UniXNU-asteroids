// SPDX-License-Identifier: BSD-2-Clause-Patent
// Host test of GpuPowerBoard.h (asteroids, stages 1-2) over fake register windows: which registers each mode reads, that mode 1
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
static struct {unsigned window; uint32_t offset;} reads[64], writes[8];
static unsigned nreads, nwrites, fail_read_window = 99;
static int rd(void *cookie, unsigned w, uint32_t off, uint32_t *out) {
    CHECK(cookie == mem && w < GPB_WINDOWS && !(off & 3) && off + 4 <= sizes[w]);
    CHECK(nreads < 64); reads[nreads].window = w; reads[nreads].offset = off; ++nreads;
    if (w == fail_read_window) return 1;
    *out = mem[w][off / 4]; return 0;
}
static int wr(void *cookie, unsigned w, uint32_t off, uint32_t v) {
    CHECK(cookie == mem && w < GPB_WINDOWS && !(off & 3) && off + 4 <= sizes[w]);
    CHECK(nwrites < 8); writes[nwrites].window = w; writes[nwrites].offset = off; ++nwrites;
    mem[w][off / 4] = v; return 0;
}
static unsigned stages[8], nstages;
static void stage(void *cookie, unsigned w) {CHECK(cookie == mem && nstages < 8); stages[nstages++] = w;}
static const struct gpb_io io = {mem, rd, wr, stage};
static const unsigned all = (1u << GPB_WINDOWS) - 1;
static void reset(void) {memset(mem, 0, sizeof mem); nreads = nwrites = nstages = 0; fail_read_window = 99;}
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

    // Modes 0 and 3+ run nothing; an unmapped GCC window reads nothing; a failed first read stops the sequence.
    for (unsigned m = 0; m < 8; m += (m == 0 ? 3 : 1)) {
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
    printf("PASS: asteroids GPU power stages 1-2: exact reads per mode, mode 1 never writes, mode 2 writes only the cfg_ahb "
           "enable when it is off, gating, window bounds, modes 0 and 3+ refused\n");
    return 0;
}
