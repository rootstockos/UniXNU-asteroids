// SPDX-License-Identifier: BSD-2-Clause-Patent
// asteroids (SM7635, volcano) Adreno 810 power/clock observation, stages 1 and 2 only. From UniXNU-sheng package/GpuPowerBoard.h
// and Allenwdk/UniXNU-uke's copy (BSD-2-Clause-Patent): the same staged, gated read sequence, with the 3a's facts. Every offset
// names its source. Nothing here touches hardware; GpuPowerProbe.cpp owns the MMIO. Host-tested by GpuPowerBoardTests.c.
//
// Why staged: the GPU SMMU, GPUCC and the GPU register window sit behind clocks and power domains that Darwin has no driver for.
// Reading a block whose clock or rail is off can hang the bus (a reset) instead of failing, so each block is read only when the
// previous one shows its prerequisite on. Mode 1 only reads; mode 2 adds the one write Linux's GCC probe makes. Powering CX (sheng
// and uke's mode 3) and everything after it are separate steps, each with its own facts and test.
#ifndef ASTEROIDS_GPU_POWER_BOARD_H
#define ASTEROIDS_GPU_POWER_BOARD_H
#include <stdint.h>

#define GPB_MAX_MODE 2u

// The 3a's live device tree (/sys/firmware/fdt): clock-controller@100000 qcom,volcano-gcc (size 0x1f4200),
// clock-controller@3d90000 qcom,volcano-gpucc (size 0x9800), qcom,kgsl-iommu@3da0000 (size 0x40000),
// qcom,kgsl-3d0@3d00000 kgsl_3d0_reg_memory (size 0x40000).
#define GPB_GCC_BASE 0x00100000ULL
#define GPB_GCC_GPU_PAGE 0x00171000ULL    // GCC_BASE + 0x71000: the GPU branches
#define GPB_GCC_VOTE_PAGE 0x00152000ULL   // GCC_BASE + 0x52000: the GPLL0 vote register
#define GPB_GPUCC_BASE 0x03d90000ULL
#define GPB_GPUCC_SIZE 0x9800ULL
#define GPB_SMMU_BASE 0x03da0000ULL
#define GPB_SMMU_GR0_SIZE 0x1000ULL       // global space 0 only
#define GPB_GPU_BASE 0x03d00000ULL
#define GPB_GPU_WINDOW 0x1000ULL          // one page holds RBBM_STATUS

// gcc-volcano.c (NothingOSS android_kernel_msm-6.1_nothing_sm7635 sm7635/v/mr, d5f5f965): the probe keeps gcc_gpu_cfg_ahb_clk
// always on with regmap_update_bits(regmap, 0x71004, BIT(0), BIT(0)) (as gcc-sm8550 does; it has no clk_branch). Branches:
// gcc_gpu_memnoc_gfx_clk 0x71010 (BRANCH_HALT_VOTED, hwcg bit 1), gcc_gpu_snoc_dvm_gfx_clk 0x71018 (BRANCH_HALT_DELAY),
// gcc_ddrss_gpu_axi_clk 0x7115c, gcc_gpu_gpll0_clk_src 0x52000 bit 15, gcc_gpu_gpll0_div_clk_src 0x52000 bit 16.
// Offsets below are relative to the mapped page.
#define GPB_GCC_GPU_CFG_AHB 0x004
#define GPB_GCC_GPU_MEMNOC_GFX 0x010
#define GPB_GCC_GPU_SNOC_DVM_GFX 0x018
#define GPB_GCC_DDRSS_GPU_AXI 0x15c
#define GPB_GCC_PLL_VOTE 0x000
#define GPB_GCC_GPLL0_GPU (1u << 15)
#define GPB_GCC_GPLL0_DIV_GPU (1u << 16)
#define GPB_CBCR_CLK_OFF (1u << 31)       // clk-branch.c: BRANCH_CLK_OFF; a running branch reads 0
#define GPB_CBCR_ENABLE (1u << 0)

// The live DT's GDSC nodes: qcom,gdsc reg 0x3d99080 (regulator-name gpu_cc_cx_gdsc; the node is named @3d99108, the reg is what
// the driver uses) with hw-ctrl-addr -> syscon reg 0x3d99094; qcom,gdsc reg 0x3d9905c (gpu_cc_gx_gdsc, status "disabled": on gen8
// the GMU owns GX). gpucc-volcano.c: gpu_cc_ahb_clk 0x90bc, gpu_cc_cx_gmu_clk 0x90d4, gpu_cc_hlos1_vote_gpu_smmu_clk 0x7000.
// gdsc.c: PWR_ON_MASK BIT(31), SW_COLLAPSE_MASK BIT(0). Offsets relative to GPUCC_BASE; stages 1-2 only read them.
#define GPB_CC_CX_GDSCR 0x9080
#define GPB_CC_CX_GDS_HW_CTRL 0x9094
#define GPB_CC_GX_GDSCR 0x905c
#define GPB_CC_AHB_CLK 0x90bc
#define GPB_CC_CX_GMU_CLK 0x90d4
#define GPB_CC_SMMU_VOTE_CLK 0x7000
#define GPB_GDSC_PWR_ON (1u << 31)
#define GPB_GDSC_SW_COLLAPSE (1u << 0)

// ARM SMMUv2 global space 0 (IHI 0062D.c): sCR0 0x0, ID0 0x20, ID1 0x24, ID2 0x28, sGFSR 0x48.
#define GPB_SMMU_SCR0 0x0
#define GPB_SMMU_ID0 0x20
#define GPB_SMMU_ID1 0x24
#define GPB_SMMU_ID2 0x28
#define GPB_SMMU_GFSR 0x48

// a6xx.xml (torvalds/linux 734bef548a96): RBBM_STATUS is dword 0x012 for A8XX- (0x210 is the A6XX-A7XX offset); NativeAdreno's
// generated NativeAdrenoA810HwInit.h has the same A810_RBBM_STATUS. Byte offset = dword * 4.
#define GPB_RBBM_STATUS_BYTES (0x012 * 4)

enum gpb_state {GPB_NOT_RUN = 0, GPB_DONE = 1, GPB_SKIPPED_PREREQ = 2, GPB_SKIPPED_MODE = 3, GPB_NOMAP = 4};
static inline const char *gpb_state_text(enum gpb_state s) {
    switch (s) {case GPB_DONE: return "ok"; case GPB_SKIPPED_PREREQ: return "skip"; case GPB_SKIPPED_MODE: return "off";
                case GPB_NOMAP: return "nomap"; default: return "-";}
}

struct gpb_io {
    void *cookie;
    int (*read32)(void *cookie, unsigned window, uint32_t offset, uint32_t *out);   // 0 on success
    int (*write32)(void *cookie, unsigned window, uint32_t offset, uint32_t value);
    // Optional: called before each block is first touched (GPB_WIN_*), so a probe can log where it is before a read that could
    // hang the bus. NULL in tests that do not need it.
    void (*stage)(void *cookie, unsigned window);
};
enum {GPB_WIN_GCC_GPU = 0, GPB_WIN_GCC_VOTE = 1, GPB_WIN_GPUCC = 2, GPB_WIN_SMMU = 3, GPB_WIN_GPU = 4, GPB_WINDOWS = 5};

struct gpb_result {
    unsigned mode;                       // 1 read-only; 2 additionally enable gcc_gpu_cfg_ahb_clk if it is off (as Linux does)
    enum gpb_state gcc, cc, smmu, gpu;
    uint32_t cfg_ahb, cfg_ahb_after, memnoc, dvm, axi, pll_vote;
    uint32_t cx_gdscr, cx_hwctrl, gx_gdscr, cc_ahb, cc_smmu, cc_gmu;
    uint32_t scr0, id0, id1, id2, gfsr;
    uint32_t rbbm_status;
    unsigned enabled_ahb;                // 1 if mode 2 wrote the enable bit
    unsigned writes;                     // every write attempted (mode 1: always 0)
    int io_error;
};

static inline int gpb_clock_on(uint32_t cbcr) {return (cbcr & GPB_CBCR_CLK_OFF) == 0;}
// gcc_gpu_cfg_ahb_clk: the enable bit is the test (an idle branch with hardware clock gating can read CLK_OFF while enabled).
static inline int gpb_cfg_ahb_enabled(uint32_t cbcr) {return (cbcr & GPB_CBCR_ENABLE) != 0;}
static inline int gpb_gdsc_on(uint32_t gdscr) {return (gdscr & GPB_GDSC_PWR_ON) != 0;}

static inline int gpb_rd(const struct gpb_io *io, struct gpb_result *r, unsigned window, uint32_t offset, uint32_t *out) {
    uint32_t v = 0;
    if (io->read32(io->cookie, window, offset, &v)) {r->io_error = 1; *out = 0; return -1;}
    *out = v;
    return 0;
}

// The gated sequence. `have` is a bitmask of mapped windows (1 << GPB_WIN_*); a missing window makes its stage GPB_NOMAP.
// A mode above GPB_MAX_MODE runs nothing.
static inline void gpb_run(const struct gpb_io *io, unsigned mode, unsigned have, struct gpb_result *r) {
    __builtin_memset(r, 0, sizeof *r);
    r->mode = mode;
    if (!mode || mode > GPB_MAX_MODE) {r->gcc = r->cc = r->smmu = r->gpu = GPB_SKIPPED_MODE; return;}
    if (!(have & (1u << GPB_WIN_GCC_GPU))) {r->gcc = r->cc = r->smmu = r->gpu = GPB_NOMAP; return;}
    // Stage 1: GCC, always clocked (the same block family the UFS bring-up reads).
    if (io->stage) io->stage(io->cookie, GPB_WIN_GCC_GPU);
    if (gpb_rd(io, r, GPB_WIN_GCC_GPU, GPB_GCC_GPU_CFG_AHB, &r->cfg_ahb)) {r->cc = r->smmu = r->gpu = GPB_SKIPPED_PREREQ; return;}
    gpb_rd(io, r, GPB_WIN_GCC_GPU, GPB_GCC_GPU_MEMNOC_GFX, &r->memnoc);
    gpb_rd(io, r, GPB_WIN_GCC_GPU, GPB_GCC_GPU_SNOC_DVM_GFX, &r->dvm);
    gpb_rd(io, r, GPB_WIN_GCC_GPU, GPB_GCC_DDRSS_GPU_AXI, &r->axi);
    if (have & (1u << GPB_WIN_GCC_VOTE)) gpb_rd(io, r, GPB_WIN_GCC_VOTE, GPB_GCC_PLL_VOTE, &r->pll_vote);
    r->cfg_ahb_after = r->cfg_ahb;
    if (!gpb_cfg_ahb_enabled(r->cfg_ahb) && mode >= 2) {
        // The single write gcc-volcano's probe makes; mode 1 never writes.
        ++r->writes;
        if (!io->write32(io->cookie, GPB_WIN_GCC_GPU, GPB_GCC_GPU_CFG_AHB, r->cfg_ahb | GPB_CBCR_ENABLE)) {
            r->enabled_ahb = 1;
            gpb_rd(io, r, GPB_WIN_GCC_GPU, GPB_GCC_GPU_CFG_AHB, &r->cfg_ahb_after);
        } else r->io_error = 1;
    }
    r->gcc = GPB_DONE;
    // Stage 2: GPUCC registers need gcc_gpu_cfg_ahb_clk.
    if (!(have & (1u << GPB_WIN_GPUCC))) r->cc = GPB_NOMAP;
    else if (!gpb_cfg_ahb_enabled(r->cfg_ahb_after)) r->cc = GPB_SKIPPED_PREREQ;
    else {
        if (io->stage) io->stage(io->cookie, GPB_WIN_GPUCC);
        gpb_rd(io, r, GPB_WIN_GPUCC, GPB_CC_CX_GDSCR, &r->cx_gdscr);
        gpb_rd(io, r, GPB_WIN_GPUCC, GPB_CC_CX_GDS_HW_CTRL, &r->cx_hwctrl);
        gpb_rd(io, r, GPB_WIN_GPUCC, GPB_CC_GX_GDSCR, &r->gx_gdscr);
        gpb_rd(io, r, GPB_WIN_GPUCC, GPB_CC_AHB_CLK, &r->cc_ahb);
        gpb_rd(io, r, GPB_WIN_GPUCC, GPB_CC_SMMU_VOTE_CLK, &r->cc_smmu);
        gpb_rd(io, r, GPB_WIN_GPUCC, GPB_CC_CX_GMU_CLK, &r->cc_gmu);
        r->cc = GPB_DONE;
    }
    // Stage 3: the GPU SMMU needs the CX domain and its vote clock (conservative, as sheng and uke gate it). Global space 0 only.
    if (!(have & (1u << GPB_WIN_SMMU))) r->smmu = GPB_NOMAP;
    else if (r->cc != GPB_DONE || !gpb_gdsc_on(r->cx_gdscr) || !gpb_clock_on(r->cc_smmu)) r->smmu = GPB_SKIPPED_PREREQ;
    else {
        if (io->stage) io->stage(io->cookie, GPB_WIN_SMMU);
        gpb_rd(io, r, GPB_WIN_SMMU, GPB_SMMU_SCR0, &r->scr0);
        gpb_rd(io, r, GPB_WIN_SMMU, GPB_SMMU_ID0, &r->id0);
        gpb_rd(io, r, GPB_WIN_SMMU, GPB_SMMU_ID1, &r->id1);
        gpb_rd(io, r, GPB_WIN_SMMU, GPB_SMMU_ID2, &r->id2);
        gpb_rd(io, r, GPB_WIN_SMMU, GPB_SMMU_GFSR, &r->gfsr);
        r->smmu = GPB_DONE;
    }
    // Stage 4: the GPU register window needs CX and GX on. GX is GMU-owned on gen8, so before the GMU runs this stays skipped.
    if (!(have & (1u << GPB_WIN_GPU))) r->gpu = GPB_NOMAP;
    else if (r->cc != GPB_DONE || !gpb_gdsc_on(r->cx_gdscr) || !gpb_gdsc_on(r->gx_gdscr)) r->gpu = GPB_SKIPPED_PREREQ;
    else {
        if (io->stage) io->stage(io->cookie, GPB_WIN_GPU);
        gpb_rd(io, r, GPB_WIN_GPU, GPB_RBBM_STATUS_BYTES, &r->rbbm_status);
        r->gpu = GPB_DONE;
    }
}
#endif
