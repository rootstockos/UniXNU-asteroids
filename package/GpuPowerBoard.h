// SPDX-License-Identifier: BSD-2-Clause-Patent
// asteroids (SM7635, volcano) Adreno 810 power/clock observation and the CX power-up, modes 1-3. From UniXNU-sheng package/GpuPowerBoard.h
// and Allenwdk/UniXNU-uke's copy (BSD-2-Clause-Patent): the same staged, gated read sequence, with the 3a's facts. Every offset
// names its source. Nothing here touches hardware; GpuPowerProbe.cpp owns the MMIO. Host-tested by GpuPowerBoardTests.c.
//
// Why staged: the GPU SMMU, GPUCC and the GPU register window sit behind clocks and power domains that Darwin has no driver for.
// Reading a block whose clock or rail is off can hang the bus (a reset) instead of failing, so each block is read only when the
// previous one shows its prerequisite on. Mode 1 only reads; mode 2 adds the one write Linux's GCC probe makes; mode 3 powers the
// CX domain as the 3a's downstream GDSC driver does and turns on the GPU SMMU's two GPU-CC clocks. GX and everything after it
// are separate steps, each with its own facts and test.
#ifndef ASTEROIDS_GPU_POWER_BOARD_H
#define ASTEROIDS_GPU_POWER_BOARD_H
#include <stdint.h>

#define GPB_MAX_MODE 19u   // 4 runs modes 1-3, then the GMU pool test below (no SMMU or GMU access); 5 then builds the GMU's
                          // SMMU context on the pool (gpb_gmu_context_gate, NativeAdrenoGmuContextBringup); 6 then reads
                          // the GMU clock tree (gpb_read_gmu_clocks) and the AOP command DB, and writes nothing; 7 then brings the GMU clocks
                          // up as Linux does (gpb_gmu_clocks_on), with the GMU still in reset; 8 then reads the GMU
                          // core and RSCC state the cold start checks first (gpb_read_gmu_state), and writes nothing more;
                          // 9 loads the GMU firmware and waits for HFI ready (ShengGmuLoadAndStartHfi) when that state is
                          // clean; 10 runs the whole cold start with the HFI startup messages (ShengGmuColdStart); 11
                          // then asks the GMU for GX (gpb_gx_on) and reads RBBM; 12 then reads the SW fuse and CP state
                          // (gpb_gpu_survey), builds the GPU's own SMMU context and copies the SQE microcode into it; 13
                          // then runs a8xx hw_init's register writes up to the SQE base (gpb_hw_init); 14 then starts
                          // the CP: ring, SQE, CP_ME_INIT and a first CP_MEM_WRITE; 15 then loads the zap shader (PAS 13)
                          // and leaves secure mode (CP_SET_SECURE_MODE 0), then writes a second word; 16 then runs a
                          // one-packet IB and has the GPU write a bar into the firmware framebuffer from a second IB; 17
                          // then adopts the BO pool the loader keeps from XNU, attaches fences and the timer, publishes
                          // NativeAdreno's user client and runs it from the kernel; 18 draws the bar once instead of for
                          // two minutes, does what 17 does, then prints what user space does with the service for ten minutes;
                          // 19 also publishes the AGXAccelerator for the original Metal discovery before it watches

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
#define GPB_SMMU_WINDOW 0x40000ULL        // the whole kgsl-smmu@3da0000 reg (mode 5): ID1 gives 2 x 32 pages of 4 KiB
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
// Mode 3, from the 3a's drivers/clk/qcom/gdsc-regulator.c (qcom,gdsc, the downstream driver the live DT's GDSC nodes bind to;
// not mainline gdsc.c, whose memory-retain writes it does not make): GDSCR HW_CONTROL BIT(1), SW_OVERRIDE BIT(2),
// RETAIN_FF_ENABLE BIT(11), CLK_DIS_WAIT (0xF << 12). Probe: clear HW_CONTROL and SW_OVERRIDE, set CLK_DIS_WAIT to the node's
// qcom,clk-dis-wait-val (8). Enable: refuse while HW_CONTROL is set, clear SW_COLLAPSE, wait 1 us, poll PWR_ON in the
// hw-ctrl-addr register (no qcom,support-cfg-gdscr on the CX node) up to TIMEOUT_US 1500 times 1 us, then set RETAIN_FF
// (qcom,retain-regs) if it is clear, then 2 us. The node needs gcc_gpu_cfg_ahb_clk (clocks = GCC 32), which stage 1 checks.
#define GPB_GDSC_HW_CONTROL (1u << 1)
#define GPB_GDSC_SW_OVERRIDE (1u << 2)
#define GPB_GDSC_RETAIN_FF (1u << 11)
#define GPB_GDSC_CLK_DIS_WAIT_MASK (0xFu << 12)
#define GPB_GDSC_CLK_DIS_WAIT_VAL (8u << 12)
#define GPB_GDSC_POLL_TRIES 1500u
// The live DT's kgsl-smmu@3da0000 clocks: gpu_cc_hlos1_vote_gpu_smmu (GPUCC 20; BRANCH_HALT_VOTED: wait for CLK_OFF to clear),
// gcc_gpu_memnoc_gfx, gcc_gpu_snoc_dvm_gfx (both on at boot, T2) and gpu_cc_ahb (GPUCC 2; BRANCH_HALT_DELAY: clk-branch.c waits
// 10 us after enabling). gpu_cc_hlos1_vote_gpu_smmu_clk has no halt delay of its own; 500 us bounds its status wait.
#define GPB_BRANCH_HALT_DELAY_US 10u
#define GPB_VOTE_POLL_US 500u

// The GMU pool: uefi-asteroids 8a92c2c reserves GMU_Pool 0xE6440000 + 0x02000000 (the first 32 MiB of the stock map's
// DXE_Heap1) for the GMU payloads and their SMMU tables, adopted over IODeviceMemory::withRange as uke's "GMU Pool" is.
#define GPB_GMU_POOL_BASE 0xE6440000ULL
#define GPB_GMU_POOL_BYTES 0x02000000ULL

// ARM SMMUv2 global space 0 (IHI 0062D.c): sCR0 0x0, ID0 0x20, ID1 0x24, ID2 0x28, sGFSR 0x48.
#define GPB_SMMU_SCR0 0x0
#define GPB_SMMU_ID0 0x20
#define GPB_SMMU_ID1 0x24
#define GPB_SMMU_ID2 0x28
#define GPB_SMMU_GFSR 0x48
// Stream match and stream-to-context registers in global space 0: SMRn 0x800 + 4n (VALID bit 31), S2CRn 0xc00 + 4n.
// IDR0.NUMSMRG (bits 0-7) gives the count. On the phone (T4/T5) the SMMU rows read sCR0 0x00290406, ID0 0x4c017e18
// (NUMSMRG 0x18: 24 entries), ID1 0x40000016 (NUMCB 0x16: 22 context banks), ID2 0x00005511. The 32-entry cap covers all of
// them and stays inside global space 0 (0x800 + 4 * 32 = 0x880, 0xc00 + 4 * 32 = 0xc80). uke's mode 4 runs only when none is
// valid.
#define GPB_SMMU_SMR(n) (0x800 + 4 * (n))
#define GPB_SMMU_S2CR(n) (0xc00 + 4 * (n))
#define GPB_SMMU_SMR_VALID (1u << 31)
#define GPB_SMMU_MAX_SMR 32u

// Mode 5: the GMU's own SMMU context, as uke's mode 4 builds it (NativeAdreno NativeAdrenoGmuContextBringup). The stream is the
// live DT's qcom,gmu@3d37000 iommus = <&kgsl_smmu 0x5 0x0> (SID 5, mask 0). Bank 1, ASID 1 and SMR/S2CR slot 2 are uke's
// choice (Linux gives bank 0 to the GPU and takes the first free SMR); T5b read all 24 SMRs invalid with S2CR 0x00020000
// (FAULT). The kit writes only that bank's registers, its CBAR/CBA2R in global space 1 and the one SMR/S2CR pair; never sCR0.
#define GPB_GMU_BANK 1u
#define GPB_GMU_ASID 1u
#define GPB_GMU_SLOT 2u
#define GPB_GMU_SID 5u
#define GPB_SMMU_S2CR_FAULT 0x00020000u   // S2CR TYPE 2 (fault), as T5b read every entry
// Context bank n's page: after the global half, 2^(NUMPAGENDXB + 1) pages of 1 << (12 + 4 * PAGESIZE) bytes (ID1 bits 28-30
// and 31). The phone's ID1 0x40000016 gives 32 pages of 4 KiB, so bank 1 is at 0x21000. FSR is +0x58 in the bank.
static inline uint32_t gpb_smmu_cb_offset(uint32_t id1, unsigned bank) {
    const uint32_t page = (id1 & (1u << 31)) ? 0x10000u : 0x1000u;
    return page * ((2u << ((id1 >> 28) & 7)) + bank);
}
#define GPB_SMMU_CB_FSR 0x58
#define GPB_SMMU_CB_ACTLR 0x4
#define GPB_GPU_ACTLR 0x32bu            // the 3a DT's kgsl-smmu qcom,actlr <0x0 0x3ff 0x32b>: every stream; the kit leaves ACTLR alone

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
    void (*delay_us)(void *cookie, unsigned us);   // mode 3 only; may be NULL where no mode 3 runs
};
#define GPB_STAGE_CX_GDSC 0x20u   // io->stage marker for the CX power-up, apart from every window index
enum {GPB_WIN_GCC_GPU = 0, GPB_WIN_GCC_VOTE = 1, GPB_WIN_GPUCC = 2, GPB_WIN_SMMU = 3, GPB_WIN_GPU = 4, GPB_WINDOWS = 5};

struct gpb_result {
    unsigned mode;                       // 1 read-only; 2 additionally enable gcc_gpu_cfg_ahb_clk if it is off (as Linux does)
    enum gpb_state gcc, cc, smmu, gpu;
    uint32_t cfg_ahb, cfg_ahb_after, memnoc, dvm, axi, pll_vote;
    uint32_t cx_gdscr, cx_hwctrl, gx_gdscr, cc_ahb, cc_smmu, cc_gmu;
    uint32_t scr0, id0, id1, id2, gfsr;
    unsigned num_smr, smr_valid;         // the stream-match survey (stage 3): entries read, entries with VALID set
    uint32_t smr[GPB_SMMU_MAX_SMR], s2cr[GPB_SMMU_MAX_SMR];
    uint32_t rbbm_status;
    // Mode 3: the CX GDSCR before, after the probe-init write and after enabling; the hw-ctrl status it polled and the tries it
    // took; the two clocks before. powered_cx and powered_clocks are 1 only when the status read confirmed them.
    uint32_t cx_before, cx_init, cx_after, cx_hwctrl_after, cc_ahb_before, cc_smmu_before;
    unsigned cx_tries, powered_cx, powered_clocks, cx_refused;
    unsigned enabled_ahb;                // 1 if mode 2 wrote the enable bit
    unsigned writes;                     // every write attempted (mode 1: always 0)
    int io_error;
};

static inline int gpb_clock_on(uint32_t cbcr) {return (cbcr & GPB_CBCR_CLK_OFF) == 0;}
// gcc_gpu_cfg_ahb_clk: the enable bit is the test (an idle branch with hardware clock gating can read CLK_OFF while enabled).
static inline int gpb_cfg_ahb_enabled(uint32_t cbcr) {return (cbcr & GPB_CBCR_ENABLE) != 0;}
static inline int gpb_gdsc_on(uint32_t gdscr) {return (gdscr & GPB_GDSC_PWR_ON) != 0;}

// Mode 4: the pool round trip. Fills the mapped pool with an address-derived 64-bit pattern using plain stores (never
// memset/bzero: uke's first pool mapping took an alignment abort in bzero's DC ZVA), reads it back, then zeroes it the same
// way and reads that back. Counts mismatches; first_bad is the first wrong 8-byte index (~0 if none).
struct gpb_pool_result {uint64_t words, pattern_bad, zero_bad, first_bad;};
static inline uint64_t gpb_pool_pattern(uint64_t index) {return (index * 0x9e3779b97f4a7c15ULL) ^ 0xa5a5a5a55a5a5a5aULL;}
static inline void gpb_pool_test(volatile uint64_t *pool, uint64_t bytes, struct gpb_pool_result *p) {
    p->words = bytes / 8; p->pattern_bad = p->zero_bad = 0; p->first_bad = ~0ULL;
    for (uint64_t i = 0; i < p->words; ++i) pool[i] = gpb_pool_pattern(i);
    for (uint64_t i = 0; i < p->words; ++i)
        if (pool[i] != gpb_pool_pattern(i)) {if (!p->pattern_bad++) p->first_bad = i;}
    for (uint64_t i = 0; i < p->words; ++i) pool[i] = 0;
    for (uint64_t i = 0; i < p->words; ++i)
        if (pool[i]) {if (!p->zero_bad++ && p->first_bad == ~0ULL) p->first_bad = i;}
}

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
    // Mode 3: power CX (downstream gdsc-regulator.c) when stage 2 read it collapsed, then the SMMU's two GPU-CC clocks.
    r->cx_before = r->cx_gdscr; r->cc_ahb_before = r->cc_ahb; r->cc_smmu_before = r->cc_smmu;
    if (mode >= 3 && r->cc == GPB_DONE && !gpb_gdsc_on(r->cx_gdscr) && !gpb_gdsc_on(r->cx_hwctrl)) {
        int bad = 0;
        uint32_t v = r->cx_gdscr;
        if (io->stage) io->stage(io->cookie, GPB_STAGE_CX_GDSC);   // announce the CX power-up before its first write
        if (v & GPB_GDSC_HW_CONTROL) r->cx_refused = 1;   // gdsc_enable: "Invalid enable while ... under HW control"
        else {
            // Probe-time init: HW trigger and SW override off, the wait between states from qcom,clk-dis-wait-val.
            v = (v & ~(GPB_GDSC_HW_CONTROL | GPB_GDSC_SW_OVERRIDE | GPB_GDSC_CLK_DIS_WAIT_MASK)) | GPB_GDSC_CLK_DIS_WAIT_VAL;
            ++r->writes; bad |= io->write32(io->cookie, GPB_WIN_GPUCC, GPB_CC_CX_GDSCR, v);
            bad |= gpb_rd(io, r, GPB_WIN_GPUCC, GPB_CC_CX_GDSCR, &r->cx_init);
            if (!bad) {
                v = r->cx_init & ~GPB_GDSC_SW_COLLAPSE;   // enable
                ++r->writes; bad |= io->write32(io->cookie, GPB_WIN_GPUCC, GPB_CC_CX_GDSCR, v);
                if (io->delay_us) io->delay_us(io->cookie, 1);
                uint32_t st = 0;
                for (r->cx_tries = 1; !bad && r->cx_tries <= GPB_GDSC_POLL_TRIES; ++r->cx_tries) {
                    if (gpb_rd(io, r, GPB_WIN_GPUCC, GPB_CC_CX_GDS_HW_CTRL, &st)) {bad = 1; break;}
                    if (gpb_gdsc_on(st)) break;
                    if (io->delay_us) io->delay_us(io->cookie, 1);
                }
                r->cx_hwctrl_after = st;
                if (!bad && gpb_gdsc_on(st)) {
                    if (!(v & GPB_GDSC_RETAIN_FF)) {++r->writes; bad |= io->write32(io->cookie, GPB_WIN_GPUCC, GPB_CC_CX_GDSCR, v | GPB_GDSC_RETAIN_FF);}
                    if (io->delay_us) io->delay_us(io->cookie, 2);
                    r->powered_cx = !bad;
                }
            }
            gpb_rd(io, r, GPB_WIN_GPUCC, GPB_CC_CX_GDSCR, &r->cx_after);
            r->cx_gdscr = r->cx_after; r->cx_hwctrl = r->cx_hwctrl_after;
        }
        if (r->powered_cx) {
            uint32_t a = 0, c = 0;
            bad |= gpb_rd(io, r, GPB_WIN_GPUCC, GPB_CC_AHB_CLK, &a);
            ++r->writes; bad |= io->write32(io->cookie, GPB_WIN_GPUCC, GPB_CC_AHB_CLK, a | GPB_CBCR_ENABLE);
            if (io->delay_us) io->delay_us(io->cookie, GPB_BRANCH_HALT_DELAY_US);
            bad |= gpb_rd(io, r, GPB_WIN_GPUCC, GPB_CC_SMMU_VOTE_CLK, &c);
            ++r->writes; bad |= io->write32(io->cookie, GPB_WIN_GPUCC, GPB_CC_SMMU_VOTE_CLK, c | GPB_CBCR_ENABLE);
            for (unsigned waited = 0; !bad; waited += 10) {
                if (gpb_rd(io, r, GPB_WIN_GPUCC, GPB_CC_SMMU_VOTE_CLK, &r->cc_smmu)) {bad = 1; break;}
                if (gpb_clock_on(r->cc_smmu) || waited >= GPB_VOTE_POLL_US) break;
                if (io->delay_us) io->delay_us(io->cookie, 10);
            }
            gpb_rd(io, r, GPB_WIN_GPUCC, GPB_CC_AHB_CLK, &r->cc_ahb);
            r->powered_clocks = !bad && gpb_clock_on(r->cc_smmu);
        }
        if (bad) r->io_error = 1;
    }
    // Stage 3: the GPU SMMU needs the CX domain and its vote clock (as sheng and uke gate it; the 3a's kgsl-smmu node lists that
    // clock). CX counts as on when either GDSCR or its hw-ctrl status shows PWR_ON. Global space 0 only.
    const int cx_on = gpb_gdsc_on(r->cx_gdscr) || gpb_gdsc_on(r->cx_hwctrl);
    if (!(have & (1u << GPB_WIN_SMMU))) r->smmu = GPB_NOMAP;
    else if (r->cc != GPB_DONE || !cx_on || !gpb_clock_on(r->cc_smmu)) r->smmu = GPB_SKIPPED_PREREQ;
    else {
        if (io->stage) io->stage(io->cookie, GPB_WIN_SMMU);
        gpb_rd(io, r, GPB_WIN_SMMU, GPB_SMMU_SCR0, &r->scr0);
        gpb_rd(io, r, GPB_WIN_SMMU, GPB_SMMU_ID0, &r->id0);
        gpb_rd(io, r, GPB_WIN_SMMU, GPB_SMMU_ID1, &r->id1);
        gpb_rd(io, r, GPB_WIN_SMMU, GPB_SMMU_ID2, &r->id2);
        gpb_rd(io, r, GPB_WIN_SMMU, GPB_SMMU_GFSR, &r->gfsr);
        // Every stream match entry and its S2CR, read only: which streams the bootloader or hypervisor already routes.
        r->num_smr = r->id0 & 0xff;
        if (r->num_smr > GPB_SMMU_MAX_SMR) r->num_smr = GPB_SMMU_MAX_SMR;
        for (unsigned i = 0; i < r->num_smr; ++i) {
            if (gpb_rd(io, r, GPB_WIN_SMMU, GPB_SMMU_SMR(i), &r->smr[i])) break;
            gpb_rd(io, r, GPB_WIN_SMMU, GPB_SMMU_S2CR(i), &r->s2cr[i]);
            if (r->smr[i] & GPB_SMMU_SMR_VALID) ++r->smr_valid;
        }
        r->smmu = GPB_DONE;
    }
    // Stage 4: the GPU register window needs CX and GX on. GX is GMU-owned on gen8, so before the GMU runs this stays skipped.
    if (!(have & (1u << GPB_WIN_GPU))) r->gpu = GPB_NOMAP;
    else if (r->cc != GPB_DONE || !cx_on || !gpb_gdsc_on(r->gx_gdscr)) r->gpu = GPB_SKIPPED_PREREQ;
    else {
        if (io->stage) io->stage(io->cookie, GPB_WIN_GPU);
        gpb_rd(io, r, GPB_WIN_GPU, GPB_RBBM_STATUS_BYTES, &r->rbbm_status);
        r->gpu = GPB_DONE;
    }
}
// Mode 5's gate. 0 when the GMU context may be built: the SMMU survey ran with no I/O error, CX was on, the vote clock ran, no
// SMR was valid, the chosen slot read invalid with S2CR FAULT, the ID1 bank layout fits the window, and the pool round trip
// (mode 4) was clean. Otherwise the first failed condition, 1-7, in that order, and the probe writes nothing more.
static inline unsigned gpb_gmu_context_gate(const struct gpb_result *r, const struct gpb_pool_result *p) {
    const int cx_on = gpb_gdsc_on(r->cx_gdscr) || gpb_gdsc_on(r->cx_hwctrl);
    if (r->smmu != GPB_DONE || r->io_error) return 1;
    if (!cx_on || !gpb_clock_on(r->cc_smmu)) return 2;
    if (r->smr_valid) return 3;
    if (r->num_smr <= GPB_GMU_SLOT || r->smr[GPB_GMU_SLOT] || r->s2cr[GPB_GMU_SLOT] != GPB_SMMU_S2CR_FAULT) return 4;
    if (gpb_smmu_cb_offset(r->id1, GPB_GMU_BANK) + 0x1000u > GPB_SMMU_WINDOW || (r->id1 & 0xff) <= GPB_GMU_BANK) return 5;
    if (!p || p->words != GPB_GMU_POOL_BYTES / 8) return 6;
    if (p->pattern_bad || p->zero_bad) return 7;
    return 0;
}
static inline const char *gpb_gmu_gate_text(unsigned gate) {
    static const char *const text[] = {"ok", "smmu not read", "cx or vote clock off", "an smr is valid", "slot 2 not free",
                                       "bank 1 outside the window", "pool not tested", "pool round trip failed"};
    return gate < sizeof text / sizeof *text ? text[gate] : "?";
}
// Mode 6 (read-only): the clocks a6xx_gmu_resume needs before a GMU cold start, and their sources. The live DT's
// qcom,gmu@3d37000 clocks are gmu_clk = GPUCC 6, cxo_clk = GPUCC 8, axi_clk = GCC 0x14, memnoc_clk = GCC 0x23, ahb_clk =
// GPUCC 2 and hub_clk = GPUCC 0x17 (23); its qcom,gmu-freq-table is 350 MHz (corner 0x40) and 650 MHz (0x80). Indices and
// offsets: mainline gpucc-milos.c and qcom,milos-gpucc.h (torvalds/linux 734bef548a96; Milos is SM7635), identical in the
// 3a's downstream gpucc-volcano.c. Only PLL0 exists (LUCID_OLE, 700 MHz: L 0x24, alpha 0x7555); gpu_cc_gmu_clk_src runs
// from PLL0_OUT_EVEN (sel 2), gpu_cc_hub_clk_src from GPLL0 (sel 5, 200 MHz = div 3). LUCID_OLE registers (clk-alpha-pll.c):
// MODE +0, OPMODE +4, STATUS +0xc, L_VAL +0x10, ALPHA +0x14, USER_CTL +0x18; RCG CMD +0, CFG +4 (clk-rcg2.c).
#define GPB_CC_PLL0 0x0
#define GPB_PLL_MODE 0x0
#define GPB_PLL_OPMODE 0x4
#define GPB_PLL_STATUS 0xc
#define GPB_PLL_L_VAL 0x10
#define GPB_PLL_ALPHA 0x14
#define GPB_PLL_USER_CTL 0x18
#define GPB_PLL_LOCK_DET (1u << 31)       // clk-alpha-pll.c PLL_LOCK_DET, in MODE
#define GPB_CC_GMU_RCGR 0x9318            // gpu_cc_gmu_clk_src
#define GPB_CC_HUB_RCGR 0x93ec            // gpu_cc_hub_clk_src
#define GPB_CC_HUB_DIV 0x942c             // gpu_cc_hub_div_clk_src (divider, width 4)
#define GPB_CC_FF_RCGR 0x9474             // gpu_cc_ff_clk_src
#define GPB_RCG_ROOT_OFF (1u << 31)       // clk-rcg2.c CMD_ROOT_OFF
// The GMU node's GPUCC branches, then the hub and memnoc branches (gpucc-milos.c) and the seven the probe keeps on
// (gpu_cc_milos_critical_cbcrs / the volcano probe's regmap_update_bits list).
#define GPB_GMU_BRANCHES 6u
static const uint32_t gpb_gmu_branch[GPB_GMU_BRANCHES] = {
    0x90d4,   // gpu_cc_cx_gmu_clk (gmu_clk)
    0x90e4,   // gpu_cc_cxo_clk (cxo_clk)
    0x90bc,   // gpu_cc_ahb_clk (ahb_clk)
    0x90e8,   // gpu_cc_hub_cx_int_clk (hub_clk)
    0x93e8,   // gpu_cc_hub_aon_clk
    0x90f4,   // gpu_cc_memnoc_gfx_clk
};
#define GPB_CRITICAL_BRANCHES 7u
static const uint32_t gpb_critical_branch[GPB_CRITICAL_BRANCHES] = {0x93a4, 0x9008, 0x9010, 0x9064, 0x93a8, 0x9004, 0x90cc};
struct gpb_gmu_clocks {
    uint32_t pll0_mode, pll0_opmode, pll0_status, pll0_l, pll0_alpha, pll0_user;
    uint32_t gmu_cmd, gmu_cfg, hub_cmd, hub_cfg, hub_div, ff_cmd, ff_cfg;
    uint32_t branch[GPB_GMU_BRANCHES], critical[GPB_CRITICAL_BRANCHES];
    uint32_t axi, memnoc, gpll0_vote;
    int done;                            // 0: GPUCC was not readable, nothing was read
};
// A branch digit for the log: bit 0 enable, bit 1 CLK_OFF (so 1 = on, 3 = enabled but off, 2 = off).
static inline unsigned gpb_branch_digit(uint32_t cbcr) {return (cbcr & GPB_CBCR_ENABLE) | ((cbcr >> 30) & 2);}
// Only reads, and only once stages 1-2 read GPUCC with cfg_ahb enabled. GCC words are read again (the stage-1 values can be
// stale after modes 3-5).
static inline void gpb_read_gmu_clocks(const struct gpb_io *io, struct gpb_result *r, struct gpb_gmu_clocks *k) {
    __builtin_memset(k, 0, sizeof *k);
    if (r->cc != GPB_DONE || !gpb_cfg_ahb_enabled(r->cfg_ahb_after)) return;
    gpb_rd(io, r, GPB_WIN_GPUCC, GPB_CC_PLL0 + GPB_PLL_MODE, &k->pll0_mode);
    gpb_rd(io, r, GPB_WIN_GPUCC, GPB_CC_PLL0 + GPB_PLL_OPMODE, &k->pll0_opmode);
    gpb_rd(io, r, GPB_WIN_GPUCC, GPB_CC_PLL0 + GPB_PLL_STATUS, &k->pll0_status);
    gpb_rd(io, r, GPB_WIN_GPUCC, GPB_CC_PLL0 + GPB_PLL_L_VAL, &k->pll0_l);
    gpb_rd(io, r, GPB_WIN_GPUCC, GPB_CC_PLL0 + GPB_PLL_ALPHA, &k->pll0_alpha);
    gpb_rd(io, r, GPB_WIN_GPUCC, GPB_CC_PLL0 + GPB_PLL_USER_CTL, &k->pll0_user);
    gpb_rd(io, r, GPB_WIN_GPUCC, GPB_CC_GMU_RCGR, &k->gmu_cmd);
    gpb_rd(io, r, GPB_WIN_GPUCC, GPB_CC_GMU_RCGR + 4, &k->gmu_cfg);
    gpb_rd(io, r, GPB_WIN_GPUCC, GPB_CC_HUB_RCGR, &k->hub_cmd);
    gpb_rd(io, r, GPB_WIN_GPUCC, GPB_CC_HUB_RCGR + 4, &k->hub_cfg);
    gpb_rd(io, r, GPB_WIN_GPUCC, GPB_CC_HUB_DIV, &k->hub_div);
    gpb_rd(io, r, GPB_WIN_GPUCC, GPB_CC_FF_RCGR, &k->ff_cmd);
    gpb_rd(io, r, GPB_WIN_GPUCC, GPB_CC_FF_RCGR + 4, &k->ff_cfg);
    for (unsigned i = 0; i < GPB_GMU_BRANCHES; ++i) gpb_rd(io, r, GPB_WIN_GPUCC, gpb_gmu_branch[i], &k->branch[i]);
    for (unsigned i = 0; i < GPB_CRITICAL_BRANCHES; ++i) gpb_rd(io, r, GPB_WIN_GPUCC, gpb_critical_branch[i], &k->critical[i]);
    gpb_rd(io, r, GPB_WIN_GCC_GPU, GPB_GCC_DDRSS_GPU_AXI, &k->axi);
    gpb_rd(io, r, GPB_WIN_GCC_GPU, GPB_GCC_GPU_MEMNOC_GFX, &k->memnoc);
    gpb_rd(io, r, GPB_WIN_GCC_VOTE, GPB_GCC_PLL_VOTE, &k->gpll0_vote);
    k->done = 1;
}
// Mode 7: the GMU clock bring-up a6xx_gmu_resume relies on, in Linux order, each step mainline's code for these clocks
// (torvalds/linux 734bef548a96):
//   1 gcc_gpu_gpll0_clk_src / gcc_gpu_gpll0_div_clk_src: the GCC vote register 0x52000 bits 15/16 (gcc-milos.c,
//     BRANCH_HALT_DELAY: 10 us, no status)
//   2 the seven always-on enables the gpucc probe sets (gpu_cc_milos_critical_cbcrs, bit 0 each)
//   3 clk_lucid_ole_pll_configure(gpu_cc_pll0, gpu_cc_pll0_config): L 0x24 with the CAL_L (bit 16) and RINGOSC_CAL_L
//     (bit 24) fields 0x44, alpha 0x7555, CONFIG_CTL/U/U1, USER_CTL/U, TEST_CTL U/U1/U2 (a zero value is not written,
//     as clk_alpha_pll_write_config skips it), then OUTCTRL off, OPMODE STANDBY, RESET_N on
//   4 alpha_pll_lucid_evo_enable: refuse an FSM-voted PLL (USER_CTL bit 25, checked before step 3 writes it), skip one already running (OPMODE RUN and
//     OUTCTRL), else RESET_N, OPMODE RUN, LOCK_DET within 1500 x 1 us, USER_CTL outputs 0x7, OUTCTRL
//   5 gpu_cc_hub_clk_src to 200 MHz (GPLL0 sel 5, pre_div 3 -> 5): a6xx_gmu_resume asks 150 MHz for non-A740 GPUs and
//     the CEIL table lookup gives 200 MHz. clk_rcg2_shared_ops on an enabled root: ROOT_EN, wait for the root (500 x
//     1 us), write CFG (mask: the 5-bit divider, MODE, HW_CLK_CTRL and SRC_SEL), CMD_UPDATE and wait for it to clear
//     (500 x 1 us), drop ROOT_EN
//   6 gpu_cc_gmu_clk_src to 350 MHz (PLL0_OUT_EVEN sel 2, pre_div 1 -> 1): resume asks 200 MHz, the table's next rate is
//     350 MHz, which is also the first qcom,gmu-freq-table level; the same RCG sequence
//   7 gpu_cc_ahb_clk enable (bit 0; BRANCH_HALT_DELAY: 10 us), the one GMU node clock T8 read disabled
// Nothing else is written; mode 7's writes are exactly gpb_mode7_writable.
#define GPB_PLL_CONFIG_CTL 0x20
#define GPB_PLL_RESET_N (1u << 2)
#define GPB_PLL_OUTCTRL (1u << 0)
#define GPB_PLL_VOTE_RUN (1u << 25)       // LUCID_EVO_ENABLE_VOTE_RUN, in USER_CTL
#define GPB_PLL_LOCK_TRIES 1500u
#define GPB_RCG_UPDATE (1u << 0)
#define GPB_RCG_ROOT_EN (1u << 1)
#define GPB_RCG_TRIES 500u
#define GPB_RCG_CFG_MASK (0x1fu | (0x3u << 12) | (1u << 20) | (0x7u << 8))
#define GPB_HUB_CFG ((5u << 8) | 5u)      // GPLL0 / 3 = 200 MHz
#define GPB_GMU_CFG ((2u << 8) | 1u)      // PLL0_OUT_EVEN / 1 = 350 MHz
// gpu_cc_pll0_config (gpucc-milos.c, the same in gpucc-volcano.c), as {offset, value}; zero values are left out.
#define GPB_PLL0_CONFIG_WRITES 10u
static const uint32_t gpb_pll0_config[GPB_PLL0_CONFIG_WRITES][2] = {
    {0x10, 0x24u | (0x44u << 16) | (0x44u << 24)},   // L_VAL with CAL_L and RINGOSC_CAL_L
    {0x14, 0x7555}, {0x20, 0x20485699}, {0x24, 0x00182261}, {0x28, 0x82aa299c},
    {0x18, 0x00000400}, {0x1c, 0x00000005},          // USER_CTL (even post-divider 2), USER_CTL_U
    {0x30, 0x00000003}, {0x34, 0x00009000}, {0x38, 0x00000034},   // TEST_CTL_U, U1, U2 (TEST_CTL is 0: not written)
};
static inline int gpb_mode7_writable(unsigned window, uint32_t off) {
    if (window == GPB_WIN_GCC_VOTE) return off == GPB_GCC_PLL_VOTE;
    if (window != GPB_WIN_GPUCC) return 0;
    if (off <= 0x38) return 1;   // PLL0
    if (off == GPB_CC_GMU_RCGR || off == GPB_CC_GMU_RCGR + 4 || off == GPB_CC_HUB_RCGR || off == GPB_CC_HUB_RCGR + 4 ||
        off == GPB_CC_AHB_CLK) return 1;
    for (unsigned i = 0; i < GPB_CRITICAL_BRANCHES; ++i) if (off == gpb_critical_branch[i]) return 1;
    return 0;
}
struct gpb_gmu_clock_on {
    unsigned step;                       // last step started, 1-7; 8 when all finished
    unsigned failed;                     // 0 ok, 1 refused (precondition or FSM PLL), 2 timeout, 3 I/O
    unsigned pll_was_running, lock_polls, hub_root_polls, hub_polls, gmu_root_polls, gmu_polls, writes;
    uint32_t vote, pll0_mode, pll0_opmode, pll0_user, hub_cmd, hub_cfg, gmu_cmd, gmu_cfg, ahb;
};
static inline int gpb_wr7(const struct gpb_io *io, struct gpb_gmu_clock_on *o, unsigned w, uint32_t off, uint32_t v) {
    if (!gpb_mode7_writable(w, off)) return -1;
    ++o->writes;
    return io->write32(io->cookie, w, off, v) ? -1 : 0;
}
static inline int gpb_set7(const struct gpb_io *io, struct gpb_result *r, struct gpb_gmu_clock_on *o, unsigned w, uint32_t off,
                           uint32_t clear, uint32_t set) {
    uint32_t v;
    if (gpb_rd(io, r, w, off, &v)) return -1;
    return gpb_wr7(io, o, w, off, (v & ~clear) | set);
}
// One clk_rcg2_shared set_rate on an enabled root. Returns 0, 2 (timeout) or 3 (I/O).
static inline unsigned gpb_rcg7(const struct gpb_io *io, struct gpb_result *r, struct gpb_gmu_clock_on *o, uint32_t rcgr,
                                uint32_t cfg, unsigned *root_polls, unsigned *polls) {
    uint32_t v = 0;
    if (gpb_set7(io, r, o, GPB_WIN_GPUCC, rcgr, 0, GPB_RCG_ROOT_EN)) return 3;
    for (*root_polls = 0;; ++*root_polls) {
        if (gpb_rd(io, r, GPB_WIN_GPUCC, rcgr, &v)) return 3;
        if (!(v & GPB_RCG_ROOT_OFF)) break;
        if (*root_polls >= GPB_RCG_TRIES) return 2;
        if (io->delay_us) io->delay_us(io->cookie, 1);
    }
    if (gpb_set7(io, r, o, GPB_WIN_GPUCC, rcgr + 4, GPB_RCG_CFG_MASK, cfg)) return 3;
    if (gpb_set7(io, r, o, GPB_WIN_GPUCC, rcgr, 0, GPB_RCG_UPDATE)) return 3;
    for (*polls = 0;; ++*polls) {
        if (gpb_rd(io, r, GPB_WIN_GPUCC, rcgr, &v)) return 3;
        if (!(v & GPB_RCG_UPDATE)) break;
        if (*polls >= GPB_RCG_TRIES) return 2;   // ROOT_EN stays set, as Linux leaves it on this error
        if (io->delay_us) io->delay_us(io->cookie, 1);
    }
    return gpb_set7(io, r, o, GPB_WIN_GPUCC, rcgr, GPB_RCG_ROOT_EN, 0) ? 3 : 0;
}
static inline void gpb_gmu_clocks_on(const struct gpb_io *io, struct gpb_result *r, const struct gpb_gmu_clocks *k,
                                     struct gpb_gmu_clock_on *o) {
    __builtin_memset(o, 0, sizeof *o);
    const int cx_on = gpb_gdsc_on(r->cx_gdscr) || gpb_gdsc_on(r->cx_hwctrl);
    o->step = 1;
    if (!k->done || !cx_on || r->io_error || (k->gmu_cmd & GPB_RCG_ROOT_OFF) || (k->hub_cmd & GPB_RCG_ROOT_OFF)) {
        o->failed = 1; return;
    }
#define GPB_STEP(n) do {o->step = (n);} while (0)
#define GPB_TRY(x) do {if (x) {o->failed = 3; return;}} while (0)
    GPB_TRY(gpb_set7(io, r, o, GPB_WIN_GCC_VOTE, GPB_GCC_PLL_VOTE, 0, GPB_GCC_GPLL0_GPU | GPB_GCC_GPLL0_DIV_GPU));
    if (io->delay_us) io->delay_us(io->cookie, GPB_BRANCH_HALT_DELAY_US);
    GPB_STEP(2);
    for (unsigned i = 0; i < GPB_CRITICAL_BRANCHES; ++i)
        if (!(k->critical[i] & GPB_CBCR_ENABLE)) GPB_TRY(gpb_set7(io, r, o, GPB_WIN_GPUCC, gpb_critical_branch[i], 0, GPB_CBCR_ENABLE));
    uint32_t mode = 0, opmode = 0, user = 0;
    GPB_TRY(gpb_rd(io, r, GPB_WIN_GPUCC, GPB_CC_PLL0 + GPB_PLL_MODE, &mode));
    GPB_TRY(gpb_rd(io, r, GPB_WIN_GPUCC, GPB_CC_PLL0 + GPB_PLL_OPMODE, &opmode));
    o->pll_was_running = (opmode & 1) && (mode & GPB_PLL_OUTCTRL);
    if (!o->pll_was_running) {
        GPB_STEP(3);
        // Checked before configure: configure writes USER_CTL, which would hide another master's vote bit.
        GPB_TRY(gpb_rd(io, r, GPB_WIN_GPUCC, GPB_CC_PLL0 + GPB_PLL_USER_CTL, &user));
        if (user & GPB_PLL_VOTE_RUN) {o->failed = 1; return;}
        for (unsigned i = 0; i < GPB_PLL0_CONFIG_WRITES; ++i)
            GPB_TRY(gpb_wr7(io, o, GPB_WIN_GPUCC, GPB_CC_PLL0 + gpb_pll0_config[i][0], gpb_pll0_config[i][1]));
        GPB_TRY(gpb_set7(io, r, o, GPB_WIN_GPUCC, GPB_CC_PLL0 + GPB_PLL_MODE, GPB_PLL_OUTCTRL, 0));
        GPB_TRY(gpb_wr7(io, o, GPB_WIN_GPUCC, GPB_CC_PLL0 + GPB_PLL_OPMODE, 0));   // STANDBY
        GPB_TRY(gpb_set7(io, r, o, GPB_WIN_GPUCC, GPB_CC_PLL0 + GPB_PLL_MODE, 0, GPB_PLL_RESET_N));
        GPB_STEP(4);
        GPB_TRY(gpb_set7(io, r, o, GPB_WIN_GPUCC, GPB_CC_PLL0 + GPB_PLL_MODE, 0, GPB_PLL_RESET_N));
        GPB_TRY(gpb_wr7(io, o, GPB_WIN_GPUCC, GPB_CC_PLL0 + GPB_PLL_OPMODE, 1));   // RUN
        for (o->lock_polls = 0;; ++o->lock_polls) {
            GPB_TRY(gpb_rd(io, r, GPB_WIN_GPUCC, GPB_CC_PLL0 + GPB_PLL_MODE, &mode));
            if (mode & GPB_PLL_LOCK_DET) break;
            if (o->lock_polls >= GPB_PLL_LOCK_TRIES) {o->failed = 2; o->pll0_mode = mode; return;}
            if (io->delay_us) io->delay_us(io->cookie, 1);
        }
        GPB_TRY(gpb_set7(io, r, o, GPB_WIN_GPUCC, GPB_CC_PLL0 + GPB_PLL_USER_CTL, 0, 0x7));
        GPB_TRY(gpb_set7(io, r, o, GPB_WIN_GPUCC, GPB_CC_PLL0 + GPB_PLL_MODE, 0, GPB_PLL_OUTCTRL));
    }
    GPB_STEP(5);
    if ((o->failed = gpb_rcg7(io, r, o, GPB_CC_HUB_RCGR, GPB_HUB_CFG, &o->hub_root_polls, &o->hub_polls))) return;
    GPB_STEP(6);
    if ((o->failed = gpb_rcg7(io, r, o, GPB_CC_GMU_RCGR, GPB_GMU_CFG, &o->gmu_root_polls, &o->gmu_polls))) return;
    GPB_STEP(7);
    GPB_TRY(gpb_set7(io, r, o, GPB_WIN_GPUCC, GPB_CC_AHB_CLK, 0, GPB_CBCR_ENABLE));
    if (io->delay_us) io->delay_us(io->cookie, GPB_BRANCH_HALT_DELAY_US);
    GPB_STEP(8);
    gpb_rd(io, r, GPB_WIN_GCC_VOTE, GPB_GCC_PLL_VOTE, &o->vote);
    gpb_rd(io, r, GPB_WIN_GPUCC, GPB_CC_PLL0 + GPB_PLL_MODE, &o->pll0_mode);
    gpb_rd(io, r, GPB_WIN_GPUCC, GPB_CC_PLL0 + GPB_PLL_OPMODE, &o->pll0_opmode);
    gpb_rd(io, r, GPB_WIN_GPUCC, GPB_CC_PLL0 + GPB_PLL_USER_CTL, &o->pll0_user);
    gpb_rd(io, r, GPB_WIN_GPUCC, GPB_CC_HUB_RCGR, &o->hub_cmd);
    gpb_rd(io, r, GPB_WIN_GPUCC, GPB_CC_HUB_RCGR + 4, &o->hub_cfg);
    gpb_rd(io, r, GPB_WIN_GPUCC, GPB_CC_GMU_RCGR, &o->gmu_cmd);
    gpb_rd(io, r, GPB_WIN_GPUCC, GPB_CC_GMU_RCGR + 4, &o->gmu_cfg);
    gpb_rd(io, r, GPB_WIN_GPUCC, GPB_CC_AHB_CLK, &o->ahb);
#undef GPB_STEP
#undef GPB_TRY
}
// Mode 8: the first GMU and RSCC register reads, only after mode 7 finished. Reads only. These are the states
// ShengGmuLoadAndStartHfi verifies before it writes anything (CM3 held in reset, RSCC sequencer idle, no stale HFI ready
// bit, no fatal AO/CM3 interrupt), plus the configuration words it will write and the always-on counter, read twice, to
// show the GMU's XO domain is ticking. GMU byte offsets are dword offsets from a6xx_gmu.xml (A8XX variants) * 4 minus the
// window delta 0x37000 (the GMU window starts at 0x3d37000, the GPU's at 0x3d00000), as ShengGmuSource.h generates them;
// the host test compares the two whenever build/sheng-gmu/ShengGmuSource.h exists.
#define GPB_WIN_GMU 5u                    // outside GPB_WINDOWS: mapped only by mode 8
#define GPB_WIN_RSCC 6u
#define GPB_ALL_WINDOWS 7u
#define GPB_GMU_BASE 0x03d37000ULL
#define GPB_GMU_SIZE 0x68000ULL
#define GPB_RSCC_BASE 0x03d50000ULL
#define GPB_RSCC_SIZE 0x10000ULL
#define GPB_GMU_DELTA 0x37000u
#define GPB_GMU_REG(dword) ((uint32_t)(dword) * 4u - GPB_GMU_DELTA)
#define GPB_GMU_CM3_SYSRESET GPB_GMU_REG(0x1f800)
#define GPB_GMU_CM3_BOOT_CONFIG GPB_GMU_REG(0x1f801)
#define GPB_GMU_CM3_FW_INIT_RESULT GPB_GMU_REG(0x1f81c)
#define GPB_GMU_CM3_CFG GPB_GMU_REG(0x1f82d)
#define GPB_GMU_AO_COUNTER_L GPB_GMU_REG(0x1f840)
#define GPB_GMU_AO_COUNTER_H GPB_GMU_REG(0x1f841)
#define GPB_GMU_HFI_CTRL_STATUS GPB_GMU_REG(0x1f980)
#define GPB_GMU_GMU2HOST_INTR_INFO GPB_GMU_REG(0x1f992)
#define GPB_GMU_GENERAL_8 GPB_GMU_REG(0x1f9c8)
#define GPB_GMU_GENERAL_9 GPB_GMU_REG(0x1f9c9)
#define GPB_GMU_GENERAL_10 GPB_GMU_REG(0x1f9ca)
#define GPB_GMU_AO_HOST_INTERRUPT_STATUS GPB_GMU_REG(0x23b05)
#define GPB_GMU_AHB_FENCE_RANGE_0 GPB_GMU_REG(0x23b11)
#define GPB_GMU_AHB_FENCE_STATUS GPB_GMU_REG(0x23b13)      // bit 0: a write inside the fence range was dropped
#define GPB_GMU_ITCM GPB_GMU_REG(0x1b400)
#define GPB_GMU_DTCM_VERSION GPB_GMU_REG(0x1c400 + 0xff8)   // the word a6xx_gmu_start reads for the ready rule
#define GPB_RSCC_SEQ_BUSY_DRV0 (0x101u * 4u)
#define GPB_GMU_AO_FATAL 0x29u                              // SHENG_GMU_AO_FATAL_MASK
#define GPB_GMU_CM3_FATAL 0x800000u                          // SHENG_GMU_CM3_FATAL_MASK
#define GPB_GMU_STATE_READS 16u
static const uint32_t gpb_gmu_state_reg[GPB_GMU_STATE_READS] = {
    GPB_GMU_CM3_SYSRESET, GPB_GMU_HFI_CTRL_STATUS, GPB_GMU_AO_HOST_INTERRUPT_STATUS, GPB_GMU_GMU2HOST_INTR_INFO,
    GPB_GMU_CM3_FW_INIT_RESULT, GPB_GMU_CM3_BOOT_CONFIG, GPB_GMU_CM3_CFG, GPB_GMU_GENERAL_8, GPB_GMU_GENERAL_9,
    GPB_GMU_GENERAL_10, GPB_GMU_AHB_FENCE_RANGE_0, GPB_GMU_ITCM, GPB_GMU_DTCM_VERSION,
    GPB_GMU_AO_COUNTER_L, GPB_GMU_AO_COUNTER_H, GPB_GMU_AO_COUNTER_L,
};
struct gpb_gmu_state {
    unsigned done;                       // 0 not run (refused), 1 read
    unsigned gate;                       // 0 the cold start's preconditions hold; else the first that fails, gpb_gmu_state_text
    uint32_t v[GPB_GMU_STATE_READS];     // in gpb_gmu_state_reg order
    uint32_t rscc_busy;
    int io_error;
};
static inline const char *gpb_gmu_state_text(unsigned gate) {
    static const char *const t[] = {"ok", "not-run", "io", "not-in-reset", "rscc-busy", "stale-hfi-ready", "ao-fatal",
                                    "cm3-fatal"};
    return gate < 8 ? t[gate] : "?";
}
// `clocks_ok`: mode 7 reached step 8 with no failure. The GMU window is read first (CM3_SYSRESET), then RSCC.
static inline void gpb_read_gmu_state(const struct gpb_io *io, struct gpb_result *r, int clocks_ok, struct gpb_gmu_state *g) {
    __builtin_memset(g, 0, sizeof *g);
    g->gate = 1;
    if (!clocks_ok) return;
    g->done = 1;
    if (io->stage) io->stage(io->cookie, GPB_WIN_GMU);
    for (unsigned i = 0; i < GPB_GMU_STATE_READS; ++i) {
        if (i == GPB_GMU_STATE_READS - 1 && io->delay_us) io->delay_us(io->cookie, 10);   // the counter's second read
        if (gpb_rd(io, r, GPB_WIN_GMU, gpb_gmu_state_reg[i], &g->v[i])) {g->io_error = 1; g->gate = 2; return;}
    }
    if (io->stage) io->stage(io->cookie, GPB_WIN_RSCC);
    if (gpb_rd(io, r, GPB_WIN_RSCC, GPB_RSCC_SEQ_BUSY_DRV0, &g->rscc_busy)) {g->io_error = 1; g->gate = 2; return;}
    g->gate = g->v[0] != 1 ? 3 : g->rscc_busy ? 4 : (g->v[1] & 1) ? 5 : (g->v[2] & GPB_GMU_AO_FATAL) ? 6 :
              (g->v[3] & GPB_GMU_CM3_FATAL) ? 7 : 0;
}
// Mode 11: GX, after a complete cold start (mode 10). The tail of a6xx_gmu_resume and the head of a8xx hw_init, as uke's
// mode 8 ran them on its A732, with the gen8 offsets of the pinned a6xx_gmu.xml / a6xx.xml:
//   1 GMU2HOST_INTR_CLR ~0, then GMU2HOST_INTR_MASK ~CM3_FAULT (A6XX_HFI_IRQ_MASK), as after a6xx_hfi_start
//   2 GMU_PWR_CLK_STATUS (A8XX 0x1f7e8): GX_HM_GDSC_POWER_OFF bit 0, GX_HM_CLK_OFF bit 1, as a8xx_gmu_gx_is_on reads it
//   3 the HFI GX_BW_PERF_VOTE (a6xx_hfi_set_freq: id 30, {ack_type 1, freq index, bw index}), at sheng-gpu-level
//     (default 6, 763 MHz; T33 and before used the lowest, 1) where Linux starts at the top one; the vote waits for its acknowledgement
//   4 a6xx_gmu_set_oob(GMU_OOB_GPU_SET), non-legacy bits: HOST2GMU_INTR_SET bit 30, GMU2HOST_INTR_INFO bit 31 within
//     100 x 100 us, then GMU2HOST_INTR_CLR bit 31. The request is held (not cleared), so GX stays on.
//   5 GMU_PWR_CLK_STATUS again; only when GX reports on: RBBM_STATUS, RBBM_GFX_STATUS, RBBM_GFX_BR_STATUS and
//     RBBM_GBIF_HALT (A8XX 0x12, 0x15, 0x1a, 0x0a) in the GPU block, then GX status once more after 100 ms
// Writes: only those four GMU registers; nothing in the GPU block.
#define GPB_GMU_GMU2HOST_INTR_CLR GPB_GMU_REG(0x1f991)
#define GPB_GMU_GMU2HOST_INTR_MASK GPB_GMU_REG(0x1f993)
#define GPB_GMU_HOST2GMU_INTR_SET GPB_GMU_REG(0x1f994)
#define GPB_GMU_PWR_CLK_STATUS GPB_GMU_REG(0x1f7e8)
#define GPB_GX_OFF_BITS 0x3u                  // GX_HM_GDSC_POWER_OFF | GX_HM_CLK_OFF
#define GPB_OOB_GPU_SET_REQUEST (1u << 30)
#define GPB_OOB_GPU_SET_ACK (1u << 31)
#define GPB_OOB_TRIES 100u
#define GPB_RBBM_STATUS (0x012u * 4u)
#define GPB_RBBM_GFX_STATUS (0x015u * 4u)
#define GPB_RBBM_GFX_BR_STATUS (0x01au * 4u)
#define GPB_RBBM_GBIF_HALT (0x00au * 4u)
struct gpb_gx_io {
    void *cookie;
    int (*read32)(void *, unsigned window, uint32_t off, uint32_t *out);   // GPB_WIN_GMU or GPB_WIN_GPU
    int (*write32)(void *, unsigned window, uint32_t off, uint32_t value);
    // The blocking HFI GX_BW_PERF_VOTE; 0 when acknowledged without a firmware error.
    int (*perf_vote)(void *, uint32_t perf, uint32_t bw, uint32_t *firmware_error);
    void (*delay_us)(void *, unsigned);
};
struct gpb_gx {
    unsigned step;          // last step started, 1-5; 6 when all finished
    unsigned failed;        // 0 ok, 1 vote refused, 2 no OOB ack, 3 I/O, 4 GX still off
    uint32_t perf, bw, vote_status, fw_error, oob_polls;
    uint32_t gx_before, gx_after, gx_later, info_after;
    uint32_t rbbm, gfx, gfx_br, gbif_halt;
    unsigned writes;
};
static inline void gpb_gx_on(const struct gpb_gx_io *io, uint32_t perf, uint32_t bw, struct gpb_gx *x) {
    __builtin_memset(x, 0, sizeof *x);
    x->perf = perf; x->bw = bw;
#define GPB_GX_W(w, off, v) do {++x->writes; if (io->write32(io->cookie, (w), (off), (v))) {x->failed = 3; return;}} while (0)
#define GPB_GX_R(w, off, p) do {if (io->read32(io->cookie, (w), (off), (p))) {x->failed = 3; return;}} while (0)
    x->step = 1;
    GPB_GX_W(GPB_WIN_GMU, GPB_GMU_GMU2HOST_INTR_CLR, ~0u);
    GPB_GX_W(GPB_WIN_GMU, GPB_GMU_GMU2HOST_INTR_MASK, ~GPB_GMU_CM3_FATAL);
    x->step = 2;
    GPB_GX_R(GPB_WIN_GMU, GPB_GMU_PWR_CLK_STATUS, &x->gx_before);
    x->step = 3;
    x->vote_status = (uint32_t)io->perf_vote(io->cookie, perf, bw, &x->fw_error);
    if (x->vote_status || x->fw_error) {x->failed = 1; return;}
    x->step = 4;
    GPB_GX_W(GPB_WIN_GMU, GPB_GMU_HOST2GMU_INTR_SET, GPB_OOB_GPU_SET_REQUEST);
    uint32_t info = 0;
    for (x->oob_polls = 1;; ++x->oob_polls) {
        GPB_GX_R(GPB_WIN_GMU, GPB_GMU_GMU2HOST_INTR_INFO, &info);
        if (info & GPB_OOB_GPU_SET_ACK) break;
        if (x->oob_polls >= GPB_OOB_TRIES) {x->info_after = info; x->failed = 2; break;}
        if (io->delay_us) io->delay_us(io->cookie, 100);
    }
    if (!x->failed) {x->info_after = info; GPB_GX_W(GPB_WIN_GMU, GPB_GMU_GMU2HOST_INTR_CLR, GPB_OOB_GPU_SET_ACK);}
    x->step = 5;
    GPB_GX_R(GPB_WIN_GMU, GPB_GMU_PWR_CLK_STATUS, &x->gx_after);
    if (x->failed) return;
    if (x->gx_after & GPB_GX_OFF_BITS) {x->failed = 4; return;}
    GPB_GX_R(GPB_WIN_GPU, GPB_RBBM_STATUS, &x->rbbm);
    GPB_GX_R(GPB_WIN_GPU, GPB_RBBM_GFX_STATUS, &x->gfx);
    GPB_GX_R(GPB_WIN_GPU, GPB_RBBM_GFX_BR_STATUS, &x->gfx_br);
    GPB_GX_R(GPB_WIN_GPU, GPB_RBBM_GBIF_HALT, &x->gbif_halt);
    if (io->delay_us) io->delay_us(io->cookie, 100000);
    GPB_GX_R(GPB_WIN_GMU, GPB_GMU_PWR_CLK_STATUS, &x->gx_later);
    x->step = 6;
#undef GPB_GX_W
#undef GPB_GX_R
}
// Modes 12-14 use the A810 register map NativeAdreno generates from the pinned a6xx.xml and a8xx_gpu.c
// (drivers/NativeAdreno/NativeAdrenoA810HwInit.h, scripts/derive-a810-hwinit.py): dword offsets in the GPU block
// (byte = dword * 4) and, for the GMU registers, GPB_GMU_REG. They reuse mode 11's gpb_gx_io, with one more window:
// CX_MISC (0x3d9e000, ShengGmu's ShengGmuCxMisc), read only.
#include "NativeAdrenoA810HwInit.h"
#define GPB_WIN_CX 7u
#define GPB_GPU_REG(dword) ((uint32_t)(dword) * 4u)
// Mode 12 (reads only, GX on): the software fuse a6xx_gmu_secure_init writes when SCM is missing (CX_MISC_SW_FUSE_VALUE
// 0x400: RAYTRACING bit 0, FASTBLEND bit 1, LPAC bit 2) and the A8XX frequency-limit status (0x405); the RBBM fuse
// violation status (0x71); then the CP and RBBM registers hw_init is about to program, as reset left them.
#define GPB_CX_SW_FUSE_VALUE (0x400u * 4u)
#define GPB_CX_SW_FUSE_FREQ_LIMIT (0x405u * 4u)
#define GPB_RBBM_SW_FUSE_INT_STATUS 0x071u
#define GPB_SURVEY_GPU 14u
static const uint32_t gpb_survey_gpu[GPB_SURVEY_GPU] = {
    A810_RBBM_STATUS, A810_RBBM_INT_0_STATUS, GPB_RBBM_SW_FUSE_INT_STATUS, A810_RBBM_GBIF_HALT, A810_GBIF_HALT,
    A810_RBBM_SECVID_TSB_CNTL, A810_CP_SQE_CNTL, A810_CP_SQE_INSTR_BASE, A810_CP_SQE_INSTR_BASE + 1, A810_CP_RB_BASE,
    A810_CP_RB_RPTR, A810_CP_RB_WPTR, A810_CP_APERTURE_CNTL_HOST, A810_CP_INTERRUPT_STATUS_GLOBAL,
};
struct gpb_survey {
    int done;               // 0: an access failed (io), nothing after it was read
    uint32_t fuse, freq_limit, gpu[GPB_SURVEY_GPU];
};
static inline void gpb_gpu_survey(const struct gpb_gx_io *io, struct gpb_survey *v) {
    __builtin_memset(v, 0, sizeof *v);
    if (io->read32(io->cookie, GPB_WIN_CX, GPB_CX_SW_FUSE_VALUE, &v->fuse) ||
        io->read32(io->cookie, GPB_WIN_CX, GPB_CX_SW_FUSE_FREQ_LIMIT, &v->freq_limit)) return;
    for (unsigned i = 0; i < GPB_SURVEY_GPU; ++i)
        if (io->read32(io->cookie, GPB_WIN_GPU, GPB_GPU_REG(gpb_survey_gpu[i]), &v->gpu[i])) return;
    v->done = 1;
}
// Mode 13: a8xx hw_init (a8xx_gpu.c, torvalds/linux 734bef548a96) from the aperture clear to the interrupt masks, in its
// order, with the OOB GPU_SET mode 11 holds; everything after (adreno_hw_init, the SQE base, the ring) is mode 14's.
//   1 aperture clear (cached_aperture reset, so CP_APERTURE_CNTL_HOST 0 is written), GBIF_HALT 0 and RBBM_GBIF_HALT 0,
//     each read back; SECVID TSB off, trusted base/size 0
//   2 PERFCTR_GPU_BUSY_MASKED ~0; UCHE_CCHE and SP_HLSQ GMEM range min 64 MiB; the four UCHE trap/write-through bases
//     0x1fffffffff000 (a6xx_gpu_init's uche_trap_base); PERFCTR_CNTL and SLICE_PERFCTR_CNTL 1; GMU XOCLK_1 countable 4;
//     CP_PERFCTR_CP_SEL(0) 1
//   3 a8xx_set_ubwc_config for milos_data (ubwc_config.c): UBWC 4.0, highest bank bit 15, and flags written as
//     UBWC_SWIZZLE_ENABLE_LVL2|LVL3 (bits 1|2), which qcom_ubwc_swizzle reads as DISABLE_SWIZZLE_LVL3 (bit 1): level 2
//     swizzling on, level 3 off. So hbb 2 (lo 2, hi 0), mode tag 2, rgba8888_lossless, fp16compoptdis,
//     rgb565_predicator and amsbc on, ubwc_mode and min_acc_len_64b off, uavflagprd_inv 2
//   4 the 20 bicubic weights; UCHE_CLIENT_PF BIT(7)|1; a8xx_nonctxt_config (a810_nonctxt_regs per pipe NONE..DDE_BV,
//     LPAC skipped, RB_GC_GMEM_PROTECT included as Linux writes it here)
//   5 hang detection (INTERFACE_HANG_INT_CNTL BIT(30)|0xcfffff, SLICE BIT(30)); GMU GPU_CX_BUSY_MASK 0xff000000, power
//     counter XOCLK_0 countable BIT(5) (read-modify-write of the low byte) and POWER_COUNTER_ENABLE 1
//   6 a8xx_set_cp_protect (CNTL 0xffff000b on BR and BV, the a810_protect list, its last value to CP_PROTECT_PIPE(15)
//     on BR and BV); RB_CONTEXT_SWITCH_GMEM_SAVE_RESTORE_ENABLE 1 on BR
//   7 per pipe BR, BV, AQE0, AQE1, DDE_BR, DDE_BV: APRIV (BR 0x6f, others 0xf), CP interrupt and HW fault masks
//   8 aperture clear; CP_INTERRUPT_STATUS_MASK_GLOBAL 0x7f007f; RBBM_INT_0_MASK 0x33d283c2
// The aperture is written only when it changes, as a8xx_aperture_slice_set caches it. Then it reads back a few written
// registers (gpb_hw_check) and the status words.
#define GPB_GMEM_RANGE_MIN 0x4000000ull         // SZ_64M
#define GPB_UCHE_TRAP_BASE 0x1fffffffff000ull
#define GPB_UBWC_GRAS 0x50u                     // hbb 2 << 5 | level3 off << 4
#define GPB_UBWC_RB_CCU 0x24u                   // level3 off << 5 | hbb_lo 2 << 1
#define GPB_UBWC_RB_CMP 0x1001eu                // mode 2 << 15 | rgba8888 << 4 | fp16 << 3 | rgb565 << 2 | amsbc << 1
#define GPB_UBWC_SP 0x2024u                     // level3 off << 13 | uavflagprd_inv 2 << 4 | hbb_lo 2 << 1
#define GPB_UBWC_TPL1 0x84u                     // level3 off << 7 | hbb_lo 2 << 1
#define GPB_HW_CHECKS 8u
struct gpb_hw {
    unsigned step;          // last step started, 1-8; 9 when all finished
    unsigned failed;        // 0 ok, 3 I/O
    unsigned writes, aperture_writes, protect_writes, nonctxt_writes, mismatches;
    uint32_t gbif_halt, rbbm_gbif_halt, xoclk0, protect_last;
    uint32_t check[GPB_HW_CHECKS];   // read back, in gpb_hw_check order
    uint32_t rbbm, int_status, cp_global;
};
// {window, offset, the value written}: registers outside the aperture, read back after step 8.
static const uint32_t gpb_hw_check[GPB_HW_CHECKS][3] = {
    {GPB_WIN_GPU, GPB_GPU_REG(A810_RBBM_PERFCTR_GPU_BUSY_MASKED), 0xffffffffu},
    {GPB_WIN_GPU, GPB_GPU_REG(A810_UCHE_TRAP_BASE), 0xfffff000u},
    {GPB_WIN_GPU, GPB_GPU_REG(A810_UCHE_TRAP_BASE + 1), 0x1ffffu},
    {GPB_WIN_GPU, GPB_GPU_REG(A810_TPL1_BICUBIC_WEIGHTS_TABLE(1)), 0x3fe05ff4u},
    {GPB_WIN_GPU, GPB_GPU_REG(A810_UCHE_CLIENT_PF), A810_INIT_UCHE_CLIENT_PF},
    {GPB_WIN_GPU, GPB_GPU_REG(A810_CP_PROTECT_GLOBAL(0)), 0x0e8c0000u},
    {GPB_WIN_GPU, GPB_GPU_REG(A810_RBBM_INT_0_MASK), A810_INIT_RBBM_INT_0_MASK},
    {GPB_WIN_GMU, GPB_GMU_REG(A810_GPU_GMU_AO_GPU_CX_BUSY_MASK), A810_INIT_GPU_GMU_AO_GPU_CX_BUSY_MASK},
};
static inline void gpb_hw_init(const struct gpb_gx_io *io, struct gpb_hw *h) {
    __builtin_memset(h, 0, sizeof *h);
    uint32_t aperture = ~0u;   // cached_aperture = UINT_MAX
#define GPB_HW_W(w, off, v) do {++h->writes; if (io->write32(io->cookie, (w), (off), (v))) {h->failed = 3; return;}} while (0)
#define GPB_HW_R(w, off, p) do {if (io->read32(io->cookie, (w), (off), (p))) {h->failed = 3; return;}} while (0)
#define GPB_GPU_W(dword, v) GPB_HW_W(GPB_WIN_GPU, GPB_GPU_REG(dword), (v))
#define GPB_GPU_W64(dword, v) do {GPB_GPU_W((dword), (uint32_t)(v)); GPB_GPU_W((dword) + 1, (uint32_t)((v) >> 32));} while (0)
#define GPB_PIPE(p) do {const uint32_t a_ = (uint32_t)(p) << A810_CP_APERTURE_CNTL_HOST_PIPEID_SHIFT; \
    if (aperture != a_) {aperture = a_; ++h->aperture_writes; GPB_GPU_W(A810_CP_APERTURE_CNTL_HOST, a_);}} while (0)
    h->step = 1;
    GPB_PIPE(A810_PIPE_NONE);
    GPB_GPU_W(A810_GBIF_HALT, A810_INIT_GBIF_HALT);
    GPB_HW_R(GPB_WIN_GPU, GPB_GPU_REG(A810_GBIF_HALT), &h->gbif_halt);
    GPB_GPU_W(A810_RBBM_GBIF_HALT, A810_INIT_RBBM_GBIF_HALT);
    GPB_HW_R(GPB_WIN_GPU, GPB_GPU_REG(A810_RBBM_GBIF_HALT), &h->rbbm_gbif_halt);
    GPB_GPU_W(A810_RBBM_SECVID_TSB_CNTL, A810_INIT_RBBM_SECVID_TSB_CNTL);
    GPB_GPU_W64(A810_RBBM_SECVID_TSB_TRUSTED_BASE, 0ull);
    GPB_GPU_W(A810_RBBM_SECVID_TSB_TRUSTED_SIZE, A810_INIT_RBBM_SECVID_TSB_TRUSTED_SIZE);
    h->step = 2;
    GPB_GPU_W(A810_RBBM_PERFCTR_GPU_BUSY_MASKED, A810_INIT_RBBM_PERFCTR_GPU_BUSY_MASKED);
    GPB_GPU_W64(A810_UCHE_CCHE_GC_GMEM_RANGE_MIN, GPB_GMEM_RANGE_MIN);
    GPB_GPU_W64(A810_SP_HLSQ_GC_GMEM_RANGE_MIN, GPB_GMEM_RANGE_MIN);
    GPB_GPU_W64(A810_UCHE_TRAP_BASE, GPB_UCHE_TRAP_BASE);
    GPB_GPU_W64(A810_UCHE_WRITE_THRU_BASE, GPB_UCHE_TRAP_BASE);
    GPB_GPU_W64(A810_UCHE_CCHE_TRAP_BASE, GPB_UCHE_TRAP_BASE);
    GPB_GPU_W64(A810_UCHE_CCHE_WRITE_THRU_BASE, GPB_UCHE_TRAP_BASE);
    GPB_GPU_W(A810_RBBM_PERFCTR_CNTL, A810_INIT_RBBM_PERFCTR_CNTL);
    GPB_GPU_W(A810_RBBM_SLICE_PERFCTR_CNTL, A810_INIT_RBBM_SLICE_PERFCTR_CNTL);
    GPB_HW_W(GPB_WIN_GMU, GPB_GMU_REG(A810_GMU_CX_GMU_POWER_COUNTER_SELECT_XOCLK_1), 0x4u);
    GPB_GPU_W(A810_CP_PERFCTR_CP_SEL(0), 1u);
    h->step = 3;
    GPB_PIPE(A810_PIPE_BV);
    GPB_GPU_W(A810_GRAS_NC_MODE_CNTL, GPB_UBWC_GRAS);
    GPB_PIPE(A810_PIPE_BR);
    GPB_GPU_W(A810_GRAS_NC_MODE_CNTL, GPB_UBWC_GRAS);
    GPB_PIPE(A810_PIPE_BR);
    GPB_GPU_W(A810_RB_CCU_NC_MODE_CNTL, GPB_UBWC_RB_CCU);
    GPB_PIPE(A810_PIPE_BR);
    GPB_GPU_W(A810_RB_CMP_NC_MODE_CNTL, GPB_UBWC_RB_CMP);
    GPB_PIPE(A810_PIPE_NONE);
    GPB_GPU_W(A810_SP_NC_MODE_CNTL, GPB_UBWC_SP);
    GPB_GPU_W(A810_TPL1_NC_MODE_CNTL, GPB_UBWC_TPL1);
    h->step = 4;
    for (unsigned i = 0; i < NATIVE_ADRENO_A810_BICUBIC_COUNT; ++i)
        GPB_GPU_W(A810_TPL1_BICUBIC_WEIGHTS_TABLE(i), NativeAdrenoA810Bicubic[i]);
    GPB_GPU_W(A810_UCHE_CLIENT_PF, A810_INIT_UCHE_CLIENT_PF);
    for (unsigned pipe = A810_PIPE_NONE; pipe <= A810_PIPE_DDE_BV; ++pipe) {
        if (pipe == A810_PIPE_LPAC) continue;
        GPB_PIPE(pipe);
        for (unsigned i = 0; i < NATIVE_ADRENO_A810_NONCTXT_COUNT; ++i) {
            if (!((1u << pipe) & NativeAdrenoA810NonCtxt[i][2])) continue;
            ++h->nonctxt_writes;
            GPB_GPU_W(NativeAdrenoA810NonCtxt[i][0], NativeAdrenoA810NonCtxt[i][1]);
        }
    }
    GPB_PIPE(A810_PIPE_NONE);
    h->step = 5;
    GPB_GPU_W(A810_RBBM_INTERFACE_HANG_INT_CNTL, A810_INIT_RBBM_INTERFACE_HANG_INT_CNTL);
    GPB_GPU_W(A810_RBBM_SLICE_INTERFACE_HANG_INT_CNTL, A810_INIT_RBBM_SLICE_INTERFACE_HANG_INT_CNTL);
    GPB_HW_W(GPB_WIN_GMU, GPB_GMU_REG(A810_GPU_GMU_AO_GPU_CX_BUSY_MASK), A810_INIT_GPU_GMU_AO_GPU_CX_BUSY_MASK);
    GPB_HW_R(GPB_WIN_GMU, GPB_GMU_REG(A810_GMU_CX_GMU_POWER_COUNTER_SELECT_XOCLK_0), &h->xoclk0);
    GPB_HW_W(GPB_WIN_GMU, GPB_GMU_REG(A810_GMU_CX_GMU_POWER_COUNTER_SELECT_XOCLK_0), (h->xoclk0 & ~0xffu) | (1u << 5));
    GPB_HW_W(GPB_WIN_GMU, GPB_GMU_REG(A810_GMU_CX_GMU_POWER_COUNTER_ENABLE), A810_INIT_GMU_CX_GMU_POWER_COUNTER_ENABLE);
    h->step = 6;
    GPB_PIPE(A810_PIPE_BR);
    GPB_GPU_W(A810_CP_PROTECT_CNTL_PIPE, A810_CP_PROTECT_CNTL_PIPE_VALUE);
    GPB_PIPE(A810_PIPE_BV);
    GPB_GPU_W(A810_CP_PROTECT_CNTL_PIPE, A810_CP_PROTECT_CNTL_PIPE_VALUE);
    GPB_PIPE(A810_PIPE_NONE);
    for (unsigned i = 0; i < NATIVE_ADRENO_A810_PROTECT_COUNT; ++i) {
        if (!NativeAdrenoA810Protect[i][1]) continue;   // "Intentionally skip writing to some registers"
        ++h->protect_writes;
        GPB_GPU_W(A810_CP_PROTECT_GLOBAL(NativeAdrenoA810Protect[i][0]), NativeAdrenoA810Protect[i][1]);
        h->protect_last = NativeAdrenoA810Protect[i][1];
    }
    GPB_PIPE(A810_PIPE_BR);
    GPB_GPU_W(A810_CP_PROTECT_PIPE(15), h->protect_last);
    GPB_PIPE(A810_PIPE_BV);
    GPB_GPU_W(A810_CP_PROTECT_PIPE(15), h->protect_last);
    GPB_PIPE(A810_PIPE_NONE);
    GPB_PIPE(A810_PIPE_BR);
    GPB_GPU_W(A810_RB_CONTEXT_SWITCH_GMEM_SAVE_RESTORE_ENABLE, 1u);
    h->step = 7;
    for (unsigned pipe = A810_PIPE_BR; pipe <= A810_PIPE_DDE_BV; ++pipe) {
        if (pipe == A810_PIPE_LPAC) continue;
        GPB_PIPE(pipe);
        GPB_GPU_W(A810_CP_APRIV_CNTL_PIPE, pipe == A810_PIPE_BR ? A810_BR_APRIV_MASK_VALUE : A810_APRIV_MASK_VALUE);
        GPB_GPU_W(A810_CP_INTERRUPT_STATUS_MASK_PIPE, A810_INIT_CP_INTERRUPT_STATUS_MASK_PIPE);
        GPB_GPU_W(A810_CP_HW_FAULT_STATUS_MASK_PIPE, A810_INIT_CP_HW_FAULT_STATUS_MASK_PIPE);
    }
    h->step = 8;
    GPB_PIPE(A810_PIPE_NONE);
    GPB_GPU_W(A810_CP_INTERRUPT_STATUS_MASK_GLOBAL, A810_INIT_CP_INTERRUPT_STATUS_MASK_GLOBAL);
    GPB_GPU_W(A810_RBBM_INT_0_MASK, A810_INIT_RBBM_INT_0_MASK);
    for (unsigned i = 0; i < GPB_HW_CHECKS; ++i) {
        GPB_HW_R(gpb_hw_check[i][0], gpb_hw_check[i][1], &h->check[i]);
        if (h->check[i] != gpb_hw_check[i][2]) ++h->mismatches;
    }
    GPB_HW_R(GPB_WIN_GPU, GPB_GPU_REG(A810_RBBM_STATUS), &h->rbbm);
    GPB_HW_R(GPB_WIN_GPU, GPB_GPU_REG(A810_RBBM_INT_0_STATUS), &h->int_status);
    GPB_HW_R(GPB_WIN_GPU, GPB_GPU_REG(A810_CP_INTERRUPT_STATUS_GLOBAL), &h->cp_global);
    h->step = 9;
#undef GPB_PIPE
#undef GPB_GPU_W64
#undef GPB_GPU_W
#undef GPB_HW_R
#undef GPB_HW_W
}

// Mode 15 (T13): the zap shader as PAS image 13, then CP_SET_SECURE_MODE 0, as a8xx hw_init does after cp_init. On the
// A840, pandora's level 11 had init_image, mem_setup and auth_and_reset all return 0, with the metadata in ordinary
// contiguous memory. Secure monitor calls follow the SMC Calling Convention as Qualcomm's monitor takes them: a yielding
// SMC64 SiP call, function (service << 8) | command, x1 the argument description (count in bits 0-3, then a 2-bit type
// per argument from bit 4: 0 value, 2 read-write buffer), the arguments from x2; x0 = 1 (interrupted) asks for the call
// again with x0 = 1 and the returned x6. Status in x0, the first result in x1.
#define GPB_SCM_SVC_PIL 0x02u
#define GPB_SCM_SVC_INFO 0x06u
#define GPB_SCM_PAS_INIT_IMAGE 0x01u
#define GPB_SCM_PAS_MEM_SETUP 0x02u
#define GPB_SCM_PAS_AUTH_AND_RESET 0x05u
#define GPB_SCM_PAS_IS_SUPPORTED 0x07u
#define GPB_SCM_INFO_IS_CALL_AVAIL 0x01u
#define GPB_SCM_INTERRUPTED 1u
#define GPB_SCM_ARGS_1 0x01u                    // one value
#define GPB_SCM_ARGS_3 0x03u                    // three values
#define GPB_SCM_ARGS_INIT_IMAGE 0x82u           // a value, then a read-write buffer
#define GPB_ZAP_PAS 13u                         // the GPU's PAS id (a6xx_zap_shader_init, pandora's level 11)
#define GPB_ZAP_REGION 0x9ea9a000ULL            // the 3a DT's gpu_microcode_region (no-map), kgsl-3d0/zap-shader's region
#define GPB_ZAP_REGION_BYTES 0x2000u
#define GPB_CP_SET_SECURE_MODE 0x66u            // PM4 type 7 opcode, one dword: 0 leaves secure mode
static inline uint32_t gpb_scm_fn(uint32_t svc, uint32_t cmd) {return 0x42000000u | (svc & 0xffu) << 8 | (cmd & 0xffu);}
// is_call_avail's argument: the SiP-owned function number without the call-type bits.
static inline uint32_t gpb_scm_query(uint32_t svc, uint32_t cmd) {return 0x02000000u | (svc & 0xffu) << 8 | (cmd & 0xffu);}

// A Qualcomm signed image (mbn) is an ELF32 file: its program headers include one whose segment type (p_flags bits
// 24-26) is 2, the hash table and signature, and PT_LOAD segments that are the image (relocatable when p_flags bit 27 is
// set). The monitor takes the ELF and program headers followed by the hash segment as metadata; each load segment goes
// into the region at its offset from the lowest load address.
#define GPB_MDT_MAX_LOADS 4u
struct gpb_mdt {
    uint32_t header_bytes, hash_offset, hash_bytes, meta_bytes;
    uint32_t min_addr, max_addr, loads, relocatable;
    uint32_t offset[GPB_MDT_MAX_LOADS], file_bytes[GPB_MDT_MAX_LOADS], mem_bytes[GPB_MDT_MAX_LOADS], addr[GPB_MDT_MAX_LOADS];
};
static inline uint32_t gpb_le32(const uint8_t *p) {return (uint32_t)p[0] | (uint32_t)p[1] << 8 | (uint32_t)p[2] << 16 | (uint32_t)p[3] << 24;}
static inline uint32_t gpb_le16(const uint8_t *p) {return (uint32_t)p[0] | (uint32_t)p[1] << 8;}
// 0 when the image fits a region of region_bytes, else the first rule it breaks: 1 not ELF, 2 not 32-bit little endian,
// 3 program headers out of the file, 4 a segment out of the file, 5 two hash segments, 6 a bad or extra load segment,
// 7 no hash segment, 8 no load segment, 9 the loads span more than the region.
static inline unsigned gpb_mdt_parse(const uint8_t *img, uint32_t bytes, uint32_t region_bytes, struct gpb_mdt *m) {
    __builtin_memset(m, 0, sizeof *m);
    if (bytes < 0x34 || img[0] != 0x7f || img[1] != 'E' || img[2] != 'L' || img[3] != 'F') return 1;
    if (img[4] != 1 || img[5] != 1) return 2;
    const uint32_t phoff = gpb_le32(img + 0x1c), phentsize = gpb_le16(img + 0x2a), phnum = gpb_le16(img + 0x2c);
    if (phentsize != 32 || !phnum || phnum > 16 || phoff > bytes || phnum * 32u > bytes - phoff) return 3;
    m->header_bytes = phoff + phnum * 32u;
    m->min_addr = ~0u;
    for (uint32_t i = 0; i < phnum; ++i) {
        const uint8_t *p = img + phoff + 32u * i;
        const uint32_t type = gpb_le32(p), off = gpb_le32(p + 4), addr = gpb_le32(p + 12), file = gpb_le32(p + 16),
                       mem = gpb_le32(p + 20), flags = gpb_le32(p + 24);
        if (off > bytes || file > bytes - off) return 4;
        if (((flags >> 24) & 7u) == 2u) {
            if (m->hash_bytes) return 5;
            m->hash_offset = off; m->hash_bytes = file;
            continue;
        }
        if (type != 1u || !mem) continue;
        if (file > mem || m->loads == GPB_MDT_MAX_LOADS || addr > ~0u - mem - 0xfffu) return 6;
        m->offset[m->loads] = off; m->file_bytes[m->loads] = file; m->mem_bytes[m->loads] = mem; m->addr[m->loads] = addr;
        ++m->loads;
        if (flags & (1u << 27)) m->relocatable = 1;
        if (addr < m->min_addr) m->min_addr = addr;
        /* The span ends on a 4K page (PAS mem_setup refuses 0x378 for the 3a's zap with 0xffcfffba). */
        if (addr + mem > m->max_addr) m->max_addr = (addr + mem + 0xfffu) & ~0xfffu;
    }
    if (!m->hash_bytes) return 7;
    if (!m->loads) return 8;
    if (m->max_addr - m->min_addr > region_bytes) return 9;
    m->meta_bytes = m->header_bytes + m->hash_bytes;
    return 0;
}
// The metadata: headers, then the hash segment (m->meta_bytes into out).
static inline void gpb_mdt_metadata(const uint8_t *img, const struct gpb_mdt *m, uint8_t *out) {
    for (uint32_t i = 0; i < m->header_bytes; ++i) out[i] = img[i];
    for (uint32_t i = 0; i < m->hash_bytes; ++i) out[m->header_bytes + i] = img[m->hash_offset + i];
}
// The load segments at their offsets in the region, the rest of each segment's memory size zeroed.
static inline void gpb_mdt_place(const uint8_t *img, const struct gpb_mdt *m, volatile uint8_t *region) {
    for (uint32_t s = 0; s < m->loads; ++s) {
        volatile uint8_t *to = region + (m->addr[s] - m->min_addr);
        for (uint32_t i = 0; i < m->mem_bytes[s]; ++i) to[i] = i < m->file_bytes[s] ? img[m->offset[s] + i] : 0;
    }
}

// Mode 16 (T14): the GPU writes a bar into the firmware framebuffer. One IB of CP_MEM_WRITE packets, one per row, each
// carrying the row's pixels; the framebuffer pages under the bar are mapped into the GPU's context. GPU addresses stay below
// the 16 MiB GMEM window, after the kernel buffers (0x100000-0x302fff).
#define GPB_IB_IOVA 0x400000ULL          // the one-packet IB, at pandora level 12's address
#define GPB_BAR_IB_IOVA 0x500000ULL      // the bar's IB
#define GPB_BAR_IB_MAX 0x100000u
#define GPB_FB_IOVA 0x800000ULL          // the framebuffer pages under the bar
#define GPB_FB_MAX 0x800000u
#define GPB_BAR_ROWS 48u
#define GPB_BAR_EDGE 4u                  // white rows at the top and the bottom of the bar
#define GPB_CP_MEM_WRITE 0x3du           // PM4 type 7: address lo, hi, then the dwords to write
#define GPB_CP_INDIRECT_BUFFER 0x3fu     // PM4 type 7: address lo, hi, size in dwords
struct gpb_bar {
    uint64_t page, bytes, offset;        // page-aligned framebuffer span under the bar, and the bar's first byte in it
    uint32_t y0, rows, width, row_bytes, ib_dwords, ib_bytes;
};
// 0, or the rule that refused: 1 not a 32-bit console, 2 width outside 3..16381 (one packet per row), 3 fewer rows than
// the bar, 4 rows narrower than width pixels, 5 span over GPB_FB_MAX, 6 IB over GPB_BAR_IB_MAX. The bar starts two
// thirds down the screen.
static inline int gpb_bar_plan(uint64_t base, uint32_t width, uint32_t height, uint32_t row_bytes, uint32_t depth,
                               struct gpb_bar *b) {
    b->page = b->bytes = b->offset = 0;
    b->y0 = b->rows = b->width = b->row_bytes = b->ib_dwords = b->ib_bytes = 0;
    if (!base || depth != 32) return 1;
    if (width < 3 || width > 16381) return 2;
    if (height < GPB_BAR_ROWS * 3) return 3;
    if (row_bytes < width * 4u) return 4;
    b->y0 = height / 3 * 2; b->rows = GPB_BAR_ROWS; b->width = width; b->row_bytes = row_bytes;
    const uint64_t first = base + (uint64_t)b->y0 * row_bytes;
    const uint64_t end = first + (uint64_t)(b->rows - 1) * row_bytes + width * 4u;
    b->page = first & ~0xfffULL;
    b->bytes = ((end + 0xfffULL) & ~0xfffULL) - b->page;
    b->offset = first - b->page;
    if (b->bytes > GPB_FB_MAX) return 5;
    b->ib_dwords = b->rows * (3u + width);
    b->ib_bytes = (b->ib_dwords * 4u + 0xfffu) & ~0xfffu;
    if (b->ib_bytes > GPB_BAR_IB_MAX) return 6;
    return 0;
}
// The pixel at (x, row): white edge rows, else red, green and blue thirds (x8r8g8b8).
static inline uint32_t gpb_bar_pixel(const struct gpb_bar *b, uint32_t x, uint32_t row) {
    if (row < GPB_BAR_EDGE || row >= b->rows - GPB_BAR_EDGE) return 0x00ffffffu;
    static const uint32_t thirds[3] = {0x00ff0000u, 0x0000ff00u, 0x000000ffu};
    return thirds[(uint64_t)x * 3 / b->width];
}
// The bar's IB: per row the CP_MEM_WRITE header (the caller's PM4 encoding of opcode 0x3d with width + 2 dwords), the
// row's GPU address in the mapped span at fb_iova, then its pixels.
static inline void gpb_bar_fill(const struct gpb_bar *b, uint32_t header, uint64_t fb_iova, uint32_t *ib) {
    uint32_t k = 0;
    for (uint32_t row = 0; row < b->rows; ++row) {
        const uint64_t at = fb_iova + b->offset + (uint64_t)row * b->row_bytes;
        ib[k++] = header; ib[k++] = (uint32_t)at; ib[k++] = (uint32_t)(at >> 32);
        for (uint32_t x = 0; x < b->width; ++x) ib[k++] = gpb_bar_pixel(b, x, row);
    }
}

// Mode 17 (T15): the BO pool. XNU sees only its owned-memory window (board.toml; 4.75 GiB in the high bank since T19), and
// this RAM is reserved in the UEFI memory map (uefi-asteroids GPU_Pool, the top of the bank), so the window never covers it:
// the loader keeps it, and the probe adopts it for NativeAdreno's BOs (pandora's patch 0178 gets the same from
// its loader through the ADT). The range is in the phone's DDR above 4 GiB: Android's /proc/iomem lists
// 0x8bf800000-0xaffffefff as System RAM, and the live DT's reserved-memory has no static region above 4 GiB (only
// mem_dump_region, placed by Linux at boot). The GPU's 36 output bits reach it. 1 GiB since T18b, as pandora's pool: T18's
// 128 MiB ran out at the Metal runtime's 64 MiB blocks, and XNU's window has no such contiguous run. The probe maps it whole
// and halves the size down to 128 MiB if the map fails (pandora's 1 GiB in one ml_io_map panicked; ours is an IOKit map);
// board.toml [gpu] gpu_pool_* must match.
#define GPB_GPU_POOL_BASE 0xABFE00000ULL
#define GPB_GPU_POOL_BYTES 0x40000000ULL
// The client-visible facts for the A810, each from a pinned source: chip id, GMEM 576 KiB from NativeAdrenoProfile's
// a810 entry (mainline a6xx_catalog.c); GMEM window and trap base from mode 13's writes; UBWC from milos_data (UBWC 4.0,
// highest bank bit 15, level 2 swizzling only). The VA window is uke's: clear of the kernel buffers, the bar's maps and
// the GMEM window, up to the 2 GiB aperture. macrotile_mode is not checked yet (0); only Turnip reads it.
#define GPB_A810_CHIP_ID 0x44010000u
#define GPB_A810_GMEM_BYTES 0x90000u
#define GPB_CLIENT_VA_START 0x20000000ULL
#define GPB_CLIENT_VA_BYTES 0x60000000ULL
#define GPB_UBWC_HBB 15u
#define GPB_UBWC_SWIZZLE 2u               // bit1: level 2 on; levels 1 and 3 off
#define GPB_UBWC_VERSION 4u
/* The open type NativeAdreno serves (NativeAdrenoPluginOpenType, 'AG'): any other type is refused as a foreign IOGPU open with
 * kIOReturnUnsupported before a client is mounted (T15 opened with type 0 and got e00002c7 three times). */
#define GPB_UC_OPEN_TYPE 0x4147u
#endif
