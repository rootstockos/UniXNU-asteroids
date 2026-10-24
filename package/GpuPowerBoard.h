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

#define GPB_MAX_MODE 6u   // 4 runs modes 1-3, then the GMU pool test below (no SMMU or GMU access); 5 then builds the GMU's
                          // SMMU context on the pool (gpb_gmu_context_gate, NativeAdrenoGmuContextBringup); 6 then reads
                          // the GMU clock tree (gpb_read_gmu_clocks) and the AOP command DB, and writes nothing

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
        if (io->stage) io->stage(io->cookie, GPB_WINDOWS);   // announce the CX power-up before its first write
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
#endif
