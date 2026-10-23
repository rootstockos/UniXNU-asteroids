/* SPDX-License-Identifier: BSD-2-Clause-Patent */
/* From UniXNU-sheng package/ShengGmuVotes.h (b0ccf61522383c5e4dda1091252c5534213aeba5), for the Adreno 810: the GPU
 * levels come in SKU bins, so the inputs carry the SKU code (SHENG_GMU_VOTES_HAS_SKU tells callers the field exists). */
#ifndef SHENG_GMU_VOTES_H
#define SHENG_GMU_VOTES_H
#include "AdrenoHfiVotes.h"
#ifdef __cplusplus
extern "C" {
#endif
typedef struct {const uint8_t *data;size_t bytes;} ShengGmuAux;
typedef struct {uint32_t address;ShengGmuAux aux;} ShengGmuBcmInput;
typedef struct {
    uint32_t chip_id;
    ShengGmuAux gfx,mx,cx,gmxc;
    unsigned gmxc_present; /* 0=authoritatively absent, 1=present; unknown must refuse */
    ShengGmuBcmInput sh0,mc0,acv;
    uint32_t cn0_address;
    unsigned enable_acd,enable_ifpc;
    /* asteroids: the SoC code KGSL matches against each bin's qcom,sku-codes ((pcode << 16) | feature code, pcode
     * only for internal parts). Bins are tried in device-tree order and a listed 0 matches any code, as KGSL's
     * adreno_of_get_pwrlevels does; with sku_present 0 only such a bin can match. No other bin is guessed. */
    uint32_t sku_code;
    unsigned sku_present;
} ShengGmuVoteInputs;
#define SHENG_GMU_VOTES_HAS_SKU 1
/* asteroids (A810): the bin listing sku_code, the A8xx GPU_PERF table (table_format 1) with MX dependency votes from
 * the cmd-db mx.lvl rail, and the A8xx fixed-BCM rule from the generated policy. ACD is refused (the A810 catalog
 * entry has none).
 * Build only CPU-side configuration from pinned board OPP/catalog policy and
 * actual cmd-db data supplied by the platform. No device read/write occurs.
 * Missing GMxC data must not silently become absent: discovery errors defer
 * this call. Caller authenticates chip/board/rail/resource ownership and must
 * establish AOSS ACD/IFPC prerequisites before ColdStart, if requested here.
 * Target pinned DT has no OPP speed-bin masks; every enabled OPP is retained.
 * All input/aux/output storage is disjoint and immutable during the call.
 * Failure leaves output unchanged. Software fixtures are not board evidence. */
AdrenoHfiStartupStatus ShengGmuBuildStartup(const ShengGmuVoteInputs *,AdrenoHfiStartupConfig *);
/* The HFI GX_BW_PERF_VOTE indices for GPU OPP `perf` (1 = lowest real level, clamped to the table): the OPP index and the
 * bandwidth-table index of that OPP's opp-peak-kBps, the pairing a6xx_gmu.c votes. Returns the OPP index used.
 * asteroids: of the bin selected by the last successful ShengGmuBuildStartup; 0 (no bin) before that. */
unsigned ShengGmuPerfVote(unsigned perf,unsigned *bandwidth);
#ifdef __cplusplus
}
#endif
#endif
