/* SPDX-License-Identifier: BSD-2-Clause-Patent */
/* From UniXNU-sheng package/ShengGmuVotes.c (b0ccf61522383c5e4dda1091252c5534213aeba5), for the Adreno 810: SKU bins,
 * the A8xx GPU_PERF table with MX dependency votes (a8xx_hfi_send_perf_table, a6xx_gmu_rpmh_dep_votes_init) and the
 * A8xx fixed-BCM rule. */
#include "ShengGmuVotes.h"
#include "ShengGmuVotesSource.h"
static const ShengGmuGpuBin *selected_bin;
/* adreno_of_get_pwrlevels: the first bin, in device-tree order, listing the SoC code or 0 (which matches any). */
static const ShengGmuGpuBin *bin_for(const ShengGmuVoteInputs *in)
{
    for(unsigned i=0;i<SHENG_GPU_BINS;++i)
        for(unsigned j=0;j<sheng_gpu_bins[i].sku_count;++j) {
            const uint32_t code=sheng_gpu_bins[i].skus[j];
            if(!code || (in->sku_present && code==in->sku_code))return &sheng_gpu_bins[i];
        }
    return 0;
}
AdrenoHfiStartupStatus ShengGmuBuildStartup(const ShengGmuVoteInputs *in,AdrenoHfiStartupConfig *out)
{
    if(!in || !out || in->chip_id!=SHENG_GMU_VOTES_CHIP_ID || in->gmxc_present>1 ||
        in->enable_acd || in->enable_ifpc>1 || in->sku_present>1 ||
        (!in->gmxc_present && (in->gmxc.data || in->gmxc.bytes)))return AdrenoHfiStartupInvalid;
    const ShengGmuGpuBin *bin=bin_for(in);
    if(!bin)return AdrenoHfiStartupInvalid;
    AdrenoHfiStartupConfig c={0};uint32_t votes[16],levels[16];
    /* a6xx_gmu_rpmh_votes_init: GMxC, when cmd-db has it, is the secondary rail of the GX votes. */
    const ShengGmuAux *gpu_secondary=in->gmxc_present?&in->gmxc:&in->mx;
    for(unsigned i=0;i<bin->opp_count;++i)levels[i]=bin->opps[i].level;
    if(AdrenoHfiArcVotes(levels,bin->opp_count,in->gfx.data,in->gfx.bytes,
        gpu_secondary->data,gpu_secondary->bytes,votes,16))return AdrenoHfiStartupInvalid;
    c.gpu_count=bin->opp_count;
    for(unsigned i=0;i<c.gpu_count;++i)c.gpu[i]=(AdrenoHfiPerformanceLevel){bin->opps[i].hz,votes[i]};
    if(AdrenoHfiDependencyVotes(levels,bin->opp_count,in->mx.data,in->mx.bytes,c.gpu_dependency,16))
        return AdrenoHfiStartupInvalid;
    c.table_format=1;
    for(unsigned i=0;i<SHENG_GMU_OPPS;++i)levels[i]=sheng_gmu_opps[i].level;
    if(AdrenoHfiArcVotes(levels,SHENG_GMU_OPPS,in->cx.data,in->cx.bytes,
        in->mx.data,in->mx.bytes,votes,16))return AdrenoHfiStartupInvalid;
    c.gmu_count=SHENG_GMU_OPPS;
    for(unsigned i=0;i<c.gmu_count;++i)c.gmu[i]=(AdrenoHfiPerformanceLevel){sheng_gmu_opps[i].hz,votes[i]};
    const ShengGmuBcmInput *inputs[]={&in->sh0,&in->mc0,&in->acv};AdrenoHfiBcm bcms[3];
    for(unsigned i=0;i<3;++i) {
        bcms[i]=sheng_bcm_policy[i];bcms[i].address=inputs[i]->address;
        if(AdrenoHfiBcmAux(bcms+i,inputs[i]->aux.data,inputs[i]->aux.bytes))return AdrenoHfiStartupInvalid;
    }
    if(AdrenoHfiBuildBandwidth(bin->bandwidths,bin->bandwidth_count,bcms,3,in->cn0_address,&c))
        return AdrenoHfiStartupInvalid;
    c.enable_ifpc=in->enable_ifpc;
    if(AdrenoHfiStartupValidate(&c))return AdrenoHfiStartupInvalid;
    *out=c;selected_bin=bin;return AdrenoHfiStartupSuccess;
}
unsigned ShengGmuPerfVote(unsigned perf,unsigned *bandwidth)
{
    const ShengGmuGpuBin *bin=selected_bin;
    if(!bin) {if(bandwidth)*bandwidth=0;return 0;}
    if(perf<1)perf=1;
    if(perf>=bin->opp_count)perf=bin->opp_count-1;
    unsigned bw=1;
    for(unsigned i=1;i<bin->bandwidth_count;++i)if(bin->bandwidths[i]==bin->opps[perf].kbps){bw=i;break;}
    if(bandwidth)*bandwidth=bw;
    return perf;
}
