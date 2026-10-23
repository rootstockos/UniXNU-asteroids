/* SPDX-License-Identifier: BSD-2-Clause-Patent */
/* From UniXNU-sheng package/ShengGmu.c (fb2d4ed7ae02), with the gen8 (A8xx) branches of mainline a6xx_gmu.c selected by
 * the generated SHENG_GMU_GENERATION (scripts/prepare-asteroids-gmu.py). A header without it is the sheng A740 one and
 * gets the unchanged gen7 sequence. Gen8 differences, each from an a8xx branch of a6xx_gmu.c:
 *   - no RSC/PDC setup (a6xx_gmu_rpmh_init returns early: AOP owns it) and no PDC window;
 *   - no TCM retention (neither CX_MISC_TCM_RET_CNTL nor GENERAL_7), so it is neither checked nor set;
 *   - the A8XX FAL interface, GENERAL_8/9/10 offsets and AHB fence range (generated);
 *   - the GBIF CX list of the catalog entry, then GBIF_CX_CONFIG again and GMU_MRC_GBIF_QOS_CTRL, instead of the four
 *     QSB writes;
 *   - the HFI startup's performance table must be the A8xx GPU_PERF table (AdrenoFirmware table_format 1, built by
 *     ShengGmuBuildStartup); ShengGmuColdStart refuses any other before touching the device. */
#include "ShengGmu.h"
#include "AdrenoFirmware.h"
/* Generated only from exact pinned XML/catalog/DT/firmware policy; ignored.
 * In particular, no vendor firmware or sequencer payload is stored in Git. */
#include "ShengGmuSource.h"
#ifndef SHENG_GMU_GENERATION
#define SHENG_GMU_GENERATION 7u
#endif
#ifndef SHENG_GMU_FIRMWARE_BLOCKS
#define SHENG_GMU_FIRMWARE_BLOCKS 15u
#endif
#ifndef SHENG_GMU_AHB_FENCE_RANGE
#define SHENG_GMU_AHB_FENCE_RANGE (UINT32_C(0x80000000)|(0x32u<<18)|0x8a0)
#endif
#if SHENG_GMU_GENERATION>=8 && (!defined(SHENG_GMU_GBIF_CX_COUNT) || !defined(SHENG_GMU_MRC_GBIF_QOS))
#error gen8 needs the generated GBIF CX list and MRC QoS value
#endif
/* Host tests run the sequence over simulated windows; only the real build is tied to AArch64. */
#if !defined(SHENG_GMU_HOST_TEST) && (!defined(__aarch64__) || __BYTE_ORDER__ != __ORDER_LITTLE_ENDIAN__)
#error sheng GMU requires little-endian AArch64
#endif
#if defined(SHENG_GMU_HOST_TEST) && __BYTE_ORDER__ != __ORDER_LITTLE_ENDIAN__
#error sheng GMU requires little-endian
#endif
static int extent(uintptr_t at,size_t bytes)
{return at && bytes && bytes-1<=UINTPTR_MAX-at;}
static int overlap(uintptr_t a,size_t na,uintptr_t b,size_t nb)
{return a<=b?(b-a<na):(a-b<nb);}
static size_t core_offset(uint32_t word)
{return ((size_t)word<<2)-SHENG_GMU_CORE_GPU_DELTA;}
static int read32(ShengGmuDevice *d,unsigned window,size_t at,uint32_t *v,ShengGmuResult *r)
{++r->read_attempts;return d->io.read32(d->io.cookie,window,at,v);}
static int write32(ShengGmuDevice *d,unsigned window,size_t at,uint32_t v,ShengGmuResult *r)
{++r->write_attempts;return d->io.write32(d->io.cookie,window,at,v);}
static int core_read(ShengGmuDevice *d,uint32_t word,uint32_t *v,ShengGmuResult *r)
{return read32(d,ShengGmuCore,core_offset(word),v,r);}
static int core_write(ShengGmuDevice *d,uint32_t word,uint32_t v,ShengGmuResult *r)
{return write32(d,ShengGmuCore,core_offset(word),v,r);}
static ShengGmuStatus check_faults(ShengGmuDevice *d,ShengGmuResult *r)
{
    if(core_read(d,REG_GMU_AO_HOST_INTERRUPT_STATUS,&r->ao_fault_status,r) ||
        core_read(d,REG_GMU_GMU2HOST_INTR_INFO,&r->cm3_fault_status,r))return ShengGmuIoError;
    return (r->ao_fault_status&SHENG_GMU_AO_FATAL_MASK) ||
        (r->cm3_fault_status&SHENG_GMU_CM3_FATAL_MASK)?ShengGmuFault:ShengGmuSuccess;
}
static int notify(void *cookie)
{
    ShengGmuDevice *d=cookie;
    return d->io.barrier(d->io.cookie) ||
        d->io.write32(d->io.cookie,ShengGmuCore,core_offset(REG_GMU_HOST2GMU_INTR_SET),1) ||
        d->io.barrier(d->io.cookie);
}
static ShengGmuStatus poll(ShengGmuDevice *d,uint32_t reg,uint32_t mask,
    uint32_t expected,unsigned budget,uint32_t *observed,ShengGmuResult *r)
{
    uint64_t start,last,now;
    if(d->services.now_us(d->services.cookie,&start))return ShengGmuClockError;
    last=start;
    for(unsigned i=0;i<budget;++i) {
        if(d->services.now_us(d->services.cookie,&now) || now<last)return ShengGmuClockError;
        if(now-start>=10000)return ShengGmuTimeout;
        last=now;++r->poll_reads;
        ShengGmuStatus fault=check_faults(d,r);if(fault!=ShengGmuSuccess)return fault;
        if(core_read(d,reg,observed,r))return ShengGmuIoError;
        if((*observed&mask)==expected) {
            if(d->io.barrier(d->io.cookie))return ShengGmuIoError;
            return ShengGmuSuccess;
        }
        if(i+1<budget && d->services.wait_us(d->services.cookie,100))return ShengGmuClockError;
    }
    return ShengGmuTimeout;
}
static void reset_after_failure(ShengGmuDevice *d,ShengGmuResult *r)
{
    uint32_t actual;r->reset_verified=0;d->poisoned=1;d->hfi.poisoned=1;
    if(!core_write(d,REG_GMU_CM3_SYSRESET,1,r) && !d->io.barrier(d->io.cookie) &&
        !core_read(d,REG_GMU_CM3_SYSRESET,&actual,r) && actual==1)r->reset_verified=1;
}
static uint32_t le32(const uint8_t *p)
{return (uint32_t)p[0]|(uint32_t)p[1]<<8|(uint32_t)p[2]<<16|(uint32_t)p[3]<<24;}
static ShengGmuStatus validate(ShengGmuDevice *d,const ShengGmuConfig *c,ShengGmuResult *r)
{
    if(!d || !c || !d->io.read32 || !d->io.write32 || !d->io.barrier ||
        !d->services.sha256 || !d->services.now_us || !d->services.wait_us ||
        c->chip_id!=SHENG_GMU_CHIP_ID || !c->firmware || c->firmware_bytes!=SHENG_GMU_FIRMWARE_BYTES ||
        !extent((uintptr_t)c->firmware,c->firmware_bytes) || !c->poll_budget || c->poll_budget>1024)
        return ShengGmuArgument;
    if(d->poisoned)return ShengGmuPoisoned;
    if(d->boot_attempted)return ShengGmuBusy;
    for(unsigned i=0;i<ShengGmuWindows;++i)if(d->io.bytes[i]!=sheng_gmu_window_bytes[i])return ShengGmuArgument;
    const ShengGmuMemory *mem[]={&c->icache,&c->dummy,&c->debug,&c->log,&c->hfi};
    const size_t sizes[]={0xffc000,0x4000,0x7000,0x4000,0x4000};
    for(unsigned i=0;i<5;++i) {
        const ShengGmuMemory *m=mem[i];
        if(m->bytes!=sizes[i] || !extent((uintptr_t)m->cpu,m->bytes) || ((uintptr_t)m->cpu&4095) ||
            !m->iova || (m->iova&4095) || m->iova>UINT32_MAX-(m->bytes-1) ||
            overlap((uintptr_t)m->cpu,m->bytes,(uintptr_t)c->firmware,c->firmware_bytes))return ShengGmuArgument;
        for(unsigned j=0;j<i;++j)if(overlap((uintptr_t)m->cpu,m->bytes,(uintptr_t)mem[j]->cpu,mem[j]->bytes) ||
            overlap((uintptr_t)m->iova,m->bytes,(uintptr_t)mem[j]->iova,mem[j]->bytes))return ShengGmuArgument;
    }
    if(c->icache.iova!=0x4000 || c->dummy.iova!=0x60000000 || c->debug.iova!=0x60400000)return ShengGmuArgument;
    if(d->services.sha256(d->services.cookie,c->firmware,c->firmware_bytes,r->complete_sha256))return ShengGmuFirmware;
    for(unsigned i=0;i<32;++i)if(r->complete_sha256[i]!=sheng_gmu_firmware_sha256[i])return ShengGmuFirmware;
    r->complete_hash_verified=1;return ShengGmuSuccess;
}
ShengGmuStatus ShengGmuLoadAndStartHfi(ShengGmuDevice *d,const ShengGmuConfig *c,ShengGmuResult *r)
{
    if(!r)return ShengGmuArgument;
    *r=(ShengGmuResult){0};r->resources_must_retain=1;
    ShengGmuStatus status=validate(d,c,r);if(status!=ShengGmuSuccess)return status;
    const AdrenoFirmwareRegion regions[]={{0,0x4000,1},{0x4000,0xffc000,2},{0x10004000,0x4000,3}};
    AdrenoFirmwareBlock blocks[32];size_t count=0;
    if(AdrenoFirmwarePlanGmu(c->firmware,c->firmware_bytes,regions,3,blocks,32,&count)!=AdrenoFirmwareSuccess ||
        count!=SHENG_GMU_FIRMWARE_BLOCKS)
        return ShengGmuFirmware;
    r->phase=1;uint32_t value;
    if(core_read(d,REG_GMU_CM3_SYSRESET,&value,r))return ShengGmuIoError;
    if(value!=1)return ShengGmuBusy;
    r->reset_verified=1;
#if SHENG_GMU_GENERATION<8
    if(read32(d,ShengGmuCxMisc,REG_CX_MISC_TCM_RET_CNTL*4,&value,r))return ShengGmuIoError;
    if(value)return ShengGmuStateMismatch;
#endif
    if(read32(d,ShengGmuRscc,REG_RSCC_SEQ_BUSY_DRV0*4,&value,r))return ShengGmuIoError;
    if(value)return ShengGmuBusy;
    if(core_read(d,REG_GMU_HFI_CTRL_STATUS,&value,r))return ShengGmuIoError;
    if(value&1)return ShengGmuStateMismatch; /* A stale ready bit cannot pass. */
    status=check_faults(d,r);if(status!=ShengGmuSuccess)return status;
    r->phase=2;
    d->boot_attempted=1;
#if SHENG_GMU_RSC_SETUP_COUNT
    for(unsigned i=0;i<SHENG_GMU_RSC_SETUP_COUNT;++i) {
        const ShengGmuSetupWrite *w=sheng_gmu_rsc_setup+i;
        if(write32(d,w->window,w->byte_offset,w->value,r)) {status=ShengGmuIoError;goto failed;}
    }
#endif
    if(d->io.barrier(d->io.cookie)) {status=ShengGmuIoError;goto failed;}
    if(core_write(d,REG_GPU_GMU_CX_GMU_CX_FALNEXT_INTF,1,r) ||
        core_write(d,REG_GPU_GMU_CX_GMU_CX_FAL_INTF,1,r)) {status=ShengGmuIoError;goto failed;}
#if SHENG_GMU_GENERATION<8
    if(write32(d,ShengGmuCxMisc,REG_CX_MISC_TCM_RET_CNTL*4,1,r)) {status=ShengGmuIoError;goto failed;}
    if(d->io.barrier(d->io.cookie) || read32(d,ShengGmuCxMisc,REG_CX_MISC_TCM_RET_CNTL*4,&value,r)) {status=ShengGmuIoError;goto failed;}
    if(value!=1) {status=ShengGmuStateMismatch;goto failed;}
#else
    if(d->io.barrier(d->io.cookie)) {status=ShengGmuIoError;goto failed;}
#endif
    /* All shared allocations are real owned coherent memory, not simulated
     * GMU addresses. Zero only after actual stopped/idle checks above. */
    const ShengGmuMemory *memory[]={&c->icache,&c->dummy,&c->debug,&c->log};
    r->memory_changed=1;
    for(unsigned m=0;m<4;++m)for(size_t i=0;i<memory[m]->bytes/4;++i)memory[m]->cpu[i]=0;
    if(AdrenoHfiInitializeStopped(&d->hfi,c->hfi.cpu,c->hfi.bytes,c->hfi.iova,d,notify)!=AdrenoHfiSuccess) {
        status=ShengGmuArgument;goto failed;
    }
    r->phase=3;
    for(size_t i=0;i<count;++i) {
        const AdrenoFirmwareBlock *b=blocks+i;
        /* Zero-size records authorize no allocation or write. The pinned
         * loader skips them; all real extents were independently supplied. */
        if(!b->bytes)continue;
        for(size_t at=0;at<b->bytes;at+=4) {
            uint32_t word=le32(c->firmware+b->payload_offset+at);
            if(b->region_identifier==2)c->icache.cpu[(b->address-0x4000+at)/4]=word;
            else {
                uint32_t reg=b->region_identifier==1?REG_GMU_CM3_ITCM_START+b->address/4:
                    REG_GMU_CM3_DTCM_START+(b->address-0x10004000)/4;
                if(core_write(d,reg+(uint32_t)(at/4),word,r)) {status=ShengGmuIoError;goto failed;}
            }
        }
    }
    if(d->io.barrier(d->io.cookie)) {status=ShengGmuIoError;goto failed;}
    /* Read every TCM payload word before releasing reset. CPU cache readback
     * checks copied bytes only; actual GMU coherency is still a prerequisite. */
    for(size_t i=0;i<count;++i) {
        const AdrenoFirmwareBlock *b=blocks+i;
        for(size_t at=0;at<b->bytes;at+=4) {
            if(b->region_identifier==2)value=c->icache.cpu[(b->address-0x4000+at)/4];
            else {
                uint32_t reg=b->region_identifier==1?REG_GMU_CM3_ITCM_START+b->address/4:
                    REG_GMU_CM3_DTCM_START+(b->address-0x10004000)/4;
                if(core_read(d,reg+(uint32_t)(at/4),&value,r)) {status=ShengGmuIoError;goto failed;}
            }
            if(value!=le32(c->firmware+b->payload_offset+at)) {status=ShengGmuStateMismatch;goto failed;}
        }
    }
    r->phase=4;
    if(core_write(d,REG_GMU_CM3_FW_INIT_RESULT,0,r) ||
        core_write(d,REG_GMU_CM3_BOOT_CONFIG,2,r) ||
        core_write(d,REG_GMU_HFI_QTBL_ADDR,(uint32_t)c->hfi.iova,r) ||
        core_write(d,REG_GMU_HFI_QTBL_INFO,1,r) ||
        core_write(d,REG_GMU_AHB_FENCE_RANGE_0,SHENG_GMU_AHB_FENCE_RANGE,r) ||
        core_write(d,REG_GMU_CM3_CFG,0x4052,r) ||
        core_write(d,REG_GMU_GENERAL_10,SHENG_GMU_FW_CHIP_ID,r) ||
        core_write(d,REG_GMU_GENERAL_8,(uint32_t)c->log.iova|3,r)) {status=ShengGmuIoError;goto failed;}
#if SHENG_GMU_GENERATION<8
    for(unsigned i=0;i<4;++i)if(write32(d,ShengGmuGpu,(REG_GBIF_QSB_SIDE0+i)*4,0x71620,r)) {status=ShengGmuIoError;goto failed;}
#else
    for(unsigned i=0;i<SHENG_GMU_GBIF_CX_COUNT;++i) {
        const ShengGmuSetupWrite *w=sheng_gmu_gbif_cx+i;
        if(write32(d,w->window,w->byte_offset,w->value,r)) {status=ShengGmuIoError;goto failed;}
    }
    if(core_write(d,REG_GMU_MRC_GBIF_QOS_CTRL,SHENG_GMU_MRC_GBIF_QOS,r)) {status=ShengGmuIoError;goto failed;}
#endif
    if(core_write(d,REG_GMU_SYS_BUS_CONFIG,1,r) || core_write(d,REG_GMU_ICACHE_CONFIG,1,r) ||
        core_write(d,REG_GMU_DCACHE_CONFIG,1,r) || core_read(d,REG_GMU_CM3_DTCM_START+0xff8,&value,r)) {status=ShengGmuIoError;goto failed;}
    uint32_t mask=value<=UINT32_C(0x20010004)?UINT32_MAX:0x1ff;
    uint32_t ready=mask==UINT32_MAX?UINT32_C(0xbabeface):0x100;
    if(core_write(d,REG_GMU_CM3_SYSRESET,1,r) || core_write(d,REG_GMU_GENERAL_9,0,r) ||
        d->io.barrier(d->io.cookie)) {status=ShengGmuIoError;goto failed;}
    const uint32_t config_regs[]={REG_GMU_CM3_FW_INIT_RESULT,REG_GMU_CM3_BOOT_CONFIG,
        REG_GMU_HFI_QTBL_ADDR,REG_GMU_HFI_QTBL_INFO,REG_GMU_CM3_CFG,REG_GMU_GENERAL_10,
        REG_GMU_GENERAL_8,REG_GMU_AHB_FENCE_RANGE_0,REG_GMU_SYS_BUS_CONFIG,
        REG_GMU_ICACHE_CONFIG,REG_GMU_DCACHE_CONFIG};
    const uint32_t config_values[]={0,2,(uint32_t)c->hfi.iova,1,0x4052,SHENG_GMU_FW_CHIP_ID,
        (uint32_t)c->log.iova|3,SHENG_GMU_AHB_FENCE_RANGE,1,1,1};
    for(unsigned i=0;i<sizeof(config_regs)/sizeof(config_regs[0]);++i) {
        if(core_read(d,config_regs[i],&value,r)) {status=ShengGmuIoError;goto failed;}
        if(value!=config_values[i]) {status=ShengGmuStateMismatch;goto failed;}
    }
    r->reset_verified=0;
    if(core_write(d,REG_GMU_CM3_SYSRESET,0,r) || d->io.barrier(d->io.cookie)) {status=ShengGmuIoError;goto failed;}
    r->phase=5;
    status=poll(d,REG_GMU_CM3_FW_INIT_RESULT,mask,ready,c->poll_budget,&r->firmware_init_result,r);
    if(status!=ShengGmuSuccess)goto failed;
    r->firmware_ready_observed=1;
    if(core_write(d,REG_GMU_HFI_CTRL_INIT,1,r) || d->io.barrier(d->io.cookie)) {status=ShengGmuIoError;goto failed;}
    r->phase=6;
    status=poll(d,REG_GMU_HFI_CTRL_STATUS,1,1,c->poll_budget,&r->hfi_status,r);
    if(status!=ShengGmuSuccess)goto failed;
    status=check_faults(d,r);if(status!=ShengGmuSuccess)goto failed;
    if(d->io.barrier(d->io.cookie)) {status=ShengGmuIoError;goto failed;}
    r->hfi_ready_observed=1;r->phase=7;return ShengGmuSuccess;
failed:
    reset_after_failure(d,r);return status;
}
typedef struct {
    ShengGmuDevice *device;
    ShengGmuStartupResult *result;
} StartupContext;
static int startup_now(void *cookie,uint64_t *value)
{
    ShengGmuDevice *d=((StartupContext *)cookie)->device;
    return d->services.now_us(d->services.cookie,value);
}
static int startup_wait(void *cookie,unsigned us)
{
    ShengGmuDevice *d=((StartupContext *)cookie)->device;
    return d->services.wait_us(d->services.cookie,us);
}
static int startup_health(void *cookie)
{
    StartupContext *c=cookie;ShengGmuDevice *d=c->device;
    ShengGmuStartupResult *r=c->result;uint32_t reset,status;
    r->health_status=check_faults(d,&r->load);
    if(r->health_status!=ShengGmuSuccess)return 1;
    if(core_read(d,REG_GMU_CM3_SYSRESET,&reset,&r->load) ||
        core_read(d,REG_GMU_HFI_CTRL_STATUS,&status,&r->load))r->health_status=ShengGmuIoError;
    else if(reset!=0 || !(status&1))r->health_status=ShengGmuStateMismatch;
    return r->health_status!=ShengGmuSuccess;
}
ShengGmuStatus ShengGmuColdStart(ShengGmuDevice *d,const ShengGmuConfig *c,
    const AdrenoHfiStartupConfig *config,const ShengGmuStartupLimits *limits,ShengGmuStartupResult *r)
{
    if(!r)return ShengGmuArgument;
    *r=(ShengGmuStartupResult){0};r->load.resources_must_retain=r->protocol.resources_must_retain=1;
    r->protocol_status=AdrenoHfiStartupInvalid; /* Not attempted until the load succeeds. */
    if(!d || !c || !limits || AdrenoHfiStartupValidate(config)!=AdrenoHfiStartupSuccess)return ShengGmuArgument;
#if SHENG_GMU_GENERATION>=8
    /* a8xx_hfi_send_perf_table: the A8xx GPU_PERF table (AdrenoFirmware table_format 1); refuse an A7xx table. */
    if(config->table_format!=1)return ShengGmuArgument;
#endif
    StartupContext context={d,r};
    AdrenoHfiStartupServices services={&context,startup_now,startup_wait,startup_health,
        limits->polls_per_command,limits->wait_interval_us,limits->command_timeout_us,limits->total_timeout_us};
    if(AdrenoHfiStartupValidateServices(&services)!=AdrenoHfiStartupSuccess)return ShengGmuArgument;
    if(d->poisoned || d->startup.poisoned)return ShengGmuPoisoned;
    if(d->startup.attempted || d->startup.completed)return ShengGmuBusy;
    ShengGmuStatus status=ShengGmuLoadAndStartHfi(d,c,&r->load);
    if(status!=ShengGmuSuccess)return status;
    r->protocol_status=AdrenoHfiStartupRun(&d->startup,&d->hfi,config,&services,&r->protocol);
    if(r->protocol_status==AdrenoHfiStartupSuccess)return ShengGmuSuccess;
    switch(r->protocol_status) {
    case AdrenoHfiStartupClockError:case AdrenoHfiStartupWaitError:status=ShengGmuClockError;break;
    case AdrenoHfiStartupTimeout:status=ShengGmuTimeout;break;
    case AdrenoHfiStartupPlatformFault:status=r->health_status;break;
    default:status=ShengGmuProtocolError;break;
    }
    reset_after_failure(d,&r->load);d->startup.poisoned=1;
    return status;
}
static int fits(ShengGmuMmio *m,unsigned window,size_t at)
{
    if(!m || window>=ShengGmuWindows)return 0;
    ShengGmuMmioWindow *w=m->windows+window;
    return extent((uintptr_t)w->cpu,w->bytes) && !((uintptr_t)w->cpu&3) && !(at&3) &&
        at<=w->bytes && w->bytes-at>=4;
}
static int mmio_read(void *cookie,unsigned window,size_t at,uint32_t *v)
{ShengGmuMmio *m=cookie;if(!v || !fits(m,window,at))return 1;*v=*(volatile uint32_t *)(m->windows[window].cpu+at);return 0;}
static int mmio_write(void *cookie,unsigned window,size_t at,uint32_t v)
{ShengGmuMmio *m=cookie;if(!fits(m,window,at))return 1;*(volatile uint32_t *)(m->windows[window].cpu+at)=v;return 0;}
static int mmio_barrier(void *cookie)
{
    (void)cookie;
#ifdef SHENG_GMU_HOST_TEST
    __atomic_thread_fence(__ATOMIC_SEQ_CST);
#else
    __asm__ volatile("dsb sy":::"memory");
#endif
    return 0;
}
ShengGmuStatus ShengGmuBindMmio(ShengGmuMmio *m,ShengGmuIo *out)
{
    if(!out)return ShengGmuArgument;
    *out=(ShengGmuIo){0};
    if(!m)return ShengGmuArgument;
    for(unsigned i=0;i<ShengGmuWindows;++i) {
        const ShengGmuMmioWindow *w=m->windows+i;
        /* A window the board does not have (A8xx PDC) is pinned as size 0 and must be passed empty. */
        if(!sheng_gmu_window_bytes[i]) {
            if(w->cpu || w->physical || w->bytes)return ShengGmuArgument;
            continue;
        }
        if(!fits(m,i,0) || w->physical!=sheng_gmu_window_physical[i] ||
            w->bytes!=sheng_gmu_window_bytes[i])return ShengGmuArgument;
        for(unsigned j=0;j<i;++j)if(overlap((uintptr_t)w->cpu,w->bytes,(uintptr_t)m->windows[j].cpu,m->windows[j].bytes))return ShengGmuArgument;
    }
    out->cookie=m;out->read32=mmio_read;out->write32=mmio_write;out->barrier=mmio_barrier;
    for(unsigned i=0;i<ShengGmuWindows;++i)out->bytes[i]=m->windows[i].bytes;
    return ShengGmuSuccess;
}
