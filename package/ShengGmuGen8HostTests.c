/* SPDX-License-Identifier: BSD-2-Clause-Patent */
/* Host test of the gen8 (A810) GMU cold start in ShengGmu.c, over simulated windows, with the pinned gen80300_gmu.bin
 * and the generated build/sheng-gmu/ShengGmuSource.h. The simulated GMU only posts what the real one reports: the
 * firmware-init value of a6xx_gmu_start's version rule once CM3 leaves reset, and HFI ready after HFI_CTRL_INIT.
 * It proves the sequence the code issues, not hardware behaviour. Build: scripts/check-asteroids-gmu.py. */
#include "ShengGmu.h"
#include "ShengGmuCommandDb.h"
#include "ShengGmuCommandDbSource.h"
#include "ShengGmuSource.h"
#include <CommonCrypto/CommonDigest.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#define CHECK(x) do {if(!(x)){fprintf(stderr,"line %d: %s\n",__LINE__,#x);exit(1);}}while(0)
typedef struct {unsigned window;size_t at;uint32_t value;} Access;
typedef struct {
    uint32_t *mem[ShengGmuWindows];
    Access writes[200000];size_t write_count;
    unsigned reads[ShengGmuWindows];
    uint32_t inject_fault;   /* AO fault status posted once CM3 leaves reset */
    volatile uint32_t *hfi;  /* the HFI queues, for the simulated firmware's ACKs */
    unsigned messages,ids[8],words[8];uint32_t payload[8][160];
} Sim;
static Sim sim;
static uint64_t clock_us;
static size_t core(uint32_t word){return (size_t)word*4-SHENG_GMU_CORE_GPU_DELTA;}
static int sim_read(void *cookie,unsigned window,size_t at,uint32_t *v)
{
    CHECK(cookie==&sim && window<ShengGmuWindows);
    if(!sheng_gmu_window_bytes[window] || (at&3) || at+4>sheng_gmu_window_bytes[window])return 1;
    ++sim.reads[window];*v=sim.mem[window][at/4];return 0;
}
static int sim_write(void *cookie,unsigned window,size_t at,uint32_t v)
{
    CHECK(cookie==&sim && window<ShengGmuWindows);
    if(!sheng_gmu_window_bytes[window] || (at&3) || at+4>sheng_gmu_window_bytes[window])return 1;
    CHECK(sim.write_count<sizeof(sim.writes)/sizeof(sim.writes[0]));
    sim.writes[sim.write_count++]=(Access){window,at,v};
    sim.mem[window][at/4]=v;
    if(window==ShengGmuCore && at==core(REG_GMU_CM3_SYSRESET) && v==0) {
        /* a6xx_gmu_start: the firmware reports 0xbabeface (version <= 0x20010004) or 0x100 in the low nine bits. */
        const uint32_t version=sim.mem[ShengGmuCore][core(REG_GMU_CM3_DTCM_START+0xff8)/4];
        sim.mem[ShengGmuCore][core(REG_GMU_CM3_FW_INIT_RESULT)/4]=version<=UINT32_C(0x20010004)?0xbabeface:0x100;
        sim.mem[ShengGmuCore][core(REG_GMU_AO_HOST_INTERRUPT_STATUS)/4]=sim.inject_fault;
    }
    if(window==ShengGmuCore && at==core(REG_GMU_HFI_CTRL_INIT) && v==1)
        sim.mem[ShengGmuCore][core(REG_GMU_HFI_CTRL_STATUS)/4]=1;
    if(window==ShengGmuCore && at==core(REG_GMU_HOST2GMU_INTR_SET) && v==1 && sim.hfi) {
        /* The firmware's side of the queues, as the kit's test_startup peer: take the command, ACK it. */
        volatile uint32_t *shared=sim.hfi;
        unsigned read=shared[16];uint32_t header=shared[1024+read];unsigned n=(header>>8)&255;
        CHECK(sim.messages<8 && n>=1 && n<=160);
        unsigned index=sim.messages++;sim.ids[index]=header&255;sim.words[index]=n-1;
        for(unsigned i=1;i<n;++i)sim.payload[index][i-1]=shared[1024+((read+i)&1023)];
        shared[16]=shared[17];
        volatile uint32_t *q=shared+18;unsigned write=q[11];
        shared[2048+write]=(3u<<8)|126u;shared[2048+((write+1)&1023)]=header;
        shared[2048+((write+2)&1023)]=0;shared[2048+((write+3)&1023)]=0xfafafafa;
        q[11]=(write+4)&1023;
    }
    return 0;
}
static int sim_barrier(void *cookie){CHECK(cookie==&sim);return 0;}
static int sha256(void *cookie,const void *data,size_t bytes,uint8_t digest[32])
{(void)cookie;CC_SHA256(data,(CC_LONG)bytes,digest);return 0;}
static int now_us(void *cookie,uint64_t *v){(void)cookie;*v=clock_us;clock_us+=50;return 0;}
static int wait_us(void *cookie,unsigned us){(void)cookie;clock_us+=us;return 0;}
static ShengGmuMemory region(size_t bytes,uint64_t iova)
{
    void *p=aligned_alloc(4096,bytes);CHECK(p);memset(p,0xa5,bytes);
    return (ShengGmuMemory){p,bytes,iova};
}
static void reset_sim(void)
{
    for(unsigned i=0;i<ShengGmuWindows;++i) {
        free(sim.mem[i]);sim.mem[i]=NULL;
        if(sheng_gmu_window_bytes[i])CHECK((sim.mem[i]=calloc(1,sheng_gmu_window_bytes[i])));
        sim.reads[i]=0;
    }
    sim.write_count=0;sim.inject_fault=0;sim.hfi=NULL;sim.messages=0;
    sim.mem[ShengGmuCore][core(REG_GMU_CM3_SYSRESET)/4]=1;   /* held in reset, as the caller must leave it */
}
static const Access *last_write(unsigned window,size_t at)
{
    for(size_t i=sim.write_count;i--;)if(sim.writes[i].window==window && sim.writes[i].at==at)return &sim.writes[i];
    return NULL;
}
static size_t writes_to(unsigned window)
{
    size_t n=0;for(size_t i=0;i<sim.write_count;++i)if(sim.writes[i].window==window)++n;return n;
}
static uint32_t le32(const uint8_t *p){return (uint32_t)p[0]|(uint32_t)p[1]<<8|(uint32_t)p[2]<<16|(uint32_t)p[3]<<24;}
int main(int argc,char **argv)
{
    CHECK(argc==2 && SHENG_GMU_GENERATION==8);
    FILE *f=fopen(argv[1],"rb");CHECK(f);
    static uint8_t image[SHENG_GMU_FIRMWARE_BYTES+1];
    CHECK(fread(image,1,sizeof(image),f)==SHENG_GMU_FIRMWARE_BYTES);fclose(f);
    ShengGmuConfig config={SHENG_GMU_CHIP_ID,image,SHENG_GMU_FIRMWARE_BYTES,
        region(0xffc000,0x4000),region(0x4000,0x60000000),region(0x7000,0x60400000),
        region(0x4000,0x60500000),region(0x4000,0x60600000),64};
    const ShengGmuServices services={NULL,sha256,now_us,wait_us};

    /* The full load through HFI ready, on gen8. */
    reset_sim();
    ShengGmuDevice device={0};
    device.io=(ShengGmuIo){&sim,{0},sim_read,sim_write,sim_barrier};
    for(unsigned i=0;i<ShengGmuWindows;++i)device.io.bytes[i]=sheng_gmu_window_bytes[i];
    device.services=services;
    ShengGmuResult result;
    CHECK(ShengGmuLoadAndStartHfi(&device,&config,&result)==ShengGmuSuccess);
    CHECK(result.phase==7 && result.complete_hash_verified && result.firmware_ready_observed && result.hfi_ready_observed);
    /* No PDC window, no RSC setup, no TCM retention: the RSCC is only read, CX misc is untouched. */
    CHECK(!writes_to(ShengGmuRscc) && !writes_to(ShengGmuCxMisc) && !sim.reads[ShengGmuCxMisc]);
    CHECK(sim.reads[ShengGmuRscc]==1);   /* RSCC_SEQ_BUSY_DRV0, idle before anything is written */
    /* The gen8 configuration words at their A8XX offsets. */
    const Access *w;
    CHECK((w=last_write(ShengGmuCore,core(REG_GPU_GMU_CX_GMU_CX_FALNEXT_INTF))) && w->value==1 && REG_GPU_GMU_CX_GMU_CX_FALNEXT_INTF==0x1f7ed);
    CHECK((w=last_write(ShengGmuCore,core(REG_GPU_GMU_CX_GMU_CX_FAL_INTF))) && w->value==1 && REG_GPU_GMU_CX_GMU_CX_FAL_INTF==0x1f7ec);
    CHECK((w=last_write(ShengGmuCore,core(REG_GMU_AHB_FENCE_RANGE_0))) && w->value==UINT32_C(0x80c808c0));
    CHECK((w=last_write(ShengGmuCore,core(REG_GMU_GENERAL_10))) && w->value==UINT32_C(0x8030000) && REG_GMU_GENERAL_10==0x1f9ca);
    CHECK((w=last_write(ShengGmuCore,core(REG_GMU_GENERAL_8))) && w->value==(UINT32_C(0x60500000)|3));
    CHECK((w=last_write(ShengGmuCore,core(REG_GMU_GENERAL_9))) && w->value==0 && REG_GMU_GENERAL_9==0x1f9c9);
    CHECK((w=last_write(ShengGmuCore,core(REG_GMU_MRC_GBIF_QOS_CTRL))) && w->value==0x33);
    CHECK((w=last_write(ShengGmuCore,core(REG_GMU_CM3_CFG))) && w->value==0x4052);
    CHECK((w=last_write(ShengGmuCore,core(REG_GMU_HFI_QTBL_ADDR))) && w->value==0x60600000);
    /* The GPU window gets exactly the GBIF CX list, in order. */
    size_t gpu=0;
    for(size_t i=0;i<sim.write_count;++i)if(sim.writes[i].window==ShengGmuGpu) {
        CHECK(gpu<SHENG_GMU_GBIF_CX_COUNT && sim.writes[i].at==sheng_gmu_gbif_cx[gpu].byte_offset &&
            sim.writes[i].value==sheng_gmu_gbif_cx[gpu].value);
        ++gpu;
    }
    CHECK(gpu==6 && sheng_gmu_gbif_cx[0].value==0x71e20 && sheng_gmu_gbif_cx[5].value==0x20023000);
    /* Payloads: ITCM from the record at 0, the icache from the record at 0x4000. */
    size_t at=0,record=0;
    while(at<sizeof(image)-1) {
        const uint32_t address=le32(image+at),bytes=le32(image+at+4);
        if(bytes && address==0x4000)
            CHECK(!memcmp((const void *)config.icache.cpu,image+at+16,bytes));
        if(bytes && address==0)
            CHECK(!memcmp(sim.mem[ShengGmuCore]+core(REG_GMU_CM3_ITCM_START)/4,image+at+16,bytes));
        at+=16+bytes;++record;
    }
    CHECK(record==SHENG_GMU_FIRMWARE_BLOCKS);
    /* CM3 left reset last among the configuration writes, after which HFI init was the only write. */
    CHECK(sim.writes[sim.write_count-1].at==core(REG_GMU_HFI_CTRL_INIT) &&
        sim.writes[sim.write_count-2].at==core(REG_GMU_CM3_SYSRESET) && sim.writes[sim.write_count-2].value==0);

    /* An AO fault after release fails the start and reasserts reset. */
    reset_sim();
    ShengGmuDevice faulted={0};
    faulted.io=device.io;faulted.services=services;
    sim.inject_fault=SHENG_GMU_AO_FATAL_MASK&(0u-SHENG_GMU_AO_FATAL_MASK);   /* lowest fatal bit */
    CHECK(ShengGmuLoadAndStartHfi(&faulted,&config,&result)==ShengGmuFault && faulted.poisoned);
    CHECK(sim.writes[sim.write_count-1].at==core(REG_GMU_CM3_SYSRESET) && sim.writes[sim.write_count-1].value==1);
    CHECK(result.reset_verified);

    /* A command-db image in the format ShengGmuCommandDbInputs reads: gfx/mx/cx rails (slave 3), SH0/MC0/ACV/CN0
     * (slave 5). Software fixture values, not the phone's. */
    static uint8_t db[SHENG_GMU_COMMAND_DB_BYTES];
    memset(db,0,sizeof(db));
    #define PUT16(at,v) do{db[(at)]=(uint8_t)(v);db[(at)+1]=(uint8_t)((v)>>8);}while(0)
    #define PUT32(at,v) do{PUT16((at),(v)&0xffff);PUT16((at)+2,(uint32_t)(v)>>16);}while(0)
    PUT32(0,1);PUT32(4,0x0c0330db);
    PUT16(8,3);PUT16(10,0);PUT16(12,0x100);PUT16(14,3);       /* ARC: entries at 144, aux at 400 */
    PUT16(24,5);PUT16(26,72);PUT16(28,0x200);PUT16(30,4);     /* BCM: entries at 216, aux at 656 */
    static const char *names[7]={"gfx.lvl","mx.lvl","cx.lvl","SH0","MC0","ACV","CN0"};
    static const uint32_t addresses[7]={0x30000,0x30010,0x30020,0x50000,0x50004,0x50008,0x5000c};
    static const uint16_t gfx[11]={0,56,64,128,192,224,256,320,384,416,432},mx_rail[10]={0,48,64,128,192,256,320,384,416,432},
        cx_rail[4]={0,64,128,192};
    const uint16_t *rails[3]={gfx,mx_rail,cx_rail};const unsigned rail_count[3]={11,10,4},rail_at[3]={0,32,64};
    for(unsigned i=0;i<7;++i) {
        const unsigned entry=i<3?144+24*i:216+24*(i-3);
        memcpy(db+entry,names[i],strlen(names[i]));PUT32(entry+16,addresses[i]);
        if(i<3) {PUT16(entry+20,2*rail_count[i]);PUT16(entry+22,rail_at[i]);
            for(unsigned j=0;j<rail_count[i];++j)PUT16(400+rail_at[i]+2*j,rails[i][j]);}
        else if(i<6) {PUT16(entry+20,8);PUT16(entry+22,8*(i-3));
            PUT32(656+8*(i-3),1000);PUT16(656+8*(i-3)+4,1);db[656+8*(i-3)+6]=(uint8_t)(i-3);}
        else {PUT16(entry+20,0);PUT16(entry+22,24);}
    }
    ShengGmuVoteInputs inputs;memset(&inputs,0,sizeof(inputs));
    CHECK(ShengGmuCommandDbInputs(db,sizeof(db),1,&inputs)==ShengGmuDbSuccess && !inputs.gmxc_present);
    inputs.chip_id=SHENG_GMU_CHIP_ID;
    AdrenoHfiStartupConfig startup_config;
    /* Votes: no SKU, an unknown SKU and ACD refuse; SKU 0x3 is bin 0 (11 levels), SKU 0x2 bin 1 (10 levels). */
    CHECK(ShengGmuBuildStartup(&inputs,&startup_config)==AdrenoHfiStartupInvalid);
    inputs.sku_present=1;inputs.sku_code=0x7;
    CHECK(ShengGmuBuildStartup(&inputs,&startup_config)==AdrenoHfiStartupInvalid);
    inputs.sku_code=0x2;
    CHECK(ShengGmuBuildStartup(&inputs,&startup_config)==AdrenoHfiStartupSuccess && startup_config.gpu_count==10);
    inputs.sku_code=0x3;inputs.enable_acd=1;
    CHECK(ShengGmuBuildStartup(&inputs,&startup_config)==AdrenoHfiStartupInvalid);
    inputs.enable_acd=0;
    CHECK(ShengGmuBuildStartup(&inputs,&startup_config)==AdrenoHfiStartupSuccess);
    CHECK(startup_config.table_format==1 && startup_config.gpu_count==11 && startup_config.gmu_count==3);
    CHECK(startup_config.gpu_dependency[0]==UINT32_MAX);
    CHECK(startup_config.gpu_dependency[1]==((0x3fffu<<14)|(2u<<8)|0xff));    /* corner 56 -> mx 64 at index 2 */
    CHECK(startup_config.gpu_dependency[10]==((0x3fffu<<14)|(9u<<8)|0xff));   /* corner 432 -> mx index 9 */
    CHECK(startup_config.gpu[10].hz==1150000000ULL && startup_config.gmu[2].hz==650000000ULL);
    /* ACV votes perfmode (4) on every nonzero bandwidth, including the 2136718 kB/s level under perfmode_bw. */
    CHECK(startup_config.ddr_data[0][2]==0x40000000u && startup_config.ddr_data[1][2]==0x60000004u);
    unsigned bw=0;CHECK(ShengGmuPerfVote(10,&bw)==10 && bw==7);

    /* ColdStart through the HFI startup: the GPU_PERF table first, then the bandwidth table, core start, start. */
    reset_sim();
    ShengGmuDevice cold={0};cold.io=device.io;cold.services=services;
    sim.hfi=config.hfi.cpu;
    const ShengGmuStartupLimits limits={16,10,100000,1000000};
    ShengGmuStartupResult startup;
    CHECK(ShengGmuColdStart(&cold,&config,&startup_config,&limits,&startup)==ShengGmuSuccess);
    CHECK(startup.protocol_status==AdrenoHfiStartupSuccess && startup.protocol.stage==AdrenoHfiStageComplete);
    CHECK(sim.messages==4 && sim.ids[0]==15 && sim.ids[1]==3 && sim.ids[2]==14 && sim.ids[3]==10);
    CHECK(sim.words[0]==6+3*11+2*3 && sim.payload[0][1]==1 && sim.payload[0][2]==11 && sim.payload[0][3]==3);
    CHECK(sim.payload[0][8]==startup_config.gpu_dependency[1] && sim.payload[0][9]==264000);
    CHECK(sim.payload[0][37]==3 && sim.payload[0][38]==2 && sim.payload[0][40]==0 && sim.payload[0][42]==350000);
    CHECK(sim.words[1]==159 && sim.payload[1][31+8+2]==0x60000004u);
    /* An A7xx-format table is refused on gen8 before the device is touched. */
    reset_sim();
    ShengGmuDevice a7={0};a7.io=device.io;a7.services=services;
    AdrenoHfiStartupConfig wrong=startup_config;wrong.table_format=0;memset(wrong.gpu_dependency,0,sizeof(wrong.gpu_dependency));
    CHECK(ShengGmuColdStart(&a7,&config,&wrong,&limits,&startup)==ShengGmuArgument);
    CHECK(!sim.write_count && !sim.reads[ShengGmuCore] && !a7.boot_attempted);

    /* MMIO binding: the absent PDC window must be passed empty. */
    ShengGmuMmio mmio={0};
    for(unsigned i=0;i<ShengGmuWindows;++i)if(sheng_gmu_window_bytes[i])
        mmio.windows[i]=(ShengGmuMmioWindow){(volatile unsigned char *)sim.mem[i],sheng_gmu_window_physical[i],
            sheng_gmu_window_bytes[i]};
    ShengGmuIo bound;
    CHECK(ShengGmuBindMmio(&mmio,&bound)==ShengGmuSuccess && bound.bytes[ShengGmuPdc]==0);
    uint32_t value=0;
    CHECK(bound.read32(bound.cookie,ShengGmuPdc,0,&value)!=0);
    mmio.windows[ShengGmuPdc]=(ShengGmuMmioWindow){(volatile unsigned char *)sim.mem[ShengGmuCxMisc],0x3d90000,0x1000};
    CHECK(ShengGmuBindMmio(&mmio,&bound)==ShengGmuArgument);
    printf("PASS: gen8 GMU load to HFI ready with the pinned firmware (%zu records): no RSC/PDC or TCM retention, A8XX "
        "FAL/GENERAL offsets, fence 0x80c808c0, GBIF CX list and MRC QoS; fault reset; cmd-db votes by SKU bin; ColdStart through the A8xx "
        "GPU_PERF table; A7xx table refused; PDC absent\n",
        record);
    return 0;
}
