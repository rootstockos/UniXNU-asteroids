// SPDX-License-Identifier: BSD-2-Clause-Patent
// asteroids T24b: UniXNU-sheng 0e1f6e5 package/LegacyDisplayProbe.cpp, the probe of UniXNU's display recipe, unchanged except the
// late hook, which starts only the key store and HID providers this board has (no sheng touch, CPU frequency, GPU power, WiFi,
// audio DSP, battery, buttons, backlight or DPU timing probes).
// Bounded offline original lifecycle instrument; no desktop claim.
#include <stdarg.h>
#include <IOKit/IOPlatformExpert.h>
#include <IOKit/IODeviceMemory.h>
#include <IOKit/IODeviceTreeSupport.h>
#include <IOKit/IOLib.h>
#include <IOKit/IOUserClient.h>
#include <libkern/c++/OSMetaClass.h>
#include <libkern/c++/OSSerialize.h>
#include <libkern/c++/OSData.h>
#include <kern/task.h>
#include <sys/proc.h>
#include <pexpert/pexpert.h>
#include <mach/kmod.h>
#include <machine/machine_routines.h>
extern "C" kmod_info_t g_kernel_kmod_info;
extern "C" void native_legacy_display_static_init(void) __asm__("__GLOBAL__sub_I_NativeLegacyDisplay.cpp");
extern "C" IOService *native_legacy_display_allocate(void);
extern "C" void native_gop_presenter_static_init(void) __asm__("__GLOBAL__sub_I_NativeGOPPresenter.cpp");
extern "C" IOService *native_gop_presenter_create(void);
static IOService *pendingLegacyDisplay;
static IOService *activeGOPPresenter,*activeGOPNub;
// GPU hand-off presenter (sheng-fb-gpu=1, phase 4): lives beside the display driver, which offers it frames instead of copying them.
static IOService *gpuHandoffPresenter;
extern "C" bool native_gop_gpu_offer(IOService *,void *display,IOMemoryDescriptor *,uint64_t stride,uint32_t width,uint32_t height);
extern "C" void native_gop_presenter_set_source(IOService *presenter,IOService *nub);
volatile uint32_t native_legacy_gpu_fallbacks,native_legacy_gpu_last_status;
static IOService *activeLegacyDisplay;
// Keep one diagnostic mapping alive until reboot, even after backend stop.
// The pulse is permanently closed before any system-frame copy.
static IOMemoryMap *diagnosticAnchorMapping;
extern "C" int boot_progress_pulse_anchor(uintptr_t,uint64_t,unsigned,unsigned,unsigned) __attribute__((weak_import));
static IORegistryEntry *unitScaleChosen;
static bool unitScaleAdded,unitRotationAdded;
static void finishUnitScalePolicy(bool keep) {
    if(unitScaleChosen && !keep) {
        if(unitScaleAdded)unitScaleChosen->removeProperty("display-scale");
        if(unitRotationAdded)unitScaleChosen->removeProperty("display-rotation");
    }
    OSSafeReleaseNULL(unitScaleChosen);unitScaleAdded=false;unitRotationAdded=false;
}
static bool publishOrMatchWord(IORegistryEntry *entry,const char *key,uint32_t value,bool &added) {
    OSObject *previous=entry->getProperty(key);
    if(previous) {
        OSData *data=OSDynamicCast(OSData,previous);uint32_t word=0;
        if(!data || data->getLength()!=sizeof(word))return false;
        memcpy(&word,data->getBytesNoCopy(),sizeof(word));return word==value;
    }
    OSData *data=OSData::withBytes(&value,sizeof(value));
    bool result=data && entry->setProperty(key,data);OSSafeReleaseNULL(data);added=result;return result;
}
extern "C" {
volatile uint32_t native_legacy_stage;
volatile int native_legacy_meta_status=-1;
volatile uint32_t native_legacy_init_result,native_legacy_start_result,native_legacy_probe_match;
volatile uint32_t native_legacy_parent_match,native_legacy_object_size,native_legacy_released;
volatile uint64_t native_legacy_object,native_legacy_nub,native_legacy_workloop;
volatile uint32_t native_legacy_guard_slot;
volatile uint64_t native_legacy_guard_object,native_legacy_guard_caller;
volatile int native_legacy_surface_status=-1;
volatile uint32_t native_legacy_surface_found;
volatile uint64_t native_legacy_video_base,native_legacy_video_bytes;
volatile uint64_t native_legacy_gop_map,native_legacy_gop_virtual,native_legacy_gop_physical,native_legacy_gop_bytes;
volatile uint32_t native_legacy_gop_width,native_legacy_gop_height;
volatile uint32_t native_legacy_registration_calls,native_legacy_properties_length;
volatile uint32_t native_legacy_dt_attached;
volatile int native_legacy_unit_scale_status=-1;
volatile uint32_t native_legacy_unit_scale_calls,native_legacy_chosen_scale,native_legacy_chosen_rotation;
volatile uint32_t native_legacy_pm_stage,native_legacy_pm_calls,native_legacy_pm_callbacks,native_legacy_pm_ordinal;
volatile int native_legacy_pm_status=-1;
volatile uint32_t native_legacy_swap_validation_calls,native_legacy_swap_validation_result,native_legacy_swap_validation_reason;
volatile uint64_t native_legacy_swap_request,native_legacy_swap_surface;
volatile uint32_t native_legacy_transfer_calls;
volatile int native_legacy_transfer_status=-1;
volatile uint64_t native_legacy_transfer_bytes,native_legacy_transfer_request;
volatile uint32_t native_legacy_retirement_calls,native_legacy_retired_generation,native_legacy_presented_generation,native_legacy_retired_frames;
volatile uint64_t native_legacy_retired_current,native_legacy_retired_pending;
char native_legacy_properties[4096];
volatile uint32_t native_legacy_client_calls,native_legacy_client_type;
volatile int native_legacy_client_pid,native_legacy_client_status=-1;
volatile uint64_t native_legacy_client_task,native_legacy_client_object;
char native_legacy_client_class[64];
extern kern_return_t OSKextLoadKextWithIdentifier(const char *);
extern int boot_progress_prepare_scanout(void) __attribute__((weak_import));
extern void boot_progress_cancel_scanout(void) __attribute__((weak_import));
extern void boot_progress_stage(const char *) __attribute__((weak_import));
volatile uint32_t native_legacy_prepare_calls,native_legacy_prepare_reason,native_legacy_prepare_attempts;
volatile uint32_t native_legacy_prepare_failures,native_legacy_prepare_retried,native_legacy_prepare_max_attempts;
volatile uint64_t native_legacy_prepare_wait_us;
static uint64_t probeNowUs(void) {uint64_t ns;absolutetime_to_nanoseconds(mach_absolute_time(),&ns);return ns/1000;}
bool native_legacy_prepare_scanout(void) {
    native_legacy_prepare_calls=native_legacy_prepare_calls+1;native_legacy_prepare_reason=0;native_legacy_prepare_attempts=0;
    if (!boot_progress_prepare_scanout) return true;
    if (ml_at_interrupt_context()) {native_legacy_prepare_reason=1;return false;}
    if (!ml_get_interrupts_enabled()) {native_legacy_prepare_reason=2;return false;}
    // Yield to any in-flight diagnostic writer; never spin on a preempted
    // holder or claim a frame that the console can still overwrite.
    const uint64_t began=probeNowUs();
    for (unsigned attempt=0;attempt<100;++attempt) {
        native_legacy_prepare_attempts=attempt+1;
        if (boot_progress_prepare_scanout()) {
            if(attempt) {
                native_legacy_prepare_retried=native_legacy_prepare_retried+1;native_legacy_prepare_wait_us=native_legacy_prepare_wait_us+(probeNowUs()-began);
                if(attempt+1>native_legacy_prepare_max_attempts)native_legacy_prepare_max_attempts=attempt+1;
            }
            return true;
        }
        IOSleep(1);
    }
    native_legacy_prepare_failures=native_legacy_prepare_failures+1;native_legacy_prepare_wait_us=native_legacy_prepare_wait_us+(probeNowUs()-began);
    native_legacy_prepare_reason=3;
    if(boot_progress_cancel_scanout)boot_progress_cancel_scanout();
    return false;
}
__attribute__((noinline)) void native_legacy_display_checkpoint(void) {__asm__ volatile("" ::: "memory");}
__attribute__((noinline)) void native_legacy_gop_mapping_checkpoint(void) {__asm__ volatile("" ::: "memory");}
__attribute__((noinline)) void native_legacy_client_checkpoint(void) {__asm__ volatile("" ::: "memory");}
void native_legacy_pm_observed(unsigned stage,IOReturn status,unsigned long ordinal) {
    native_legacy_pm_stage=stage;native_legacy_pm_status=status;native_legacy_pm_ordinal=(uint32_t)ordinal;
    native_legacy_pm_calls=native_legacy_pm_calls+1;
    if(stage==3)native_legacy_pm_callbacks=native_legacy_pm_callbacks+1;
}
__attribute__((noinline)) void native_legacy_swap_validation_checkpoint(bool,unsigned,const void *,const void *) {__asm__ volatile("" ::: "memory");}
// Last rejected swap request, copied from the original request layout the validator reads (offsets
// qualified in NativeLegacyDisplay::validateSwap). Observation only; nothing here changes the verdict.
volatile uint32_t native_legacy_swap_accepted_count,native_legacy_swap_rejected_count,native_legacy_rej_reason;
volatile uint64_t native_legacy_rej_surfaces[4];
volatile uint32_t native_legacy_rej_state[3],native_legacy_rej_options[3],native_legacy_rej_size[2],native_legacy_rej_rects[2][4];
char native_legacy_rej_class[32];
volatile uint64_t native_legacy_rej_surface[7];   // width, height, format, planes, stride, allocation, descriptor bytes of the last rejected surface
void native_legacy_surface_rejected(uint64_t width,uint64_t height,uint32_t format,uint32_t planes,uint64_t stride,uint64_t allocation,uint64_t bytes) {
    native_legacy_rej_surface[0]=width;native_legacy_rej_surface[1]=height;native_legacy_rej_surface[2]=format;native_legacy_rej_surface[3]=planes;
    native_legacy_rej_surface[4]=stride;native_legacy_rej_surface[5]=allocation;native_legacy_rej_surface[6]=bytes;
}
static void recordRejectedSwap(unsigned reason,const void *request,const void *surface) {
    const unsigned char *bytes=reinterpret_cast<const unsigned char *>(request);
    if(!bytes)return;
    native_legacy_rej_reason=reason;
    uint64_t surfaces[4]={};memcpy(surfaces,bytes+2624,sizeof(surfaces));
    for(unsigned i=0;i<4;++i)native_legacy_rej_surfaces[i]=surfaces[i];
    const unsigned stateOffsets[3]={792,796,800},optionOffsets[3]={804,808,812};
    for(unsigned i=0;i<3;++i){uint32_t a=0,b=0;memcpy(&a,bytes+stateOffsets[i],4);memcpy(&b,bytes+optionOffsets[i],4);
        native_legacy_rej_state[i]=a;native_legacy_rej_options[i]=b;}
    uint32_t w=0,h=0;memcpy(&w,bytes+816,4);memcpy(&h,bytes+820,4);native_legacy_rej_size[0]=w;native_legacy_rej_size[1]=h;
    const unsigned rectOffsets[2]={17,81};
    for(unsigned r=0;r<2;++r){uint32_t rect[4]={};memcpy(rect,bytes+rectOffsets[r],sizeof(rect));
        for(unsigned i=0;i<4;++i)native_legacy_rej_rects[r][i]=rect[i];}
    char name[32]={};
    if(surface){const OSObject *object=reinterpret_cast<const OSObject *>(surface);
        const OSMetaClass *meta=object->getMetaClass();const char *cls=meta?meta->getClassName():nullptr;
        if(cls)strlcpy(name,cls,sizeof(name));}
    memcpy(native_legacy_rej_class,name,sizeof(name));
}
// Content statistics of presented frames (copy = kind 0, blank fill = kind 1): the first 32 are kept, with the time
// in seconds since the first frame. A per-frame record shows the animation cadence and when the OS blanks the panel.
#define NATIVE_LEGACY_FRAMES 32
volatile uint32_t native_legacy_frame_count;
volatile uint32_t native_legacy_frame_kind[NATIVE_LEGACY_FRAMES];
volatile uint32_t native_legacy_frame_time[NATIVE_LEGACY_FRAMES];
volatile uint64_t native_legacy_frame_stats[NATIVE_LEGACY_FRAMES][5];
static uint64_t frameEpoch;
void native_legacy_frame_observed(unsigned kind,uint64_t count,uint64_t minx,uint64_t miny,uint64_t maxx,uint64_t maxy) {
    uint32_t n=native_legacy_frame_count;
    uint64_t now,frequency;
    __asm__ volatile("isb\n\tmrs %0, cntpct_el0":"=r"(now));__asm__ volatile("mrs %0, cntfrq_el0":"=r"(frequency));
    if(!frameEpoch)frameEpoch=now;
    if(n<NATIVE_LEGACY_FRAMES) {
        native_legacy_frame_kind[n]=kind;
        native_legacy_frame_time[n]=frequency?(uint32_t)((now-frameEpoch)/frequency):0;
        native_legacy_frame_stats[n][0]=count;native_legacy_frame_stats[n][1]=minx;native_legacy_frame_stats[n][2]=miny;
        native_legacy_frame_stats[n][3]=maxx;native_legacy_frame_stats[n][4]=maxy;
    }
    native_legacy_frame_count=n+1;
}
volatile uint32_t native_legacy_frame_colors[8][4];
volatile uint64_t native_legacy_frame_color_counts[8][4],native_legacy_frame_color_other[8];
// Called right after the matching native_legacy_frame_observed(); fills slot count-1 (or the blank slot).
void native_legacy_frame_colors_observed(const uint32_t *colors,const uint64_t *counts,uint64_t other) {
    uint32_t n=native_legacy_frame_count;
    if(!n || n>8)return;   // dominant colours are kept for the first eight frames only
    n-=1;
    for(unsigned i=0;i<4;++i){native_legacy_frame_colors[n][i]=colors[i];native_legacy_frame_color_counts[n][i]=counts[i];}
    native_legacy_frame_color_other[n]=other;
}
// The life of every accepted swap, from the kernel's side: validated (the client submitted it), offered to the GPU copy (or started on the CPU),
// and retired. Times are microseconds since boot; the last 64 are kept.
struct SwapEvent {uint64_t submitted,offered,done;uint32_t number;};
static SwapEvent swapEvents[64];static uint32_t swapEventCount;
extern "C" void native_legacy_swap_stage(unsigned stage) {
    const uint64_t now=probeNowUs();
    uint32_t total=swapEventCount;
    if(stage==0) {
        SwapEvent &e=swapEvents[total%64];e={now,0,0,total+1};swapEventCount=total+1;return;
    }
    uint32_t first=total>64?total-64:0;
    for(uint32_t i=first;i<total;++i) {
        SwapEvent &e=swapEvents[i%64];
        if(stage==1 && !e.offered && !e.done){e.offered=now;return;}
        if(stage==2 && !e.done){e.done=now;if(!e.offered)e.offered=now;return;}
    }
}
extern "C" unsigned native_legacy_swap_timeline(char *out,unsigned cap) {
    unsigned at=0;out[0]=0;
    uint32_t total=swapEventCount,first=total>40?total-40:0;
    for(uint32_t i=first;i<total && at+90<cap;++i) {
        const SwapEvent &e=swapEvents[i%64];
        uint64_t previous=i>first?swapEvents[(i-1)%64].submitted:e.submitted;
        at+=snprintf(out+at,cap-at,"W%u t=%llu.%03llus +%llums offer=%lldms done=%lldms\n",e.number,(unsigned long long)(e.submitted/1000000),
            (unsigned long long)((e.submitted/1000)%1000),(unsigned long long)((e.submitted-previous)/1000),
            e.offered?(long long)((e.offered-e.submitted)/1000):-1LL,e.done?(long long)((e.done-e.submitted)/1000):-1LL);
    }
    return at;
}
// The display refresh tick (NativeLegacyDisplay, sheng-vsync): vsync messages sent, arms, idle stops, late ticks, starts, refresh
// properties reported at that rate, and the rate.
static volatile uint64_t vsyncEvents[6];
static volatile unsigned vsyncHz;
void native_legacy_vsync_observed(unsigned event,unsigned hz) {
    if(event<6)vsyncEvents[event]=vsyncEvents[event]+1;
    vsyncHz=hz;
}
extern "C" unsigned native_legacy_prepare_report(char *out,unsigned cap) {
    return snprintf(out,cap,"prepare calls=%u failures=%u retried=%u max_attempts=%u waited_ms=%llu vsync hz=%u sent=%llu arms=%llu idle=%llu late=%llu start=%llu refresh=%llu",
        native_legacy_prepare_calls,native_legacy_prepare_failures,native_legacy_prepare_retried,native_legacy_prepare_max_attempts,
        (unsigned long long)(native_legacy_prepare_wait_us/1000),vsyncHz,(unsigned long long)vsyncEvents[1],(unsigned long long)vsyncEvents[0],
        (unsigned long long)vsyncEvents[2],(unsigned long long)vsyncEvents[3],(unsigned long long)vsyncEvents[4],(unsigned long long)vsyncEvents[5]);
}
void native_legacy_swap_validation_observed(bool accepted,unsigned reason,const void *request,const void *surface) {
    if(accepted)native_legacy_swap_stage(0);
    if(accepted)native_legacy_swap_accepted_count=native_legacy_swap_accepted_count+1;
    else {native_legacy_swap_rejected_count=native_legacy_swap_rejected_count+1;recordRejectedSwap(reason,request,surface);}
    native_legacy_swap_validation_calls=native_legacy_swap_validation_calls+1;
    native_legacy_swap_validation_result=accepted;native_legacy_swap_validation_reason=reason;
    native_legacy_swap_request=(uintptr_t)request;native_legacy_swap_surface=(uintptr_t)surface;
    native_legacy_swap_validation_checkpoint(accepted,reason,request,surface);
}
// Time of the pixel copy into the scanout buffer (counter ticks): swaps completed, last/maximum/sum, for the banner.
volatile uint64_t native_legacy_copy_n,native_legacy_copy_last,native_legacy_copy_max,native_legacy_copy_sum,native_legacy_copy_bytes_last,native_legacy_copy_bytes_sum;
void native_legacy_copy_timing_observed(uint64_t ticks,uint64_t bytes) {
    native_legacy_copy_bytes_last=bytes;native_legacy_copy_bytes_sum=native_legacy_copy_bytes_sum+bytes;
    native_legacy_copy_n=native_legacy_copy_n+1;native_legacy_copy_last=ticks;native_legacy_copy_sum=native_legacy_copy_sum+ticks;
    if(ticks>native_legacy_copy_max)native_legacy_copy_max=ticks;
}
__attribute__((noinline)) void native_legacy_transfer_observed(IOReturn status,const void *request,uint64_t bytes) {
    native_legacy_transfer_calls=native_legacy_transfer_calls+1;
    if(status==kIOReturnSuccess)native_legacy_swap_stage(2);
    native_legacy_transfer_status=status;native_legacy_transfer_request=(uintptr_t)request;native_legacy_transfer_bytes=bytes;
    if(status!=kIOReturnSuccess && boot_progress_stage) {
        char message[120];snprintf(message,sizeof(message),"DISPLAY COPY FAILED status=%08x prepare=%u tries=%u",
          (unsigned)status,native_legacy_prepare_reason,native_legacy_prepare_attempts);
        boot_progress_stage(message);
    }
}
bool native_legacy_gpu_offer(void *display,IOMemoryDescriptor *memory,uint64_t stride,uint32_t width,uint32_t height) {
    return gpuHandoffPresenter && native_gop_gpu_offer(gpuHandoffPresenter,display,memory,stride,width,height);
}
void native_legacy_gpu_fallback_observed(int status) {
    native_legacy_gpu_last_status=(uint32_t)status;native_legacy_gpu_fallbacks=native_legacy_gpu_fallbacks+1;
}
__attribute__((noinline)) void native_legacy_retirement_observed(IOService *display) {
    const unsigned char *bytes=reinterpret_cast<const unsigned char *>(display);
    uint64_t current=0,pending=0;uint32_t completed=0,presented=0,frames=0;
    memcpy(&current,bytes+5472,8);memcpy(&pending,bytes+5480,8);
    memcpy(&completed,bytes+5820,4);memcpy(&presented,bytes+5824,4);memcpy(&frames,bytes+252,4);
    native_legacy_retired_current=current;native_legacy_retired_pending=pending;
    native_legacy_retired_generation=completed;native_legacy_presented_generation=presented;native_legacy_retired_frames=frames;
    native_legacy_retirement_calls=native_legacy_retirement_calls+1;
}
// The display object's properties as they are now (the registration copy above predates setDisplayRefreshProperties and the rest): for the
// UFS log. Returns the bytes written (NUL-terminated).
extern "C" unsigned native_legacy_properties_now(char *out,unsigned cap) {
    if(!out || !cap)return 0;
    out[0]=0;
    IOService *display=reinterpret_cast<IOService *>(native_legacy_object);
    if(!display)return 0;
    OSSerialize *xml=OSSerialize::withCapacity(8192);
    unsigned size=0;
    if(xml && display->serializeProperties(xml)) {
        size=xml->getLength();if(size>=cap)size=cap-1;
        memcpy(out,xml->text(),size);out[size]=0;
    }
    OSSafeReleaseNULL(xml);
    return size;
}
void native_legacy_registration_observed(IOService *display) {
    native_legacy_registration_calls=native_legacy_registration_calls+1;
    OSSerialize *xml=OSSerialize::withCapacity(2048);
    if(xml && display->serializeProperties(xml)) {
        unsigned size=xml->getLength();if(size>=sizeof(native_legacy_properties))size=sizeof(native_legacy_properties)-1;
        memcpy(native_legacy_properties,xml->text(),size);native_legacy_properties[size]=0;native_legacy_properties_length=size;
    }
    OSSafeReleaseNULL(xml);
}
void native_legacy_client_enter(task_t task,uint32_t type) {
    native_legacy_client_calls=native_legacy_client_calls+1;native_legacy_client_task=(uintptr_t)task;
    native_legacy_client_type=type;native_legacy_client_pid=task?proc_pid((proc_t)get_bsdtask_info(task)):0;
}
// Every framebuffer user client seen (object, pid), with what it does: the original client's vtable is copied and two entries (externalMethod, slot 168,
// and registerNotificationPort(port,type,refCon), slot 175) are pointed at wrappers that count and forward to the original function. The original code
// is not touched; only this object's vptr names the copy. XNU's ARM64 vptr points at slot 0 (the original code dispatches with ldr x8,[vptr,#slot*8]).
enum {kSeenClients=8,kSelectors=96,kRegEvents=8,kVtableSlots=256};
struct ClientStat {
    uintptr_t object;int pid;uint32_t calls,fails;uint32_t selector_calls[kSelectors],selector_fails[kSelectors],selector_status[kSelectors];
    unsigned regs;struct {uint32_t type;uint64_t port;uint32_t ref,status;} reg[kRegEvents];
    uint64_t sel6_first[3],sel6_last[3];uint32_t sel6_distinct;uint64_t sel6_seen[8][3];uint32_t sel6_calls[8],sel6_fails[8],sel6_status[8];
    uint32_t seq_count;struct {uint32_t a,b,c,status,done,presented;} seq[24];
    uintptr_t original_external,original_register;
    uint32_t type,open_ds;char name[17];   // asteroids T27
};
static ClientStat seenClients[kSeenClients];static unsigned seenClientCount;
// asteroids T27: every user-client call in order (the first kFirstCalls kept, then a ring of the latest kLastCalls), each with its
// time, caller, selector, argument shape (scalar and structure counts in and out, async = a wake port came with it) and status;
// and every open attempt, successful or not.
enum {kFirstCalls=48,kLastCalls=32,kOpens=12};
struct CallEvent {uint32_t ds;int pid;uint32_t selector,status;uint16_t scalar_in,scalar_out;uint32_t struct_in,struct_out;uint8_t async;uint64_t arg0;
                  uint8_t stack,changed;uint32_t original;};   // asteroids T28: caller stack index (0xff none), state words changed, original status
static CallEvent firstCalls[kFirstCalls],lastCalls[kLastCalls];
static uint32_t callEvents;
struct OpenEvent {uint32_t ds,type,status;int pid;};
static OpenEvent opens[kOpens];static uint32_t openEvents;
extern "C" void proc_name(int pid,char *buf,int size);
static uint32_t probeNowDs(void) {uint64_t ns;absolutetime_to_nanoseconds(mach_absolute_time(),&ns);return (uint32_t)(ns/100000000);}
// asteroids T28: where each call comes from, and what it does to the state words.
// Caller stacks: the calling thread's user return addresses (BootFaultProbe.c), shared-cache addresses unslid, kept once per distinct
// stack. State: the words at display+5808..5868 before and after each call; a change during a call or between two calls is logged.
// Main stacks: every distinct main-thread sample of backboardd the UI trace publishes (boot-ui-focus*=backboardd), with when it was seen,
// so the ~30 s wait before selector 12 shows what it sleeps in.
extern "C" unsigned boot_progress_caller_stack(uint64_t *out,unsigned limit,uint64_t *slide) __attribute__((weak_import));
extern "C" unsigned boot_ui_trace_main_stack(const char *name,uint64_t *out,unsigned limit,int *pid,unsigned *slide) __attribute__((weak_import));
enum {kStackFrames=14,kStacks=16,kStateWords=16,kStateChanges=32,kMainFrames=22,kMainStacks=10};
static const uint64_t kCacheLow=0x180000000ULL,kCacheHigh=0x400000000ULL;
struct CallerStack {uint64_t frame[kStackFrames];unsigned count,uses;int pid;};
static CallerStack callerStacks[kStacks];static unsigned callerStackCount,callerStackBusy;
struct StateChange {uint32_t ds,call,selector,from,to;uint16_t offset;char when;};
static StateChange stateChanges[kStateChanges];static uint32_t stateChangeCount;
static uint32_t stateLast[kStateWords],stateFirst[kStateWords];static bool stateSeen;
struct MainStack {uint64_t frame[kMainFrames];unsigned count;int pid;uint32_t first,last,samples;};
static MainStack mainStacks[kMainStacks];static unsigned mainStackCount,mainStackOverflow,mainSampleCount;
static uint64_t unslid(uint64_t address,uint64_t slide) {
    return address>=kCacheLow+slide && address<kCacheHigh+slide?address-slide:address|(1ULL<<60);   // bit 60: outside the cache, raw
}
static uint8_t callerStackIndex(int pid) {
    if(!&boot_progress_caller_stack || __atomic_exchange_n(&callerStackBusy,1,__ATOMIC_ACQUIRE))return 0xff;
    uint64_t frame[kStackFrames]={},slide=0;
    unsigned n=boot_progress_caller_stack(frame,kStackFrames,&slide);
    for(unsigned i=0;i<n;++i)frame[i]=unslid(frame[i],slide);
    uint8_t index=0xff;
    for(unsigned k=0;k<callerStackCount;++k)
        if(callerStacks[k].pid==pid && callerStacks[k].count==n && !memcmp(callerStacks[k].frame,frame,sizeof frame)){index=(uint8_t)k;break;}
    if(index==0xff && n && callerStackCount<kStacks) {
        CallerStack &c=callerStacks[callerStackCount];memcpy(c.frame,frame,sizeof frame);c.count=n;c.pid=pid;c.uses=0;index=(uint8_t)callerStackCount++;
    }
    if(index!=0xff)++callerStacks[index].uses;
    __atomic_store_n(&callerStackBusy,0,__ATOMIC_RELEASE);
    return index;
}
static void readStateWords(uint32_t *words) {
    const unsigned char *object=reinterpret_cast<const unsigned char *>(native_legacy_object);
    if(!object){memset(words,0,kStateWords*4);return;}
    memcpy(words,object+5808,kStateWords*4);
}
static unsigned noteStateChanges(const uint32_t *from,const uint32_t *to,uint32_t call,uint32_t selector,char when) {
    unsigned changed=0;
    for(unsigned w=0;w<kStateWords;++w) {
        if(from[w]==to[w])continue;
        ++changed;
        uint32_t n=__atomic_fetch_add(&stateChangeCount,1,__ATOMIC_RELAXED);
        if(n<kStateChanges)stateChanges[n]={probeNowDs(),call,selector,from[w],to[w],(uint16_t)(5808+4*w),when};
    }
    return changed;
}
static void sampleMainStack(void) {
    if(!&boot_ui_trace_main_stack)return;
    uint64_t frame[kMainFrames]={};int pid=0;unsigned slide=0;
    unsigned n=boot_ui_trace_main_stack("backboardd",frame,kMainFrames,&pid,&slide);
    if(!n)return;
    ++mainSampleCount;
    for(unsigned i=0;i<n;++i)frame[i]=unslid(frame[i],slide);
    uint32_t now=probeNowDs();
    for(unsigned k=0;k<mainStackCount;++k) {
        MainStack &m=mainStacks[k];
        if(m.pid==pid && m.count==n && !memcmp(m.frame,frame,sizeof frame)){m.last=now;++m.samples;return;}
    }
    if(mainStackCount>=kMainStacks){++mainStackOverflow;return;}
    MainStack &m=mainStacks[mainStackCount++];memcpy(m.frame,frame,sizeof frame);m.count=n;m.pid=pid;m.first=m.last=now;m.samples=1;
}
// asteroids T28: t28-power-ack=1 answers selector 12 (RequestPowerChange) with success when the original refuses a power-on as
// unsupported while the powered word (display+5840) already says on: the firmware-lit panel is on, which is what the caller asks for.
static unsigned powerAckMode,powerAckParsed,powerAckCalls;static uint32_t powerAckOriginal;
static void native_legacy_call_logged(int pid,uint32_t selector,const IOExternalMethodArguments *args,IOReturn status,
                                      uint8_t stack,uint8_t changed,IOReturn original) {
    uint32_t n=__atomic_fetch_add(&callEvents,1,__ATOMIC_RELAXED);
    CallEvent e={probeNowDs(),pid,selector,(uint32_t)status,0,0,0,0,0,0,stack,changed,(uint32_t)original};
    if(args) {
        e.scalar_in=(uint16_t)args->scalarInputCount;e.scalar_out=(uint16_t)args->scalarOutputCount;
        e.struct_in=args->structureInputDescriptor?(uint32_t)args->structureInputDescriptor->getLength():args->structureInputSize;
        e.struct_out=args->structureOutputDescriptor?(uint32_t)args->structureOutputDescriptor->getLength():args->structureOutputSize;
        e.async=args->asyncWakePort!=MACH_PORT_NULL;
        if(args->scalarInput && args->scalarInputCount)e.arg0=args->scalarInput[0];
    }
    if(n<kFirstCalls)firstCalls[n]=e;
    lastCalls[n%kLastCalls]=e;
}
typedef IOReturn (*ExternalFn)(IOUserClient *,uint32_t,IOExternalMethodArguments *,IOExternalMethodDispatch *,OSObject *,void *);
typedef IOReturn (*RegisterFn)(IOUserClient *,mach_port_t,UInt32,UInt32);
static ClientStat *statFor(IOUserClient *client) {
    // The newest entry first: a closed client's address can be reused by the next open (T28: backboardd's calls were credited to
    // IOMFB_bics_daemon's freed client).
    for(unsigned i=seenClientCount;i-->0;)if(seenClients[i].object==(uintptr_t)client)return &seenClients[i];
    return nullptr;
}
static IOReturn wrappedExternal(IOUserClient *client,uint32_t selector,IOExternalMethodArguments *args,IOExternalMethodDispatch *dispatch,OSObject *target,void *ref) {
    ClientStat *stat=statFor(client);
    uint8_t stack=0xff,changed=0;uint32_t before[kStateWords],after[kStateWords];uint32_t call=callEvents;
    if(stat) {
        stack=callerStackIndex(stat->pid);
        readStateWords(before);
        if(!stateSeen){memcpy(stateFirst,before,sizeof before);stateSeen=true;}
        else noteStateChanges(stateLast,before,call,selector,'b');
    }
    IOReturn status=stat?reinterpret_cast<ExternalFn>(stat->original_external)(client,selector,args,dispatch,target,ref):kIOReturnNotAttached;
    const IOReturn original=status;
    if(stat) {
        readStateWords(after);
        changed=(uint8_t)noteStateChanges(before,after,call,selector,'c');
        memcpy(stateLast,after,sizeof after);
        if(!powerAckParsed){PE_parse_boot_argn("t28-power-ack",&powerAckMode,sizeof powerAckMode);powerAckParsed=1;}
        if(selector==12 && powerAckMode==1 && status==kIOReturnUnsupported && args && args->scalarInput && args->scalarInputCount>=1 &&
           args->scalarInput[0]!=0 && after[(5840-5808)/4]==1) {
            status=kIOReturnSuccess;++powerAckCalls;powerAckOriginal=(uint32_t)original;
        }
    }
    if(stat) {
        if(selector==6 && args && args->scalarInput && args->scalarInputCount>=3) {
            const uint64_t *in=args->scalarInput;
            if(!stat->sel6_distinct && !stat->sel6_first[0] && !stat->sel6_first[1])memcpy(stat->sel6_first,in,sizeof stat->sel6_first);
            memcpy(stat->sel6_last,in,sizeof stat->sel6_last);
            int at=-1;
            for(unsigned k=0;k<stat->sel6_distinct && k<8;++k)if(!memcmp(stat->sel6_seen[k],in,24))at=(int)k;
            if(at<0 && stat->sel6_distinct<8){at=(int)stat->sel6_distinct++;memcpy(stat->sel6_seen[at],in,24);}
            if(stat->seq_count<24){
                auto &e=stat->seq[stat->seq_count++];e.a=(uint32_t)in[0];e.b=(uint32_t)in[1];e.c=(uint32_t)in[2];e.status=(uint32_t)status;
                const unsigned char *object=reinterpret_cast<const unsigned char *>(native_legacy_object);
                e.done=e.presented=0;if(object){memcpy(&e.done,object+5820,4);memcpy(&e.presented,object+5824,4);}
            }
            if(at>=0){++stat->sel6_calls[at];if(status)++stat->sel6_fails[at];stat->sel6_status[at]=(uint32_t)status;}
        }
        native_legacy_call_logged(stat->pid,selector,args,status,stack,changed,original);
        ++stat->calls;if(status)++stat->fails;
        if(selector<kSelectors){++stat->selector_calls[selector];if(status)++stat->selector_fails[selector];stat->selector_status[selector]=(uint32_t)status;}
    }
    return status;
}
static IOReturn wrappedRegister(IOUserClient *client,mach_port_t port,UInt32 type,UInt32 refCon) {
    ClientStat *stat=statFor(client);
    IOReturn status=stat?reinterpret_cast<RegisterFn>(stat->original_register)(client,port,type,refCon):kIOReturnNotAttached;
    if(stat && stat->regs<kRegEvents){auto &e=stat->reg[stat->regs++];e.type=type;e.port=(uint64_t)(uintptr_t)port;e.ref=refCon;e.status=(uint32_t)status;}
    return status;
}
static void interpose(IOUserClient *client,int pid) {
    if(seenClientCount>=kSeenClients)return;
    ClientStat &stat=seenClients[seenClientCount];
    memset(&stat,0,sizeof stat);
    uintptr_t *vtable=*reinterpret_cast<uintptr_t **>(client);
    uintptr_t *copy=static_cast<uintptr_t *>(IOMalloc(kVtableSlots*sizeof(uintptr_t)));
    if(!copy)return;
    memcpy(copy,vtable,kVtableSlots*sizeof(uintptr_t));
    stat.object=(uintptr_t)client;stat.pid=pid;stat.original_external=copy[168];stat.original_register=copy[175];
    copy[168]=(uintptr_t)&wrappedExternal;copy[175]=(uintptr_t)&wrappedRegister;
    ++seenClientCount;
    *reinterpret_cast<uintptr_t **>(client)=copy;
}
// The display object's state words the original wait selector tests (offsets 5832..5864, as dwords), for the log.
extern "C" unsigned native_legacy_state_words(char *out,unsigned cap) {
    out[0]=0;
    const unsigned char *object=reinterpret_cast<const unsigned char *>(native_legacy_object);
    if(!object)return 0;
    unsigned at=snprintf(out,cap,"d+5808:");
    for(unsigned o=5808;o<5872 && at+16<cap;o+=4){uint32_t v;memcpy(&v,object+o,4);at+=snprintf(out+at,cap-at," %x",v);}
    return at;
}
// One page row per client (asteroids): pid, calls, fails, registered notification ports, then every selector used as
// selector:calls or selector:calls/fails. Returns the number of clients seen; an index past the end writes an empty row.
extern "C" unsigned native_legacy_client_row(unsigned c,char *out,unsigned cap) {
    out[0]=0;
    if(c>=seenClientCount)return seenClientCount;
    const ClientStat &s=seenClients[c];
    int n=snprintf(out,cap,"C%u/%u p%d c%u f%u r%u sel",c+1,seenClientCount,s.pid,s.calls,s.fails,s.regs);
    unsigned at=n>0 && (unsigned)n<cap?(unsigned)n:cap-1;
    for(unsigned sel=0;sel<kSelectors;++sel) {
        if(!s.selector_calls[sel])continue;
        char item[24];
        n=s.selector_fails[sel]?snprintf(item,sizeof item," %u:%u/%u",sel,s.selector_calls[sel],s.selector_fails[sel])
                               :snprintf(item,sizeof item," %u:%u",sel,s.selector_calls[sel]);
        if(n<0 || at+(unsigned)n+2>=cap){if(at+2<cap){out[at++]='+';out[at]=0;}break;}
        memcpy(out+at,item,(unsigned)n+1);at+=(unsigned)n;
    }
    return seenClientCount;
}
// asteroids T27: the display page (boot-display-page=1), every line of it; BootProgress.c pages through them. IOKit statuses
// (e00002xx) are written as their low three digits, success as "ok".
struct LineWriter {
    char (*rows)[120];unsigned max,count,at;
    void put(const char *format,...) __attribute__((format(printf,2,3))) {
        if(count>=max)return;
        char item[120];va_list ap;va_start(ap,format);int n=vsnprintf(item,sizeof item,format,ap);va_end(ap);
        if(n<0)return;if(n>119)n=119;
        if(at+(unsigned)n>119){if(++count>=max)return;at=0;rows[count][0]=0;}
        memcpy(rows[count]+at,item,(unsigned)n+1);at+=(unsigned)n;
    }
    void end(void) {if(count<max && at){++count;}at=0;if(count<max)rows[count][0]=0;}
};
static const char *findText(const char *from,const char *needle) {
    size_t n=strlen(needle);
    for(;*from;++from)if(!strncmp(from,needle,n))return from;
    return nullptr;
}
static const char *statusText(uint32_t status,char *out,unsigned cap) {
    if(!status)snprintf(out,cap,"ok");
    else if((status&0xfffff000)==0xe0000000)snprintf(out,cap,"%x",status&0xfff);
    else snprintf(out,cap,"%x",status);
    return out;
}
static void callItem(LineWriter &w,const CallEvent &e) {
    char st[12],orig[16]="",tail[16]="";
    if(e.original!=e.status){char o[12];snprintf(orig,sizeof orig,"(%s)",statusText(e.original,o,sizeof o));}
    if(e.stack!=0xff)snprintf(tail,sizeof tail," k%u",e.stack);
    if(e.changed){size_t l=strlen(tail);snprintf(tail+l,sizeof tail-l," w%u",e.changed);}
    w.put("%u.%u %d#%u %u/%u>%u/%u%s %llx=%s%s%s| ",e.ds/10,e.ds%10,e.pid,e.selector,e.scalar_in,e.struct_in,e.scalar_out,e.struct_out,
          e.async?" A":"",(unsigned long long)e.arg0,statusText(e.status,st,sizeof st),orig,tail);
}
static void frameItems(LineWriter &w,const uint64_t *frame,unsigned count) {
    for(unsigned i=0;i<count;++i) {
        if(frame[i]>>60)w.put(" !%llx",(unsigned long long)(frame[i]&~(1ULL<<60)));
        else w.put(" %llx",(unsigned long long)frame[i]);
    }
}
extern "C" unsigned native_legacy_display_lines(char (*rows)[120],unsigned max) {
    LineWriter w={rows,max,0,0};if(max)rows[0][0]=0;
    char st[12];
    sampleMainStack();
    w.put("IOMFB obj=%llx reg=%u props=%uB opens=%u clients=%u calls=%u; call: s.d pid#sel in scalars/bytes>out scalars/bytes A=async arg0=status",
          (unsigned long long)native_legacy_object,native_legacy_registration_calls,native_legacy_properties_length,openEvents,seenClientCount,callEvents);w.end();
    const unsigned char *object=reinterpret_cast<const unsigned char *>(native_legacy_object);
    if(object) {
        w.put("STATE d+5808:");
        for(unsigned o=5808;o<5872;o+=4){uint32_t v;memcpy(&v,object+o,4);w.put(" %x",v);}
        w.end();
    }
    for(unsigned i=0;i<openEvents && i<kOpens;++i)w.put("OPEN %u.%u p%d type%u %s | ",opens[i].ds/10,opens[i].ds%10,opens[i].pid,opens[i].type,statusText(opens[i].status,st,sizeof st));
    w.end();
    for(unsigned c=0;c<seenClientCount;++c) {
        const ClientStat &s=seenClients[c];
        w.put("C%u p%d %s type%u at %u.%u calls=%u fails=%u ports=%u:",c+1,s.pid,s.name,s.type,s.open_ds/10,s.open_ds%10,s.calls,s.fails,s.regs);
        for(unsigned sel=0;sel<kSelectors;++sel) {
            if(!s.selector_calls[sel])continue;
            if(s.selector_fails[sel])w.put(" %u:%u/%u@%s",sel,s.selector_calls[sel],s.selector_fails[sel],statusText(s.selector_status[sel],st,sizeof st));
            else w.put(" %u:%u",sel,s.selector_calls[sel]);
        }
        w.end();
        for(unsigned r=0;r<s.regs && r<kRegEvents;++r)
            w.put("  port type%u %llx ref%x %s |",s.reg[r].type,(unsigned long long)s.reg[r].port,s.reg[r].ref,statusText(s.reg[r].status,st,sizeof st));
        if(s.regs)w.end();
        for(unsigned k=0;k<s.seq_count && k<6;++k)
            w.put("  s6 %x %x %x=%s done%x pres%x |",s.seq[k].a,s.seq[k].b,s.seq[k].c,statusText(s.seq[k].status,st,sizeof st),s.seq[k].done,s.seq[k].presented);
        if(s.seq_count)w.end();
    }
    uint32_t n=callEvents;
    w.put("FIRST %u calls:",n<kFirstCalls?n:(uint32_t)kFirstCalls);w.end();
    for(unsigned i=0;i<n && i<kFirstCalls;++i)callItem(w,firstCalls[i]);
    w.end();
    if(n>kFirstCalls) {
        unsigned shown=n-kFirstCalls<kLastCalls?n-kFirstCalls:(unsigned)kLastCalls;
        w.put("LAST %u of %u calls:",shown,n);w.end();
        for(unsigned i=n-shown;i<n;++i)callItem(w,lastCalls[i%kLastCalls]);
        w.end();
    }
    // asteroids T28: caller stacks, state changes, selector 12 answers, backboardd's main-thread stacks.
    w.put("STACKS %u (k<n> on a call; cache addresses unslid, !raw = outside the cache; w<n> = state words the call changed):",callerStackCount);w.end();
    for(unsigned k=0;k<callerStackCount;++k){w.put("k%u p%d x%u:",k,callerStacks[k].pid,callerStacks[k].uses);frameItems(w,callerStacks[k].frame,callerStacks[k].count);w.end();}
    if(stateSeen){w.put("STATE at first call d+5808:");for(unsigned i=0;i<kStateWords;++i)w.put(" %x",stateFirst[i]);w.end();}
    w.put("STATE CHANGES %u (c = during call, b = between calls; s.d call#sel +offset old>new):",stateChangeCount);w.end();
    for(unsigned i=0;i<stateChangeCount && i<kStateChanges;++i){const StateChange &c=stateChanges[i];
        w.put("%u.%u %c%u#%u +%u %x>%x| ",c.ds/10,c.ds%10,c.when,c.call,c.selector,c.offset,c.from,c.to);}
    w.end();
    w.put("POWER ACK mode=%u answered=%u original=%s",powerAckMode,powerAckCalls,statusText(powerAckOriginal,st,sizeof st));w.end();
    w.put("MAIN backboardd samples=%u distinct=%u overflow=%u (pc lr frames; s.d first-last xN):",mainSampleCount,mainStackCount,mainStackOverflow);w.end();
    for(unsigned k=0;k<mainStackCount;++k){const MainStack &m=mainStacks[k];
        w.put("m%u p%d %u.%u-%u.%u x%u:",k,m.pid,m.first/10,m.first%10,m.last/10,m.last%10,m.samples);frameItems(w,m.frame,m.count);w.end();}
    // The display's own properties as registered: key names only.
    w.put("KEYS:");
    for(const char *p=native_legacy_properties;(p=findText(p,"<key>"));) {
        p+=5;const char *e=findText(p,"</key>");if(!e)break;
        w.put(" %.*s",(int)(e-p<60?e-p:60),p);p=e;
    }
    w.end();
    return w.count;
}
extern "C" unsigned native_legacy_clients_dump(char *out,unsigned cap) {
    unsigned at=0;out[0]=0;
    for(unsigned c=0;c<seenClientCount && at+120<cap;++c) {
        const ClientStat &s=seenClients[c];
        at+=snprintf(out+at,cap-at,"C%u pid=%d calls=%u fails=%u regs=%u",c,s.pid,s.calls,s.fails,s.regs);
        for(unsigned r=0;r<s.regs && r<kRegEvents && at+60<cap;++r)
            at+=snprintf(out+at,cap-at," reg(t%u,p%llx,r%x)=%x",s.reg[r].type,(unsigned long long)s.reg[r].port,s.reg[r].ref,s.reg[r].status);
        at+=snprintf(out+at,cap-at,"\n");
        for(unsigned k=0;k<s.sel6_distinct && k<8 && at+100<cap;++k)
            at+=snprintf(out+at,cap-at,"  sel6 args %llx %llx %llx: %u calls %u fails last %x\n",(unsigned long long)s.sel6_seen[k][0],(unsigned long long)s.sel6_seen[k][1],(unsigned long long)s.sel6_seen[k][2],s.sel6_calls[k],s.sel6_fails[k],s.sel6_status[k]);
        for(unsigned k=0;k<s.seq_count && at+90<cap;++k)
            at+=snprintf(out+at,cap-at,"  s6#%u %x %x %x -> %x [5820]=%x [5824]=%x\n",k,s.seq[k].a,s.seq[k].b,s.seq[k].c,s.seq[k].status,s.seq[k].done,s.seq[k].presented);
        unsigned n=0;
        for(unsigned sel=0;sel<kSelectors && at+40<cap;++sel) {
            if(!s.selector_calls[sel])continue;
            if(!n)at+=snprintf(out+at,cap-at,"  sel");
            at+=snprintf(out+at,cap-at," %u:%u/%u/%x",sel,s.selector_calls[sel],s.selector_fails[sel],s.selector_status[sel]);
            if(++n%6==0)at+=snprintf(out+at,cap-at,"\n  sel");
        }
        if(n)at+=snprintf(out+at,cap-at,"\n");
    }
    return at;
}
void native_legacy_client_observed(IOReturn status,IOUserClient *client) {
    uint32_t o=__atomic_fetch_add(&openEvents,1,__ATOMIC_RELAXED);
    if(o<kOpens)opens[o]={probeNowDs(),native_legacy_client_type,(uint32_t)status,native_legacy_client_pid};
    unsigned before=seenClientCount;
    if(client && !status)interpose(client,native_legacy_client_pid);
    if(seenClientCount>before) {
        ClientStat &c=seenClients[before];c.type=native_legacy_client_type;c.open_ds=probeNowDs();
        proc_name(c.pid,c.name,sizeof c.name);
    }
    native_legacy_client_status=status;native_legacy_client_object=(uintptr_t)client;
    if(client)strlcpy(native_legacy_client_class,client->getMetaClass()->getClassName(),sizeof(native_legacy_client_class));
    native_legacy_client_checkpoint();
}
__attribute__((noinline)) bool native_legacy_gop_mapping_observed(IOService *object,IOMemoryMap *mapping,uint32_t width,uint32_t height) {
    if((uintptr_t)object!=native_legacy_object)panic("NativeLegacyDisplay: wrong mapping owner");
    native_legacy_gop_map=(uintptr_t)mapping;native_legacy_gop_virtual=mapping->getVirtualAddress();
    native_legacy_gop_physical=mapping->getPhysicalAddress();native_legacy_gop_bytes=mapping->getLength();
    native_legacy_gop_width=width;native_legacy_gop_height=height;
    if(boot_progress_pulse_anchor && !diagnosticAnchorMapping) {
        PE_Video anchorVideo={};
        if(!PE_current_console(&anchorVideo) && anchorVideo.v_depth==32 &&
           anchorVideo.v_rowBytes<=65536 &&
           anchorVideo.v_width==width && anchorVideo.v_height==height &&
           anchorVideo.v_baseAddr==mapping->getPhysicalAddress() && !anchorVideo.v_rotate) {
            mapping->retain();
            if(boot_progress_pulse_anchor(mapping->getVirtualAddress(),mapping->getLength(),width,height,
                (unsigned)anchorVideo.v_rowBytes))diagnosticAnchorMapping=mapping;
            else mapping->release();
        }
    }
    native_legacy_gop_mapping_checkpoint();
    unsigned enabled=0;PE_parse_boot_argn("offline-legacy-unit-scale",&enabled,sizeof(enabled));
    if(!enabled)return true;
    native_legacy_unit_scale_calls=native_legacy_unit_scale_calls+1;native_legacy_unit_scale_status=1;
    unsigned phase=0;PE_parse_boot_argn("offline-legacy-phase",&phase,sizeof(phase));
    PE_Video video={};
    // The value is the UI scale: logical points = actual scanout pixels / scale (1..3). It changes
    // only the published display-scale; the scanout copy still moves one actual pixel per pixel.
    if(enabled<1 || enabled>3 || phase!=4 || unitScaleChosen || PE_current_console(&video) ||
       video.v_width!=width || video.v_height!=height || video.v_rotate ||
       mapping->getPhysicalAddress()!=video.v_baseAddr)return false;
    // Explicit offline UI policy: `enabled` actual GOP scanout pixels per logical
    // point in each direction (1 = native pixels). This is not physical pixel density,
    // panel timing, an Apple device identity or a synthesized mode list.
    unitScaleChosen=IORegistryEntry::fromPath("/chosen",gIODTPlane);
    if(!unitScaleChosen || !publishOrMatchWord(unitScaleChosen,"display-scale",enabled,unitScaleAdded) ||
       !publishOrMatchWord(unitScaleChosen,"display-rotation",video.v_rotate,unitRotationAdded)) {
        finishUnitScalePolicy(false);return false;
    }
    OSData *scale=OSDynamicCast(OSData,unitScaleChosen->getProperty("display-scale"));
    OSData *rotation=OSDynamicCast(OSData,unitScaleChosen->getProperty("display-rotation"));
    uint32_t actualScale=0,actualRotation=0;
    if(!scale || !rotation || scale->getLength()!=4 || rotation->getLength()!=4) {
        finishUnitScalePolicy(false);return false;
    }
    memcpy(&actualScale,scale->getBytesNoCopy(),4);memcpy(&actualRotation,rotation->getBytesNoCopy(),4);
    native_legacy_chosen_scale=actualScale;native_legacy_chosen_rotation=actualRotation;
    native_legacy_unit_scale_status=actualScale==enabled && actualRotation==video.v_rotate?0:2;
    if(native_legacy_unit_scale_status){finishUnitScalePolicy(false);return false;}
    return true;
}
__attribute__((noreturn,noinline)) void native_legacy_backend_guard(unsigned slot,void *object,void *caller) {
    native_legacy_guard_slot=slot;native_legacy_guard_object=(uintptr_t)object;native_legacy_guard_caller=(uintptr_t)caller;
    panic("NativeLegacyDisplay: leaf=%u caller=%llx stage=%u PM=%u swap=%u/%u copy=%08x bytes=%llu retire=%u handoff=%u/%u",
      slot,(uint64_t)(uintptr_t)caller,native_legacy_stage,native_legacy_pm_stage,
      native_legacy_swap_validation_result,native_legacy_swap_validation_reason,
      (unsigned)native_legacy_transfer_status,native_legacy_transfer_bytes,native_legacy_retirement_calls,
      native_legacy_prepare_reason,native_legacy_prepare_attempts);
}
void native_legacy_display_probe(void);
void native_legacy_display_probe(void) {
    // The registration path and optional original-userclient inspection both
    // reach this boot-thread entry. Run class constructors only once; a second
    // postModLoad would attempt to register the same metaclasses again.
    if(native_legacy_stage)return;
    native_legacy_stage=1;void *handle=OSMetaClass::preModLoad(g_kernel_kmod_info.name);
    if(!handle){native_legacy_display_checkpoint();return;}
    native_legacy_display_static_init();native_gop_presenter_static_init();native_legacy_meta_status=OSMetaClass::postModLoad(handle);
    if(native_legacy_meta_status){native_legacy_display_checkpoint();return;}
    PE_Video video={};native_legacy_stage=2;
    if(PE_current_console(&video) || !video.v_baseAddr || !video.v_width || !video.v_height || video.v_depth!=32
        || video.v_rotate || video.v_width>16384 || video.v_height>16384 || video.v_rowBytes<video.v_width*4
        || video.v_rowBytes>UINT64_MAX/video.v_height){native_legacy_display_checkpoint();return;}
    IOService *display=native_legacy_display_allocate();native_legacy_object=(uintptr_t)display;
    if(!display){native_legacy_display_checkpoint();return;}
    native_legacy_object_size=display->getMetaClass()->getClassSize();
    native_legacy_parent_match=(display->metaCast("IOMobileFramebufferLegacy") && display->metaCast("IOMobileFramebuffer") && display->metaCast("IOService"));
    native_legacy_stage=3;native_legacy_init_result=display->init();
    unsigned phase=1;PE_parse_boot_argn("offline-legacy-phase",&phase,sizeof(phase));
    if(native_legacy_init_result && (phase==2 || phase==3 || phase==4)) {
        // IOSurface matches the real IOBSD resource, published by bsd_autoconf.
        // Retain this actual object until that normal lifecycle boundary.
        pendingLegacyDisplay=display;return;
    }
    native_legacy_stage=7;display->release();native_legacy_released=1;native_legacy_stage=8;
    native_legacy_display_checkpoint();
}
void native_legacy_display_late_probe(void);
extern void native_hid_input_probe(void);
extern void native_keystore_probe(void);
extern void sheng_gpu_power_probe(void);
extern "C" void asteroids_touch_probe(void);
void native_legacy_display_late_probe(void) {
    native_keystore_probe();
    sheng_gpu_power_probe();   // asteroids T30: GPU power (package/GpuPowerProbe.cpp); nothing without sheng-gpu-power
    asteroids_touch_probe();   // asteroids T30: touch stage 1 (package/AsteroidsTouchProbe.cpp); nothing without asteroids-touch
    native_hid_input_probe();
    IOService *display=pendingLegacyDisplay;if(!display)return;pendingLegacyDisplay=nullptr;
    native_legacy_surface_status=OSKextLoadKextWithIdentifier("com.apple.iokit.IOSurface");
    OSDictionary *matching=IOService::nameMatching("IOCoreSurfaceRoot");
    IOService *surface=matching?IOService::waitForMatchingService(matching,5000000000ULL):nullptr;
    native_legacy_surface_found=surface!=nullptr;OSSafeReleaseNULL(surface);OSSafeReleaseNULL(matching);
    PE_Video video={};
    if(!native_legacy_surface_status && native_legacy_surface_found && !PE_current_console(&video)
        && video.v_baseAddr && video.v_width && video.v_height && video.v_depth==32
        && !video.v_rotate && video.v_width<=16384 && video.v_height<=16384
        && video.v_rowBytes>=video.v_width*4 && video.v_rowBytes<=UINT64_MAX/video.v_height) {
        native_legacy_video_base=video.v_baseAddr;native_legacy_video_bytes=video.v_rowBytes*video.v_height;
        // A real firmware framebuffer resource, with addresses and extent
        // obtained from the actual GOP handoff. It has no invented compatible,
        // display list, panel timing, GPU/fence completion or vendor registers.
        native_legacy_stage=4;IOPlatformDevice *nub=new IOPlatformDevice;
        native_legacy_nub=(uintptr_t)nub;bool nubAttached=false,nubStarted=false,attached=false,started=false;
        IORegistryEntry *displayTreeParent=nullptr;bool dtAttached=false;
        IOService *parent=IOService::getResourceService();
        IODeviceMemory *memory=IODeviceMemory::withRange(video.v_baseAddr,video.v_rowBytes*video.v_height);
        OSArray *ranges=OSArray::withCapacity(1);
        if(nub && memory && ranges && parent && nub->init()) {
            ranges->setObject(memory);nub->setDeviceMemory(ranges);nub->setName("NativeGOPFramebuffer");
            nubAttached=nub->attach(parent);if(nubAttached)nubStarted=nub->start(parent);
            if(nubStarted) {
                unsigned phase=0;PE_parse_boot_argn("offline-legacy-phase",&phase,sizeof(phase));
                if(phase==4) {
                    // This is the actually implemented GOP display subsystem,
                    // attached to its real IOPlatformDevice memory resource.
                    // No Apple/vendor compatible or panel timing is supplied.
                    displayTreeParent=IORegistryEntry::fromPath("/arm-io",gIODTPlane);
                    const char type[]="display-subsystem";OSData *data=OSData::withBytes(type,sizeof(type));
                    if(displayTreeParent && data && nub->setProperty("device_type",data)) {
                        nub->setName("gop-display",gIODTPlane);dtAttached=nub->attachToParent(displayTreeParent,gIODTPlane);
                    }
                    OSSafeReleaseNULL(data);native_legacy_dt_attached=dtAttached;
                }
                if(phase==3) {
                    IOService *presenter=native_gop_presenter_create();bool presenterAttached=false;
                    if(presenter && presenter->init() && (presenterAttached=presenter->attach(nub)) && presenter->start(nub)) {
                        // Explicit own transport service only. The incomplete
                        // original framebuffer backend is never published.
                        activeGOPPresenter=presenter;activeGOPNub=nub;nub=nullptr;nubAttached=false;nubStarted=false;
                    } else {if(presenterAttached)presenter->detach(nub);OSSafeReleaseNULL(presenter);}
                } else {
                    SInt32 score=0;native_legacy_probe_match=(phase!=4 || dtAttached) && display->probe(nub,&score)==display;
                    if(native_legacy_probe_match)attached=display->attach(nub);
                    if(attached){native_legacy_stage=5;started=display->start(nub);native_legacy_start_result=started;}
                    if(started && phase==4) {
                        activeLegacyDisplay=display;activeGOPNub=nub;native_legacy_stage=10;
                        {
                            unsigned gpu=0;PE_parse_boot_argn("sheng-fb-gpu",&gpu,sizeof gpu);
                            if(gpu==1) {
                                IOService *handoff=native_gop_presenter_create();
                                // Not a child of the display subsystem's node: the original display code enumerates that node's children (int-075/077:
                                // SpringBoard found no display and crashed with the presenter attached there, registered or not).
                                IOService *home=IOService::getResourceService();
                                bool handoffAttached=false;
                                if(handoff && home)native_gop_presenter_set_source(handoff,nub);
                                if(handoff && home && handoff->init() && (handoffAttached=handoff->attach(home)) && handoff->start(home))gpuHandoffPresenter=handoff;
                                else {if(handoffAttached)handoff->detach(home);OSSafeReleaseNULL(handoff);}
                            }
                        }
                        finishUnitScalePolicy(true);
                        OSSafeReleaseNULL(displayTreeParent);
                        native_legacy_display_checkpoint();return;
                    }
                }
            }
        }
        native_legacy_stage=6;finishUnitScalePolicy(false);
        if(started)display->stop(nub);if(attached)display->detach(nub);
        if(dtAttached)nub->detachFromParent(displayTreeParent,gIODTPlane);
        if(nubStarted)nub->stop(parent);if(nubAttached)nub->detach(parent);
        OSSafeReleaseNULL(ranges);OSSafeReleaseNULL(memory);OSSafeReleaseNULL(nub);
        OSSafeReleaseNULL(displayTreeParent);
    }
    native_legacy_stage=7;display->release();native_legacy_released=1;native_legacy_stage=8;
    native_legacy_display_checkpoint();
}
}
