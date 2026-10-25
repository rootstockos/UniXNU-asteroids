// SPDX-License-Identifier: MIT
// From Allenwdk/UniXNU-uke package/BootWatchdogPolicy.c. asteroids also runs under Gunyah, so the watchdog pet thread
// applies here as well. asteroids additions (only with sheng-gpu-addon=1): sheng-lite keeps the bluetoothd job for the GPU addon's
// daemon (T16b), an exec log for the GPU probe's diagnostics (T16b), and bluetoothd's exec turned into the daemon (T16c,
// with sheng-gpu-addon-swap=1).
// Explicit temporary bring-up policy for an unavailable watchdog backend.
// Runs after writable scratch Data mounts, before the unchanged launchd exec.
#include <sys/param.h>
#include <sys/systm.h>
#include <sys/proc.h>
#include <sys/sysproto.h>
#include <sys/fcntl.h>
#include <sys/stat.h>
#include <pexpert/pexpert.h>
#include <kern/thread.h>
#include <kern/clock.h>
#include <kern/sched_prim.h>
#include <sys/kauth.h>

static const char policy_head[] =
 "<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n"
 "<!DOCTYPE plist PUBLIC \"-//Apple//DTD PLIST 1.0//EN\" \"http://www.apple.com/DTDs/PropertyList-1.0.dtd\">\n"
 "<plist version=\"1.0\"><dict>";
static const char policy_watchdogd[]="<key>com.apple.watchdogd</key><true/><key>com.apple.watchdog</key><true/>";
// Bring-up isolation: hardware stacks show identityservicesd, contextstored and locationd in a
// cycle of synchronous XPC waits. A disabled job makes its Mach service lookups fail instead of hang.
// sheng-disable-jobs=<label>[,<label>...] (restricted label characters, bounded length).
static const char policy_tail[]="</dict></plist>\n";
// sheng-lite=1: jobs that only reach for the network, iCloud or the photo library and have nothing to do on this tablet (no radios, no accounts),
// all of which start at the first boot of a fresh Data and compete with SpringBoard for the CPU and for memory. The first four are the
// cycle of synchronous XPC waits the isolation work found. Labels are the ones in the target's own LaunchDaemons.
static const char lite_jobs[]=
 "com.apple.identityservicesd,com.apple.locationd,com.apple.CommCenter,com.apple.bluetoothd,"
 "com.apple.photoanalysisd,com.apple.mediaanalysisd,com.apple.mediastream.mstreamd,com.apple.icloud.searchpartyd,"
 "com.apple.findmy.findmybeaconingd,com.apple.familycircled,com.apple.financed,com.apple.softwareupdateservicesd,"
 "com.apple.ndoagent,com.apple.biomesyncd,com.apple.email.maild,com.apple.amsengagementd,com.apple.adid,com.apple.eyereliefd,"
 // Crash or respawn loops seen on hardware (int-176): fairplayd.A2 is KeepAlive and exits at once (22 launches in 6 minutes), passd crashes,
 // obliteration is a data-wipe daemon with nothing to do here. iconservicesagent also dies in CI::GLContext::init, but disabling it leaves
 // the icons missing (seen on hardware), so it stays enabled.
 "com.apple.fairplayd.A2,com.apple.passd,com.apple.mobile.obliteration";
static char policy[2048],jobs[200];
volatile int sheng_watchdog_policy_status=-1;
// asteroids T16b: with the GPU addon (sheng-gpu-addon=1) its test daemon takes the bluetoothd job name (UniXNU's
// build-adreno-metal-plugin.py: that job is RunAtLoad and was seen to run on every boot), so sheng-lite leaves that one label enabled.
static const char lite_kept_for_addon[]="com.apple.bluetoothd";

// asteroids T16b: an exec log for the boot diagnostics (the GPU probe prints it). A KAUTH_SCOPE_FILEOP listener sees every exec with
// its path: a count of all execs, the last eight names, and counts for the names the GPU addon's daemon may run under. Plain counters
// written without a lock: a lost update only blurs a diagnostic. Registered before launchd starts; never removed.
volatile uint32_t asteroids_exec_total,asteroids_exec_ngpu,asteroids_exec_bluetoothd,asteroids_exec_fdr,asteroids_exec_seq;
char asteroids_exec_last[8][24];
char asteroids_exec_ngpu_path[64];
volatile int32_t asteroids_exec_ngpu_uid=-1;   // T16c: the user the daemon runs as (bluetoothd's job decides)
static int asteroids_exec_listener(kauth_cred_t credential,void *data,kauth_action_t action,uintptr_t arg0,uintptr_t arg1,uintptr_t arg2,uintptr_t arg3) {
 (void)data;(void)arg0;(void)arg2;(void)arg3;
 const char *path=(const char *)arg1;
 if(action!=KAUTH_FILEOP_EXEC || !path)return KAUTH_RESULT_DEFER;
 const char *name=strrchr(path,'/');name=name?name+1:path;
 const uint32_t n=asteroids_exec_total;asteroids_exec_total=n+1;
 strlcpy(asteroids_exec_last[n&7],name,sizeof(asteroids_exec_last[0]));
 if(!strncmp(path,"/private/var/ngpu/",18)){asteroids_exec_ngpu=asteroids_exec_ngpu+1;strlcpy(asteroids_exec_ngpu_path,path,sizeof(asteroids_exec_ngpu_path));asteroids_exec_ngpu_uid=credential?(int32_t)kauth_cred_getuid(credential):-2;}
 if(!strcmp(name,"bluetoothd"))asteroids_exec_bluetoothd=asteroids_exec_bluetoothd+1;
 for(const char *c=name;*c;++c)if(!strncmp(c,"fdr_loader",10)){asteroids_exec_fdr=asteroids_exec_fdr+1;break;}
 asteroids_exec_seq=asteroids_exec_seq+1;
 return KAUTH_RESULT_DEFER;
}
static void asteroids_exec_log_start(void) {
 static kauth_listener_t listener;
 if(!listener)listener=kauth_listen_scope(KAUTH_SCOPE_FILEOP,asteroids_exec_listener,NULL);
}

// asteroids T16c: launchd starts its jobs from its own compiled list, so the addon's job plists never ran (T16b: the real bluetoothd ran,
// ngpu 0). Instead the kernel's exec hook (UniXNU zzy-native-gpu-addon-mount.patch) runs the GPU test daemon in place of bluetoothd, which
// this board does not use: launchd's own com.apple.bluetoothd job, only the program changes. Same idea as pandora's sweetexecswap.
extern const char *(*native_gpu_addon_exec_substitute)(const char *path);
volatile uint32_t asteroids_exec_swaps;
static const char *asteroids_exec_substitute(const char *path) {
 if(!path || strcmp(path,"/usr/sbin/bluetoothd"))return NULL;
 const uint32_t n=asteroids_exec_swaps;asteroids_exec_swaps=n+1;
 if(n<4)printf("ASTEROIDS GPU: exec swap %u: /usr/sbin/bluetoothd -> /private/var/ngpu/bin/asteroidsgpud\n",n+1);
 return "/private/var/ngpu/bin/asteroidsgpud";
}

// sheng-setup=1 or sheng-data-reset=2 (the install entry's "show Hello" choice): remove Setup's completion flags once, so this boot shows the Hello page.
// iPadOS writes them again when Setup finishes, so later boots do not show it. Absent files are fine.
static int sheng_boot_setup_reset(proc_t p,uint64_t scratch) {
 static const char *const files[]={
  "/private/var/mobile/Library/Preferences/com.apple.purplebuddy.plist",
  "/private/var/mobile/Library/Preferences/com.apple.purplebuddy.notbackedup.plist"};
 for(unsigned i=0;i<sizeof(files)/sizeof(files[0]);++i) {
  int32_t result=0;
  int error=copyout(files[i],scratch,strlen(files[i])+1);
  if(error)return error;
  struct unlink_args rm={.path=scratch};
  error=unlink(p,&rm,&result);
  if(error && error!=ENOENT)return error;
 }
 return 0;
}

// One comma-separated label at *c into the policy as a disabled key; advances *c past it.
static int append_job(const char **cursor,const char *skip) {
 const char *c=*cursor,*e=c;while(*e && *e!=',')++e;
 if(e>c && !(skip && strlen(skip)==(size_t)(e-c) && !memcmp(skip,c,(size_t)(e-c)))) {
  if(strlcat(policy,"<key>",sizeof(policy))>=sizeof(policy))return EINVAL;
  size_t at=strlen(policy);if(at+(size_t)(e-c)+1>=sizeof(policy))return EINVAL;
  memcpy(policy+at,c,(size_t)(e-c));policy[at+(size_t)(e-c)]=0;
  if(strlcat(policy,"</key><true/>",sizeof(policy))>=sizeof(policy))return EINVAL;
 }
 *cursor=*e?e+1:e;return 0;
}

// The boot-log archive compiles against the exported headers only; the
// scheduler entry point and priority below are private osfmk declarations.
extern kern_return_t kernel_thread_start_priority(thread_continue_t continuation,
                                                  void *parameter, int priority,
                                                  thread_t *new_thread);
#define BASEPRI_PREEMPT 92

// The device repository's one SMC: x0-x7 in r[], and back in r[] after the call. The SMC Calling Convention lets the
// callee change x4-x17. The Gunyah watchdog pet below and the GPU probe's secure monitor calls (mode 15) share it.
void sheng_smc_call(uint64_t r[8]);
void sheng_smc_call(uint64_t r[8]) {
 register uint64_t x0 __asm("x0")=r[0];
 register uint64_t x1 __asm("x1")=r[1];
 register uint64_t x2 __asm("x2")=r[2];
 register uint64_t x3 __asm("x3")=r[3];
 register uint64_t x4 __asm("x4")=r[4];
 register uint64_t x5 __asm("x5")=r[5];
 register uint64_t x6 __asm("x6")=r[6];
 register uint64_t x7 __asm("x7")=r[7];
 __asm__ volatile("smc #0" : "+r"(x0),"+r"(x1),"+r"(x2),"+r"(x3),"+r"(x4),"+r"(x5),"+r"(x6),"+r"(x7) : :
                  "x8","x9","x10","x11","x12","x13","x14","x15","x16","x17","memory");
 r[0]=x0;r[1]=x1;r[2]=x2;r[3]=x3;r[4]=x4;r[5]=x5;r[6]=x6;r[7]=x7;
}

// The device runs under Gunyah and the hypervisor keeps a virtual watchdog for
// this VM. Android's gh_virt_wdt pets it from a timer; the UEFI pet only covers
// the firmware handoff, and the CPU display renderer can take minutes, so keep
// petting it from a kernel thread even when boot-progress is compiled out.
static void uke_gunyah_wdt_pet_thread(void *arg, wait_result_t result) {
 (void)arg;(void)result;
 for (;;) {
  uint64_t r[8]={0x86000007,0,0,0,0,0,0,0};
  sheng_smc_call(r);
  assert_wait_timeout((event_t)&uke_gunyah_wdt_pet_thread,THREAD_UNINT,5000,NSEC_PER_MSEC);
  thread_block(THREAD_CONTINUE_NULL);
 }
}
static int uke_gunyah_wdt_started;
static void uke_gunyah_wdt_start(void) {
 thread_t thread;
 if(uke_gunyah_wdt_started)return;
 uke_gunyah_wdt_started=1;
 if(kernel_thread_start_priority(uke_gunyah_wdt_pet_thread,0,BASEPRI_PREEMPT,&thread)==KERN_SUCCESS)thread_deallocate(thread);
 else printf("sheng bring-up Gunyah watchdog pet thread failed\n");
}

int sheng_boot_watchdog_policy(void *process, uint64_t scratch) {
 proc_t p=process;
 int enabled=0,error=0,fd=-1;int32_t result=0;
 uke_gunyah_wdt_start();
 int addon=0;PE_parse_boot_argn("sheng-gpu-addon",&addon,sizeof(addon));
 // The swap only when the addon carries the daemon (sheng-gpu-addon-swap=1, T16c); T17's addon brings the Metal plugin instead.
 int swap=0;PE_parse_boot_argn("sheng-gpu-addon-swap",&swap,sizeof(swap));
 if(addon==1){asteroids_exec_log_start();if(swap==1)native_gpu_addon_exec_substitute=asteroids_exec_substitute;}
 PE_parse_boot_argn("sheng-disable-watchdogd",&enabled,sizeof(enabled));
 if(!PE_parse_boot_argn("sheng-disable-jobs",jobs,sizeof(jobs)))jobs[0]=0;jobs[sizeof(jobs)-1]=0;
 int lite=0;PE_parse_boot_argn("sheng-lite",&lite,sizeof(lite));if(lite!=1)lite=0;
 if(enabled!=1)enabled=0;
 for(const char *c=jobs;*c;++c)if(!((*c>='a'&&*c<='z')||(*c>='A'&&*c<='Z')||(*c>='0'&&*c<='9')||*c=='.'||*c=='-'||*c==','))return EINVAL;
 int setup=0,reset=0;
 PE_parse_boot_argn("sheng-setup",&setup,sizeof(setup));
 PE_parse_boot_argn("sheng-data-reset",&reset,sizeof(reset));   // 2 = erase Data and show the Hello page; 1 = erase Data, Setup stays skipped
 if(reset==2)setup=1;
 if(setup==1 && (error=sheng_boot_setup_reset(p,scratch))) {
  sheng_watchdog_policy_status=error;
  printf("sheng setup flags reset failed: %d\n",error);
  return error;
 }
 if(setup==1)printf("sheng setup flags reset: Hello page on this boot\n");
 if(!enabled && !jobs[0] && !lite)return 0;
 policy[0]=0;strlcat(policy,policy_head,sizeof(policy));
 if(enabled)strlcat(policy,policy_watchdogd,sizeof(policy));
 if(lite)for(const char *c=lite_jobs;*c;) {if((error=append_job(&c,addon==1?lite_kept_for_addon:NULL)))return error;}
 for(const char *c=jobs;*c;) {if((error=append_job(&c,NULL)))return error;}
 strlcat(policy,policy_tail,sizeof(policy));
 const char *directory="/private/var/db/com.apple.xpc.launchd";
 const char *path="/private/var/db/com.apple.xpc.launchd/disabled.plist";
 if((error=copyout(directory,scratch,strlen(directory)+1)))goto done;
 struct mkdir_args md={.path=scratch,.mode=0755};
 error=mkdir(p,&md,&result);
 if(error && error!=EEXIST)goto done;
 if((error=copyout(path,scratch,strlen(path)+1)))goto done;
 // With persistent Data the previous boot's file is there: this policy is rewritten in place on every boot (it must follow the
 // boot arguments). Symlinks are still refused (O_NOFOLLOW) and the fstat check below insists on a root-owned regular file.
 struct open_args op={.path=scratch,.flags=O_WRONLY|O_CREAT|O_TRUNC|O_NOFOLLOW,.mode=0644};
 if((error=open(p,&op,&result)))goto done;
 fd=result;
 // The scratch page keeps the cache-mount environment slots at +0x400, so the policy (the lite list makes it longer than 0x200 bytes) goes
 // out in pieces through the 0x200 bytes at +0x200.
 for(size_t sent=0,total=strlen(policy);sent<total;) {
  size_t piece=total-sent<0x1f0?total-sent:0x1f0;
  if((error=copyout(policy+sent,scratch+0x200,piece)))goto done;
  struct write_args wr={.fd=fd,.cbuf=scratch+0x200,.nbyte=piece};
  user_ssize_t written=0;
  if((error=write(p,&wr,&written)))goto done;
  if(written!=(user_ssize_t)piece){error=EIO;goto done;}
  sent+=piece;
 }
 struct fchown_args owner={.fd=fd,.uid=0,.gid=0};
 if((error=fchown(p,&owner,&result)))goto done;
 struct fchmod_args mode={.fd=fd,.mode=0644};
 if((error=fchmod(p,&mode,&result)))goto done;
 struct fsync_args sync={.fd=fd};
 if((error=fsync(p,&sync,&result)))goto done;
 // Preserve the cache-mount environment slots at +0x400, +0x500, +0x600.
 struct fstat64_args sa={.fd=fd,.ub=scratch+0x800};struct stat64 st={0};
 if((error=sys_fstat64(p,&sa,&result)) || (error=copyin(sa.ub,&st,sizeof(st))))goto done;
 if(st.st_uid!=0 || st.st_gid!=0 || !S_ISREG(st.st_mode) || (st.st_mode&0777)!=0644 || st.st_size!=(off_t)(strlen(policy)))error=EINVAL;
 done:
 if(fd>=0){struct close_args cl={.fd=fd};int e=sys_close(p,&cl,&result);if(!error)error=e;}
 // A real empty migration marker selects the qualified modern override loader.
 if(!error) {
  const char *marker="/private/var/db/com.apple.xpc.launchd/disabled.migrated";
  error=copyout(marker,scratch,strlen(marker)+1);
  if(!error) {
   struct open_args mark={.path=scratch,.flags=O_WRONLY|O_CREAT|O_EXCL|O_NOFOLLOW,.mode=0644};
   error=open(p,&mark,&result);
   if(!error){struct close_args cl={.fd=result};error=sys_close(p,&cl,&result);}
   else if(error==EEXIST)error=0;
  }
 }
 sheng_watchdog_policy_status=error;
 printf("sheng bring-up watchdogd launch policy: %d (hardware watchdog not implemented)\n",error);
 return error;
}
