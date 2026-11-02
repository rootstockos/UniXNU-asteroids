// SPDX-License-Identifier: MIT
// Adreno Metal plugin, reached through the original discovery (IOAcceleratorES -> MetalPluginName -> NSBundle -> principal class ->
// initWithAcceleratorPort:). Default (stage 1): it runs the shared userspace GPU test (NativeAdrenoUserTest.h) from inside a real system
// process, under that process's sandbox, and then returns nil. No device is registered, so the original software rendering keeps running unchanged.
// Published (the image carries the file NativeAdrenoMetal.publish beside the bundle): backboardd, the one process that owns the GPU, returns the native
// Metal frontend's device (the metal2vulkan runtime on Turnip, on the Adreno 740), which the original discovery registers; the CARenderServer then
// renders the system UI through it, and the display driver's frames are written to the scanout by the GPU (GpuScanoutClient.h). Another process
// gets its own native device too (its own driver connection, GPU address space and shader cache), as every process does with Apple's
// own GPU plugin.
//
// asteroids T63: sheng's NativeAdrenoMetalPlugin.mm (UniXNU/AdrenoMetal feat/ipados27-gpu-parity ca7faaf, MIT) with the 3a's diagnostics
// carried over from the earlier asteroids plugin:
// - Turnip's switches from the kernel's boot arguments (setTurnipDebug): asteroids-tu-debug=<flags> adds TU_DEBUG flags in every process;
//   asteroids-tu-time, asteroids-tu-trace and asteroids-tu-light set TU_DW_TIME, TU_DW_TRACE and TU_DW_LIGHT in the render server only
//   (UniXNU patches/mesa/turnip-asteroids-timing.patch). The trace's directory is made here, parents included.
// - The render server's stacks (sampleThreads) on the status stream as "BT<n>" pieces, and the stack-only mode (NativeAdrenoMetal.sample).
// - supportsConditionalLoadStore answered NO on every device (T20).
// - The deep framebuffer turned off through backboardd's preferences (deepFramebufferOff), not by writing the render server's memory:
//   reportDisplayState only marks here, it never writes.
// - Only the processes NativeAdrenoMetal.clients names get a device ("*" names every process).
#import <Foundation/Foundation.h>
#include <pthread.h>
#include <dlfcn.h>
#if __has_feature(ptrauth_calls)
#include <ptrauth.h>
#endif
#include <unistd.h>
#include <pwd.h>
#include <fts.h>
#include <mach/mach.h>
#include <objc/message.h>
#include <objc/runtime.h>
#include <sys/stat.h>
#include <sys/sysctl.h>
#ifdef NATIVE_TURNIP_REPLAY
#error "TurnipReplay.h is an Adreno 740 recording: nothing recorded on another GPU is replayed on the A810"
#endif
#include "NativeAdrenoUserTest.h"
#include "NativeMetalBridgePhase.h"
#include "CacheExport.h"
#include "PrimeShaders.h"
#include "CacheImport.h"
#include "ExceptionDiag.h"

@interface _MTLDevice : NSObject
@end
@interface NativeAdrenoMetalDeviceBase : NSObject   // only a lock object for the published mode's one-time setup
@end
@implementation NativeAdrenoMetalDeviceBase
@end
@interface NativeAdrenoMetalDevice : _MTLDevice
- (id)initWithAcceleratorPort:(unsigned)port;
@end

// T54: Turnip's debug flags: the file /private/var/ngpu/tu_debug (sheng's) or nocb (A8xx renders without concurrent binning, T40e), plus
// asteroids-tu-debug=<flags> from the kernel's boot arguments for one boot ("gmem" or "sysmem" forces every pass's rendering mode; several
// flags joined with ':'), so GMEM and system memory can be compared without a rebuild. T55: asteroids-tu-time=<passes|all> turns on
// Turnip's GPU timing brackets (TU_DW_TIME). T61: asteroids-tu-trace=<letters> turns on Turnip's trace file (TU_DW_TRACE: p passes,
// d draws, s samples per draw, b shader binaries, t pass stages, a blits too; 1 = pdb); sheng-gpu-trace-export=<seconds> brings it home
// with the boot log, and scripts/gpu-trace.py reads it. T62: asteroids-tu-light=<percent> (TU_DW_LIGHT) renders a pass in system memory
// when its history draws less than that share of its area. Those three in the render server only.
static void bootArgument(const char *args, const char *name, char *out, size_t size) {
    const char *at = strstr(args, name);
    if (!at) return;
    at += strlen(name);
    size_t n = 0;
    while (at[n] && at[n] != ' ' && n + 1 < size) {out[n] = at[n]; ++n;}
    out[n] = 0;
}
static void setTurnipDebug(bool renderServer) {
    char flags[128] = "nocb", timing[16] = {}, trace[16] = {}, light[8] = {}, extra[96] = {};
    if (FILE *file = fopen("/private/var/ngpu/tu_debug", "r")) {
        char line[128] = {};
        if (fgets(line, sizeof line, file)) {line[strcspn(line, "\n")] = 0; if (line[0]) strlcpy(flags, line, sizeof flags);}
        fclose(file);
    }
    char args[1024] = {};
    size_t length = sizeof args - 1;
    if (sysctlbyname("kern.bootargs", args, &length, nullptr, 0) == 0) {
        bootArgument(args, "asteroids-tu-time=", timing, sizeof timing);
        bootArgument(args, "asteroids-tu-trace=", trace, sizeof trace);
        bootArgument(args, "asteroids-tu-light=", light, sizeof light);
        bootArgument(args, "asteroids-tu-debug=", extra, sizeof extra);
    }
    if (extra[0]) {
        for (char *c = extra; *c; ++c) if (*c == ':') *c = ',';
        strlcat(flags, ",", sizeof flags);
        strlcat(flags, extra, sizeof flags);
    }
    setenv("TU_DEBUG", flags, 1);
    if (!renderServer) return;
    if (timing[0]) setenv("TU_DW_TIME", timing, 1);   // "passes": render passes only; anything else: every kind
    if (light[0]) setenv("TU_DW_LIGHT", light, 1);
    if (trace[0]) {
        // Turnip makes only the last directory of m2v/gpu (T62b found none of it on the phone): make the whole chain first.
        if (const passwd *user = getpwuid(getuid()); user && user->pw_dir && user->pw_dir[0]) {
            char path[512];
            snprintf(path, sizeof path, "%s/Library/Caches/com.apple.backboardd/m2v/gpu", user->pw_dir);
            for (char *slash = strchr(path + 1, '/'); slash; slash = strchr(slash + 1, '/')) {*slash = 0; mkdir(path, 0755); *slash = '/';}
            mkdir(path, 0755);
        }
        setenv("TU_DW_TRACE", trace, 1);
    }
}
// T20: metal2vulkan declines supportsConditionalLoadStore on purpose (no document or AIR form names it), so the frontend installs no answer
// and Metal's _MTLDevice base class would decide. The bridge has no such feature: say no on the device itself.
static void refuseConditionalLoadStore(id device) {
    if (device)
        class_addMethod(object_getClass(device), sel_registerName("supportsConditionalLoadStore"), imp_implementationWithBlock(^BOOL(id) {
                            return NO;
                        }), "B16@0:8");
}

// ---- published mode ----
namespace publish {
// The image's marker: a file beside the bundle in the union view of /System/Library/Extensions (the bundle's own seal stays untouched).
// A file beside the bundle: its path in path, true when it is readable.
static bool requestedPath(const char *name, char *path, size_t size) {
    Dl_info info = {};
    if (!dladdr(reinterpret_cast<const void *>(&requestedPath), &info) || !info.dli_fname) return false;
    snprintf(path, size, "%s", info.dli_fname);
    char *slash = strrchr(path, '/');          // the bundle's executable: <dir>/NativeAdrenoMetal.bundle/NativeAdrenoMetal
    if (!slash) return false;
    *slash = 0;
    slash = strrchr(path, '/');
    if (!slash) return false;
    snprintf(slash + 1, size - size_t(slash + 1 - path), "%s", name);
    return access(path, R_OK) == 0;
}
static bool requested() {
    char path[1024];
    return requestedPath("NativeAdrenoMetal.publish", path, sizeof path);
}
static Gpu *makeGpu(unsigned port) {
    void *iokit = dlopen("/System/Library/Frameworks/IOKit.framework/IOKit", RTLD_NOW | RTLD_LOCAL);
    if (!iokit) return nullptr;
    Gpu *gpu = new Gpu;
    gpu->call = (CallFn)dlsym(iokit, "IOConnectCallStructMethod");
    gpu->map = (MapFn)dlsym(iokit, "IOConnectMapMemory64");
    gpu->unmap = (UnmapFn)dlsym(iokit, "IOConnectUnmapMemory64");
    gpu->openFn = (IOServiceOpenFn)dlsym(iokit, "IOServiceOpen");
    gpu->closeFn = (IOServiceCloseFn)dlsym(iokit, "IOServiceClose");
    gpu->port = port;
    return gpu;
}
// T20: where backboardd's threads wait. Each thread that has a QuartzCore frame (the render server's), apart from this plugin's status and
// scanout threads, as "<name>:" and up to eight frames "<image letter><offset in the image, hex>": the interrupted pc, the link register,
// then the frame-pointer chain. The thread is not stopped: a blocked thread's state is exact, a running one's a sample. Memory is read
// with vm_read_overwrite, so a bad frame pointer ends the walk instead of faulting. Offsets go back to symbols offline.
static mach_port_t gStatusThread, gScanoutThread;
static char imageLetter(const char *path) {
    const char *base = strrchr(path, '/');
    base = base ? base + 1 : path;
    static const struct { const char *name; char letter; } known[] = {
        {"QuartzCore", 'Q'}, {"Metal", 'M'}, {"NativeAdrenoMetal", 'P'}, {"libsystem_kernel.dylib", 'K'}, {"libsystem_pthread.dylib", 'T'},
        {"libdispatch.dylib", 'D'}, {"libobjc.A.dylib", 'O'}, {"IOKit", 'I'}, {"IOSurface", 'S'}, {"CoreFoundation", 'F'},
        {"libvulkan_freedreno.dylib", 'V'}, {"backboardd", 'B'}, {"libc++abi.dylib", 'A'}, {"libsystem_c.dylib", 'C'}};
    for (const auto &k : known)
        if (!strcmp(base, k.name)) return k.letter;
    return (char)tolower((unsigned char)base[0]);
}
static void sampleThreads(char *out, size_t size) {
    out[0] = 0;
    thread_act_array_t threads = nullptr;
    mach_msg_type_number_t count = 0;
    if (task_threads(mach_task_self(), &threads, &count) != KERN_SUCCESS) {
        snprintf(out, size, "no threads");
        return;
    }
    size_t used = (size_t)snprintf(out, size, "n%u ", count);
    const mach_port_t self = mach_thread_self();
    for (mach_msg_type_number_t i = 0; i < count; ++i) {
        const mach_port_t t = threads[i];
        uint64_t frames[16];
        unsigned n = 0;
        arm_thread_state64_t state;
        mach_msg_type_number_t words = ARM_THREAD_STATE64_COUNT;
        if (t != self && t != gStatusThread && t != gScanoutThread &&
            thread_get_state(t, ARM_THREAD_STATE64, (thread_state_t)&state, &words) == KERN_SUCCESS) {
            frames[n++] = arm_thread_state64_get_pc(state);
            frames[n++] = arm_thread_state64_get_lr(state);
            uint64_t fp = arm_thread_state64_get_fp(state);
            while (n < 16 && fp && !(fp & 7)) {
                uint64_t record[2] = {};   // the caller's frame pointer, then the return address
                vm_size_t got = 0;
                if (vm_read_overwrite(mach_task_self(), (vm_address_t)fp, sizeof record, (vm_address_t)record, &got) != KERN_SUCCESS ||
                    got != sizeof record || !record[1])
                    break;
                frames[n++] = record[1];
                if (record[0] <= fp) break;
                fp = record[0];
            }
        }
        bool render = false;
        for (unsigned f = 0; f < n && !render; ++f) {
            Dl_info info;
            render = dladdr((const void *)frames[f], &info) && info.dli_fname && imageLetter(info.dli_fname) == 'Q';
        }
        if (render && used + 24 < size) {
            char name[64] = "?";
            if (pthread_t p = pthread_from_mach_thread_np(t)) pthread_getname_np(p, name, sizeof name);
            const char *last = strrchr(name, '.');
            used += (size_t)snprintf(out + used, size - used, "%.10s:", last ? last + 1 : name);
            for (unsigned f = 0; f < n && f < 8 && used + 12 < size; ++f) {
                Dl_info info;
                if (dladdr((const void *)frames[f], &info) && info.dli_fname)
                    used += (size_t)snprintf(out + used, size - used, "%c%llx ", imageLetter(info.dli_fname),
                                             (unsigned long long)(frames[f] - (uint64_t)info.dli_fbase));
                else
                    used += (size_t)snprintf(out + used, size - used, "?%llx ", (unsigned long long)frames[f]);
            }
            if (used + 2 < size) out[used++] = '|', out[used] = 0;
        }
    }
    for (mach_msg_type_number_t i = 0; i < count; ++i) mach_port_deallocate(mach_task_self(), threads[i]);
    vm_deallocate(mach_task_self(), (vm_address_t)threads, count * sizeof *threads);
    mach_port_deallocate(mach_task_self(), self);
}
// T22b: the render server's deep (10-bit, extended range) framebuffer, which this display chain cannot show, off through the
// CAEnableDeepFramebuffer preference next to the two compression ones, in backboardd's own domain (kept across boots on the persistent Data),
// so a render server that read it before this plugin loaded still sees it on the next boot. Bits as marker 0x6Axx: 1 the preference was
// already set, 2 it was false, 4 MobileGestalt's deep-framebuffer answer is true, 8 that answer exists, 16 the compression capability
// answer exists, 32 not root.
static unsigned deepFramebufferOff() {
    unsigned bits = 0;
    if (CFTypeRef value = CFPreferencesCopyAppValue(CFSTR("CAEnableDeepFramebuffer"), kCFPreferencesCurrentApplication)) {
        bits |= 1;
        if (CFGetTypeID(value) == CFBooleanGetTypeID() && !CFBooleanGetValue((CFBooleanRef)value)) bits |= 2;
        CFRelease(value);
    }
    if (auto copyAnswer = (CFTypeRef (*)(CFStringRef))dlsym(RTLD_DEFAULT, "MGCopyAnswer")) {
        if (CFTypeRef deep = copyAnswer(CFSTR("Aixt/MEN2O2B7f+8m4TxUA"))) {
            bits |= 8;
            if (CFGetTypeID(deep) == CFBooleanGetTypeID() && CFBooleanGetValue((CFBooleanRef)deep)) bits |= 4;
            CFRelease(deep);
        }
        if (CFTypeRef caps = copyAnswer(CFSTR("AoKnINTLPoKML3ctoP0AZg"))) {bits |= 16; CFRelease(caps);}
    }
    if (getuid() != 0) bits |= 32;
    CFPreferencesSetAppValue(CFSTR("CAEnableDeepFramebuffer"), kCFBooleanFalse, kCFPreferencesCurrentApplication);
    CFPreferencesSetAppValue(CFSTR("CAEnableFramebufferCompression"), kCFBooleanFalse, kCFPreferencesCurrentApplication);
    CFPreferencesSetAppValue(CFSTR("CAEnableIOSurfaceCompression"), kCFBooleanFalse, kCFPreferencesCurrentApplication);
    CFPreferencesAppSynchronize(kCFPreferencesCurrentApplication);
    return bits;
}
// T31: whether NativeAdrenoMetal.clients beside the bundle names this process (names separated by blanks, commas or new lines; "*" names
// every process, the add-on's list since T62).
static bool listedClient() {
    char path[1024];
    requestedPath("NativeAdrenoMetal.clients", path, sizeof path);
    FILE *file = fopen(path, "r");
    if (!file) return false;
    char text[1024] = {};
    const size_t length = fread(text, 1, sizeof text - 1, file);
    fclose(file);
    text[length] = 0;
    const char *self = getprogname();
    for (char *save = nullptr, *name = strtok_r(text, " ,\t\r\n", &save); name; name = strtok_r(nullptr, " ,\t\r\n", &save))
        if (!strcmp(name, "*") || !strcmp(name, self)) return true;
    return false;
}
static std::string gCacheDirectory;   // the runtime's cache directory (the one that opened the device), exported by CacheExport.h
// Progress markers 0x68xx (GPU6): 6800 entered, 6801 Turnip found, 6802 device made, 6803 returned to the discovery, 68f1 no Turnip, 68f2 no device.
static unsigned gDiagBits;
// Reads of the render server's own state for the framebuffer-format question (diagnostic only, for the exact QuartzCore of the target image; a read
// that fails or a build that differs reports zeros). Each value is one marker, 0x7Kvv: K picks the item, vv the value.
static bool readTask(uintptr_t address, void *out, size_t size) {
    vm_size_t got = 0;
    return vm_read_overwrite(mach_task_self(), address, size, (vm_address_t) out, &got) == KERN_SUCCESS && got == size;
}
[[maybe_unused]] static bool writeTask(uintptr_t address, const void *data, size_t size) {   // arm64e only on asteroids
    return vm_write(mach_task_self(), address, (vm_offset_t) data, (mach_msg_type_number_t) size) == KERN_SUCCESS;
}
// EXPERIMENT (second report onward): this display path takes plain 8-bit BGRA only, so tell the render server's display that it has neither the deep
// (10-bit) framebuffer nor universal buffer compression. Whether the state it reads those from is the whole story is what the markers show.
static unsigned gReports;
struct DisplaySample {unsigned flags = 0, mode = 0, behaviour = 0, globals = 0; uint16_t display = 0; uintptr_t object = 0;};
static DisplaySample sampleDisplay(id first, intptr_t slide) {
    DisplaySample out;
    uintptr_t a = first ? (uintptr_t) *(void **) ((char *) (__bridge void *) first + 8) : 0, b = 0, d = 0;
    if (a && readTask(a + 0x40, &b, 8) && b && readTask(b + 0x60, &d, 8) && d) {
        uint32_t f = 0; uint64_t m = 0; uint8_t byte[4] = {};
        readTask(d + 0x238, &f, 4);
        readTask(d + 0x258, &out.display, 2);
        readTask(d + 0x248, &m, 8);
        readTask(d + 0x6c17, &byte[0], 1);
        readTask(d + 0x6658 + 0x57d, &byte[1], 1);
        readTask(d + 0x6658 + 0x58a, &byte[2], 1);
        readTask(d + 0x6658 + 0x59b, &byte[3], 1);
        out.object = d;
        out.flags = f & 0xff;
        out.mode = (unsigned) ((m >> 55) & 31) | (m ? 0x20 : 0);
        out.behaviour = (byte[0] ? 1 : 0) | (byte[1] ? 2 : 0) | (byte[2] ? 4 : 0) | (byte[3] ? 8 : 0);
    }
    uint8_t v[4] = {};
    readTask(0x1f491ae10 + slide, &v[0], 1);    // the render server's deep-framebuffer flag
    readTask(0x1ed7abf10 + slide, &v[1], 1);    // framebuffer compression in effect
    readTask(0x1f4919dc0 + slide, &v[2], 1);    // its default
    readTask(0x1f4919c20 + slide, &v[3], 1);    // universal buffer compression
    out.globals = (v[0] ? 1 : 0) | (v[1] ? 2 : 0) | (v[2] ? 4 : 0) | (v[3] ? 8 : 0);
    return out;
}
static void markSample(Gpu &gpu, const DisplaySample &x, unsigned base) {
    gpu.mark(base | (x.flags << 4) | x.globals);                  // display flags (bit 3 deep), globals: deep/compression/default/universal
    gpu.mark((base + 0x1000) | (x.display >> 4));                 // display limits word: 0x200 compression support, 0x1c00 type
    gpu.mark((base + 0x2000) | (x.mode << 4) | x.behaviour);     // mode colour field, ICC/other behaviour bytes
}
static void reportDisplayState(Gpu &gpu) {
#if defined(__arm64e__)
    (void)gpu;
    return;   // the addresses above are the 21H461 cache's; the arm64e (iPadOS 27) build must not read or write them
#endif
    ++gReports;
    static DisplaySample before;
    static bool changed;
    Class serverClass = NSClassFromString(@"CAWindowServer");
    void *start = dlsym(RTLD_DEFAULT, "CARenderServerStart");
    if (!serverClass || !start) return;
    const intptr_t slide = (intptr_t) start - 0x189651efc;
    @try {
        id server = ((id (*)(id, SEL)) objc_msgSend)(serverClass, NSSelectorFromString(@"serverIfRunning"));
        NSArray *displays = server ? ((id (*)(id, SEL)) objc_msgSend)(server, NSSelectorFromString(@"displays")) : nil;
        id first = displays.count ? displays[0] : nil;
        DisplaySample now = sampleDisplay(first, slide);
        // asteroids: read only. Sheng's experiment cleared the display's deep flag and universal compression in the render server's
        // memory at fixed addresses from the second report on; this board turns the deep framebuffer off by preference
        // (deepFramebufferOff) and writes nothing into the render server.
        if (!changed) before = now;
        markSample(gpu, before, 0x7000);
        markSample(gpu, now, 0xA000);
    } @catch (NSException *) {}
}
// Every process other than the render server: the same runtime on its own connection, with none of the render server's threads (no
// display hand-off, telemetry, benchmark or cache export). Its shader cache lives in its own cache or temp directory.
// The reason a client's device failed, as a text line the driver keeps per process (the render server's report row is rewritten every ten seconds):
// "<step> <detail>", through the marker stream (a connection is not needed). The Gpu object is made only on this path.
// Deletes every "*.import" file under `root`: the copying import of an earlier image wrote each file beside its place and renamed it only
// once the whole pack was read; a boot that ended first left them all there.
static void removeHalfImported(const char *root) {
    char *paths[] = {const_cast<char *>(root), nullptr};
    FTS *fts = fts_open(paths, FTS_PHYSICAL | FTS_NOCHDIR | FTS_XDEV, nullptr);
    if (!fts) return;
    while (FTSENT *e = fts_read(fts))
        if (e->fts_info == FTS_F && e->fts_namelen > 7 && !strcmp(e->fts_name + e->fts_namelen - 7, ".import")) unlink(e->fts_path);
    fts_close(fts);
}
static unsigned gClientPort;
static void clientFailure(unsigned port, const char *step, const char *detail) {
    if (Gpu *gpu = makeGpu(port)) {
        // The row holds 63 characters: the common prefix goes, and a long reason keeps its start (what was refused) and its end (why).
        if (!detail) detail = "";
        if (!strncmp(detail, "native-metal: ", 14)) detail += 14;
        char shown[48];
        const size_t length = strlen(detail);
        if (length <= 46) strlcpy(shown, detail, sizeof shown);
        else snprintf(shown, sizeof shown, "%.18s~%s", detail, detail + length - 27);
        char text[64];
        snprintf(text, sizeof text, "%.14s %s", step, shown);
        streamText(*gpu, text);
    }
}
static id clientDevice(unsigned port) {
    char turnip[1024];
    if (!turnip::libraryPath(turnip, sizeof turnip) || access(turnip, R_OK) != 0) {clientFailure(port, "no-turnip", ""); return nil;}
    uint64_t registryID = 0;
    if (void *iokit = dlopen("/System/Library/Frameworks/IOKit.framework/IOKit", RTLD_NOW | RTLD_LOCAL)) {
        if (auto retain = (int (*)(unsigned))dlsym(iokit, "IOObjectRetain")) retain(port);
        if (auto entryID = (int (*)(unsigned, uint64_t *))dlsym(iokit, "IORegistryEntryGetRegistryEntryID")) entryID(port, &registryID);
    }
    // IOSurfaces this GPU reads or writes must be plain rows, as in the render server.
    CFPreferencesSetAppValue(CFSTR("CAEnableIOSurfaceCompression"), kCFBooleanFalse, kCFPreferencesCurrentApplication);
    char candidates[2][512] = {};
    if (!confstr(_CS_DARWIN_USER_CACHE_DIR, candidates[0], sizeof candidates[0] - 16)) candidates[0][0] = 0;
    if (!confstr(_CS_DARWIN_USER_TEMP_DIR, candidates[1], sizeof candidates[1] - 16)) candidates[1][0] = 0;
    id device = nil;
    const char *lastError = "";
    unsigned usable = 0;
    for (unsigned i = 0; i < 2 && !device; ++i) {
        if (!candidates[i][0]) continue;
        strlcat(candidates[i], "m2v", sizeof candidates[i]);
        mkdir(candidates[i], 0700);
        if (access(candidates[i], W_OK) != 0) continue;
        ++usable;
        @try {
            device = native_bridge_metal_open(turnip, candidates[i], registryID, nullptr);
        } @catch (NSException *exception) {
            device = nil;
            lastError = "exception";
        }
        if (!device && !lastError[0]) lastError = nmb_last_error();
    }
    if (!device && !usable) {
        setenv("METAL2VULKAN_SHADER_CACHE_OPTIONAL", "1", 1);
        const char *path = candidates[0][0] ? candidates[0] : candidates[1][0] ? candidates[1] : "/var/mobile/tmp/m2v";
        @try {
            device = native_bridge_metal_open(turnip, path, registryID, nullptr);
        } @catch (NSException *exception) {
            device = nil;
            lastError = "exception";
        }
        if (!device && !lastError[0]) lastError = nmb_last_error();
    }
    if (!device) clientFailure(port, usable ? "no-device" : "no-cachedir", lastError);
    refuseConditionalLoadStore(device);
    // The client's first problem, once, on the same text line, sent from the thread that hit it: WidgetRenderer crashed on a nil render
    // encoder with no reason anywhere readable (its own log lines need the plugin's format strings, which the device does not keep), and a
    // once-a-second reader never ran before the crash (i568).
    if (device) {
        gClientPort = port;
        native_bridge_metal_problem_hook = [](const char *problem) {clientFailure(gClientPort, "problem", problem);};
        char problem[200] = {};
        if (native_bridge_metal_first_problem(problem, sizeof problem) && problem[0]) clientFailure(port, "problem", problem);
    }
    return device;
}
static id deviceForPort(unsigned port) {
    static id device;
    static bool tried;
    @synchronized([NativeAdrenoMetalDeviceBase class]) {
        if (tried) return device;
        tried = true;
        exceptiondiag::start(); // the throw site of an uncaught exception, for the kernel's THROW row (ExceptionDiag.h)
        // No concurrent binning in any Metal process: with it BV ran ahead of BR and the two stopped waiting on each other inside a pass
        // (int-261p, int-262, and in system memory int-265). Without it Turnip's autotune picks tile memory (GMEM) or system memory per
        // pass: i553 forced system memory for every pass, the GPU busy 19 % at 680 MHz. A tile store writes whole 16x4 blocks, which the
        // runtime keeps within a texture's rows (bridge patch 0016). asteroids: plus the boot arguments' switches (setTurnipDebug).
        const bool renderServer = strcmp(getprogname(), "backboardd") == 0;
        setTurnipDebug(renderServer);
        if (!renderServer) return device = listedClient() ? clientDevice(port) : nil;
        // The discovery releases the accelerator port when it is done; the monitor and the scanout client keep using it.
        if (void *iokit = dlopen("/System/Library/Frameworks/IOKit.framework/IOKit", RTLD_NOW | RTLD_LOCAL)) {
            if (auto retain = (int (*)(unsigned))dlsym(iokit, "IOObjectRetain")) retain(port);
        }
        Gpu *gpu = makeGpu(port);
        if (!gpu) return nil;
        gpu->mark(0x6800);
        char turnip[1024];
        if (!turnip::libraryPath(turnip, sizeof turnip) || access(turnip, R_OK) != 0) {gpu->mark(0x68f1); return nil;}
        gpu->mark(0x6801);
        // The runtime refuses to open without a writable shader-cache directory, and which directories backboardd's sandbox lets it write is not
        // known from here: try the user cache and temp directories, then /tmp and /var/tmp, and use the first that opens a device.
        uint64_t registryID = 0;
        if (void *iokit = dlopen("/System/Library/Frameworks/IOKit.framework/IOKit", RTLD_NOW | RTLD_LOCAL)) {
            if (auto entryID = (int (*)(unsigned, uint64_t *))dlsym(iokit, "IORegistryEntryGetRegistryEntryID")) entryID(port, &registryID);
        }
        // What the render server sees for its framebuffer choice (marker 0x6Axx): bit0 the deep-framebuffer preference is set, bit1 it is false,
        // bit2 MobileGestalt's answer behind the render server's deep flag is true, bit3 it exists, bit4 the compression capability dictionary exists,
        // bit5 the process is not root.
        // asteroids: the same bits, and the deep framebuffer turned off by preference (deepFramebufferOff).
        gDiagBits = deepFramebufferOff();
        gpu->mark(0x6A00 | gDiagBits);
        // The render server compresses its framebuffer (an Apple/AGX layout this GPU does not write) unless told not to; the display driver takes plain rows.
        setenv("CA_DISABLE_FRAMEBUFFER_COMPRESSION", "1", 1);
#if defined(__arm64e__)
        // iPadOS 27 picks a 64-bit half-float framebuffer ('RGhA') whenever the display's EDR headroom is above 1 (this panel reports XDR
        // headroom), which the display path can only show converted, slowly and greyed. As on iPadOS 17, where the render server drew plain
        // 8-bit BGRA, the headroom is held at 1: the render server's headroom override (CA::WindowServer::Display::edr_headroom reads it
        // first when nonzero; 24A446 QuartzCore __DATA_DIRTY 0x1e7021014, CARenderServerStart at 0x1846dce88 for the cache slide) and its
        // environment spelling for an option table read after this.
        setenv("CA_DISPLAY_EDR_HEADROOM", "1", 1);
        if (void *start = dlsym(RTLD_DEFAULT, "CARenderServerStart")) {
            const uintptr_t slide = (uintptr_t)ptrauth_strip(start, ptrauth_key_function_pointer) - 0x1846dce88;
            const float one = 1.0f;
            gpu->mark(writeTask(0x1e7021014 + slide, &one, sizeof one) ? 0x6E01 : 0x6EF1);
        }
#endif
        // On an iPad the render server's queue runs at a high GPU priority (CoreAnimation asks for it with setGPUPriority: when it makes
        // its MetalContext). A Vulkan queue's priority is fixed when the device opens, before that call, so the render server asks the
        // runtime for it here (patches/metal2vulkan/0012): where the host offers priorities, its frames no longer wait on the GPU behind
        // an app's.
        setenv("METAL2VULKAN_QUEUE_PRIORITY", "high", 0);
        // The runtime's creation log reports each VkPipeline making's time to the device's sink, which the telemetry's V row sums.
        setenv("METAL2VULKAN_LOG_PIPELINES", "1", 0);
        CFPreferencesSetAppValue(CFSTR("CAEnableFramebufferCompression"), kCFBooleanFalse, kCFPreferencesCurrentApplication);
        CFPreferencesSetAppValue(CFSTR("CAEnableIOSurfaceCompression"), kCFBooleanFalse, kCFPreferencesCurrentApplication);
        char candidates[5][512] = {};
        if (!confstr(_CS_DARWIN_USER_CACHE_DIR, candidates[0], sizeof candidates[0] - 16)) candidates[0][0] = 0;
        if (!confstr(_CS_DARWIN_USER_TEMP_DIR, candidates[1], sizeof candidates[1] - 16)) candidates[1][0] = 0;
        strlcpy(candidates[2], "/tmp/", sizeof candidates[2]);
        strlcpy(candidates[3], "/private/var/tmp/", sizeof candidates[3]);
        // The user temp directory of the mobile user is under /var/mobile/tmp, which launchd's boot task dirs_cleaner empties at every boot,
        // so with a persistent Data store the pipelines were still compiled again each boot (V row hit0). Try the user's Library/Caches first
        // (marker 0x6814); the earlier candidates keep their markers.
        if (const passwd *user = getpwuid(getuid()); user && user->pw_dir && user->pw_dir[0]) {
            snprintf(candidates[4], sizeof candidates[4] - 16, "%s/Library/Caches/com.apple.backboardd", user->pw_dir);
            mkdir(candidates[4], 0700);
            strlcat(candidates[4], "/", sizeof candidates[4]);
        }
        static const unsigned order[] = {4, 0, 1, 2, 3};
        for (unsigned i : order) {
            if (device) break;
            if (!candidates[i][0]) continue;
            strlcat(candidates[i], "m2v", sizeof candidates[i]);
            mkdir(candidates[i], 0700);
            if (access(candidates[i], W_OK) != 0) continue;
            // The pipeline caches compiled ahead on the host, read on demand from the userdata import region behind this directory
            // (CacheImport.h), and the files an earlier image's copying import left half written there (int 590aaee2), which hold memory
            // of the RAM Data disk.
            {
                static bool once;
                if (!once) {
                    once = true;
                    removeHalfImported(candidates[i]);
                    if (cacheimport::open(*gpu)) {
                        nmb_set_shader_cache_fallback(cacheimport::read);
                        uint8_t *done = nullptr;
                        size_t length = 0;
                        if (cacheimport::read("prime-done", &done, &length)) {
                            const std::string text(reinterpret_cast<const char *>(done), length);
                            free(done);
                            for (size_t at = 0, end; at < text.size(); at = end + 1) {
                                end = text.find('\n', at);
                                if (end == std::string::npos) end = text.size();
                                if (end > at) prime::extraDone().insert(text.substr(at, end - at));
                            }
                        }
                    }
                }
            }
            // The image's pipeline pack (an earlier boot's export), into this directory before the runtime reads it. Marker 0x6Cnn: files written
            // (saturated at 0xfe), 0x6Cff the pack was refused.
            if (char pack[1024]; requestedPath("NativeAdrenoMetal.m2vpack", pack, sizeof pack)) {
                const int imported = cacheexport::importPack(pack, candidates[i]);
                if (imported) gpu->mark(0x6C00 | (imported < 0 ? 0xff : imported > 0xfe ? 0xfe : imported));
            }
            @try {
                device = native_bridge_metal_open(turnip, candidates[i], registryID, nullptr);
            } @catch (NSException *exception) {
                device = nil;
            }
            if (device) {gpu->mark(0x6810 + i); gCacheDirectory = candidates[i];}   // which directory served
        }
        if (!device) {
            gpu->mark(0x68f2);
            char text[64];
            snprintf(text, sizeof text, "PUB no device: %.40s", nmb_last_error());
            streamText(*gpu, text);
            return nil;
        }
        gpu->mark(0x6802);
        refuseConditionalLoadStore(device);
        // The status line (GPU8) every ten seconds, and the display hand-off client on its own thread: both only read or serve, neither is the discovery's.
        pthread_t monitor, scanout;
        pthread_attr_t attributes;
        pthread_attr_init(&attributes);
        pthread_attr_setdetachstate(&attributes, PTHREAD_CREATE_DETACHED);
        pthread_attr_setstacksize(&attributes, 1u << 20);
        pthread_create(&monitor, &attributes, [](void *arg) -> void * {
            Gpu *g = static_cast<Gpu *>(arg);
            gStatusThread = mach_thread_self();
            // asteroids: after the status pieces, the render server's stacks (sampleThreads, taken once a round) in up to eight "BT<n>" pieces.
            static char stacks[460];
            unsigned statusPieces = 1, stackPieces = 0;
            for (unsigned step = 0;; ++step) {
                sleep(10);
                char status[64], text[64];
                // status (counters, then the last two refusals' texts), formats; the stream carries 63 characters per line, so a long status goes
                // out in 57-character pieces on consecutive lines (the UFS log keeps every line).
                static char full[420];
                if (step >= 1 + statusPieces + stackPieces) step = 0;
                if (step == 0) {
                    native_bridge_metal_status(full, sizeof full);
                    sampleThreads(stacks, sizeof stacks);
                    statusPieces = (unsigned)((strlen(full) + 56) / 57);
                    statusPieces = statusPieces < 1 ? 1 : statusPieces > 7 ? 7 : statusPieces;
                    stackPieces = (unsigned)((strlen(stacks) + 56) / 57);
                    stackPieces = stackPieces > 8 ? 8 : stackPieces;
                    native_bridge_metal_formats(status, sizeof status);
                    snprintf(text, sizeof text, "PUB %.58s", status);
                } else if (step <= statusPieces) {
                    snprintf(text, sizeof text, "ST%u %.57s", step - 1, full + (size_t)(step - 1) * 57);
                } else {
                    const unsigned piece = step - 1 - statusPieces;
                    snprintf(text, sizeof text, "BT%u %.57s", piece, stacks + (size_t)piece * 57);
                }
                streamText(*g, text);
                reportDisplayState(*g);
                g->mark(0x6A00 | gDiagBits);   // the diagnostics page lists the last few markers only
            }
            return nullptr;
        }, gpu);
        pthread_t telemetry;
        pthread_create(&telemetry, &attributes, [](void *arg) -> void * {
            Gpu *g = static_cast<Gpu *>(arg);
            sleep(5);
            // The bridge's telemetry table, resent whole every four seconds as markers (two characters each): 0xC000|row starts a row of the
            // kernel's table, 0x8000|(a<<7)|b appends two characters. Rows are at most 90 characters; 48 rows.
            for (;;) {
                static char table[6144];
                native_bridge_metal_telemetry(table, sizeof table);
                unsigned row = 0;
                for (char *line = table; *line && row < 48;) {
                    char *end = strchr(line, '\n');
                    if (end) *end = 0;
                    g->mark(0xC000u | row);
                    const size_t length = strlen(line) < 90 ? strlen(line) : 90;
                    for (size_t i = 0; i < length; i += 2) {
                        const unsigned a = (unsigned char)line[i] & 0x7f, b = i + 1 < length ? (unsigned char)line[i + 1] & 0x7f : 0;
                        g->mark(0x8000u | (a << 7) | b);
                    }
                    ++row;
                    if (!end) break;
                    line = end + 1;
                }
                sleep(6);
            }
            return nullptr;
        }, gpu);
        pthread_create(&scanout, &attributes, [](void *arg) -> void * {
            Gpu *g = static_cast<Gpu *>(arg);
            gScanoutThread = mach_thread_self();
            sleep(3);
            @try {
                bridgephase::serveScanout(*g, device);
            } @catch (NSException *exception) {
                g->mark(0x68f3);
            }
            return nullptr;
        }, gpu);
        // The cache directory's export to the UFS log only when NativeAdrenoMetal.cacheexport sits beside the bundle: Data persists across
        // boots now, so the caches no longer need the trip through the host, and once priming has filled the directory, every rescan and
        // export competed with the render server (int-245: the scanning thread at 45% CPU, the sending one at 48%, frames twice as slow).
        char exportPath[1024];
        if (requestedPath("NativeAdrenoMetal.cacheexport", exportPath, sizeof exportPath)) {
            pthread_t exporter;
            pthread_create(&exporter, &attributes, [](void *arg) -> void * {
                cacheexport::run(*static_cast<Gpu *>(arg), gCacheDirectory);
                return nullptr;
            }, gpu);
        }
        // Every Metal library of the OS through the runtime, 90 s after the render server opened the device (PrimeShaders.h); the boots that froze
        // or crashed before three minutes never primed anything. Progress is kept in the Data cache directory, so later boots resume.
        // The translator recurses deeply on large functions: give it the stack of a main thread.
        pthread_attr_setstacksize(&attributes, 8u << 20);
        pthread_t primer;
        pthread_create(&primer, &attributes, [](void *) -> void * {
            char list[1024];
            prime::run((id<MTLDevice>)device, gCacheDirectory, 90,
                       requestedPath("NativeAdrenoMetal.primelist", list, sizeof list) ? std::string(list) : std::string());
            return nullptr;
        }, nullptr);
        pthread_attr_destroy(&attributes);
        gpu->mark(0x6803);
        return device;
    }
}
// T22: with NativeAdrenoMetal.sample beside the bundle (and no .publish), no Metal device at all: the discovery gets nil, so the render
// server draws on the CPU, while this thread streams its stacks (sampleThreads) every ten seconds as "BT<n>" pieces, and "SMP <threads>"
// once a round. Marker 6900 started.
static void sampleOnly(unsigned port) {
    static bool started;
    @synchronized([NativeAdrenoMetalDeviceBase class]) {
        if (started) return;
        started = true;
    }
    if (void *iokit = dlopen("/System/Library/Frameworks/IOKit.framework/IOKit", RTLD_NOW | RTLD_LOCAL)) {
        if (auto retain = (int (*)(unsigned))dlsym(iokit, "IOObjectRetain")) retain(port);
    }
    Gpu *gpu = makeGpu(port);
    if (!gpu) return;
    gpu->mark(0x6900);
    setenv("CA_DISABLE_FRAMEBUFFER_COMPRESSION", "1", 1);
    gDiagBits = deepFramebufferOff();
    gpu->mark(0x6A00 | gDiagBits);
    pthread_t thread;
    pthread_attr_t attributes;
    pthread_attr_init(&attributes);
    pthread_attr_setdetachstate(&attributes, PTHREAD_CREATE_DETACHED);
    pthread_attr_setstacksize(&attributes, 1u << 20);
    pthread_create(&thread, &attributes, [](void *arg) -> void * {
        Gpu *g = static_cast<Gpu *>(arg);
        gStatusThread = mach_thread_self();
        char stacks[460] = "", text[64];
        unsigned pieces = 0;
        for (unsigned step = 0;; ++step) {
            sleep(10);
            if (step >= 1 + pieces) step = 0;
            if (step == 0) {
                sampleThreads(stacks, sizeof stacks);
                pieces = (unsigned)((strlen(stacks) + 56) / 57);
                pieces = pieces > 8 ? 8 : pieces;
                snprintf(text, sizeof text, "SMP d%02x %.54s", gDiagBits, stacks);
            } else {
                snprintf(text, sizeof text, "BT%u %.57s", step - 1, stacks + (size_t)(step - 1) * 57);
            }
            streamText(*g, text);
        }
        return nullptr;
    }, gpu);
    pthread_attr_destroy(&attributes);
}
}  // namespace publish

@implementation NativeAdrenoMetalDevice
- (id)initWithAcceleratorPort:(unsigned)port {
    self = [super init];
    if (!self || !port) return nil;
    if (publish::requested()) return publish::deviceForPort(port);
    if (char sample[1024]; publish::requestedPath("NativeAdrenoMetal.sample", sample, sizeof sample)) {
        if (strcmp(getprogname(), "backboardd") == 0) publish::sampleOnly(port);
        return nil;
    }
    static bool attempted;
    if (attempted) return nil;
    attempted = true;
    // The test runs on its own thread: a kernel call that never returns (int-042: BoFree) then stalls that thread only, not SpringBoard's
    // main thread (the UI froze with the test running inline).
    // The discovery code releases the accelerator port as soon as this returns nil, which is before the test thread has started: the first
    // marker (and every later open) then went to a dead name and never reached the kernel (K0 in int-043..052, with the plugin file opened by
    // SpringBoard). Keep one extra user reference on the port, and let the thread look the service up itself as well.
    if (void *iokit = dlopen("/System/Library/Frameworks/IOKit.framework/IOKit", RTLD_NOW | RTLD_LOCAL)) {
        if (auto retain = (int (*)(unsigned))dlsym(iokit, "IOObjectRetain")) retain(port);
    }
    struct Job {unsigned port; char name[32];};
    Job *job = new Job{port, {}};
    strlcpy(job->name, getprogname(), sizeof job->name);
    pthread_t thread;
    pthread_attr_t attributes;
    pthread_attr_init(&attributes);
    pthread_attr_setdetachstate(&attributes, PTHREAD_CREATE_DETACHED);
    if (pthread_create(&thread, &attributes, [](void *arg) -> void * {
            Job *j = static_cast<Job *>(arg);
            unsigned port = j->port;
            if (void *iokit = dlopen("/System/Library/Frameworks/IOKit.framework/IOKit", RTLD_NOW | RTLD_LOCAL)) {
                auto matching = (void *(*)(const char *))dlsym(iokit, "IOServiceMatching");
                auto getService = (unsigned (*)(unsigned, void *))dlsym(iokit, "IOServiceGetMatchingService");
                if (matching && getService) {
                    const unsigned own = getService(0, matching("IOAcceleratorES"));
                    if (own) port = own;
                }
            }
            gAfterTurnip = bridgephase::run;
            setTurnipDebug(false);   // A8xx: nocb, as in the published mode
            adrenoUserTest(port, j->name);
            delete j;
            return nullptr;
        }, job) != 0) delete job;
    pthread_attr_destroy(&attributes);
    return nil;
}
@end
