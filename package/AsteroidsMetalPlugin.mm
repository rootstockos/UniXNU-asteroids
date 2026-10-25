// SPDX-License-Identifier: MIT
// asteroids T17/T18: the Metal plugin. The original discovery in backboardd (IOServiceMatching("IOAcceleratorES") -> MetalPluginName
// "NativeAdrenoMetal" -> /System/Library/Extensions/NativeAdrenoMetal.bundle -> principal class -> initWithAcceleratorPort:) reaches this class
// through the GPU addon's Extensions overlay; GPU power mode 19 publishes the accelerator it reads.
// Test stage (T17, the default): like the default stage of UniXNU's drivers/AdrenoMetal/NativeAdrenoMetalPlugin.mm, it runs the shared user test
// (NativeAdrenoUserTest.h: two rounds of three CP_MEM_WRITEs through its own mappings, then the Turnip smoke test with the libvulkan_freedreno.dylib
// beside it) on a thread of its own and returns nil, so no device is registered and the software renderer keeps running. Unlike it, it tests only in
// backboardd (the one process the kernel grants the open, see NativeSandboxGrantProcess).
// Published stage (T18: built with ASTEROIDS_METAL_PUBLISH and the metal2vulkan bridge, and the addon carries NativeAdrenoMetal.publish beside the
// bundle): backboardd returns the native Metal frontend's device (NativeBridgeMetalFrontend.mm, the metal2vulkan runtime on Turnip), which the
// original discovery registers, so the render server renders through it; a second thread serves the display driver's frames with the GPU (boot-arg
// sheng-fb-gpu=1, NativeMetalBridgePhase.h serveScanout). It follows the upstream published mode with less: backboardd only (no
// NativeAdrenoMetal.clients), and no telemetry stream, cache export, shader priming or render-server state experiment (that one writes fixed
// QuartzCore addresses of another build). Every other process returns nil and keeps the software renderer.
#import <Foundation/Foundation.h>
#include <dlfcn.h>
#include <pthread.h>
#include <stdlib.h>
#include <string.h>
#include "NativeAdrenoUserTest.h"
#ifdef NATIVE_TURNIP_REPLAY
#error "TurnipReplay.h is an Adreno 740 recording: nothing recorded on another GPU is replayed on the A810"
#endif
#ifdef ASTEROIDS_METAL_PUBLISH
#include <pwd.h>
#include <sys/stat.h>
#include <unistd.h>
// Its bridge self-test (bridgephase::run) is the upstream test stage's, unused here; only serveScanout is.
#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wunused-function"
#include "NativeMetalBridgePhase.h"
#include <mach/mach.h>
#pragma clang diagnostic pop
#endif

@interface _MTLDevice : NSObject
@end
@interface NativeAdrenoMetalDevice : _MTLDevice
- (id)initWithAcceleratorPort:(unsigned)port;
@end

#ifdef ASTEROIDS_METAL_PUBLISH
namespace publish {
// Whether a file of this name is readable beside the bundle (the union view of /System/Library/Extensions; the bundle's own seal stays untouched).
// T20: where backboardd's threads wait. Each thread that has a QuartzCore frame (the render server's), apart from this plugin's own two, as
// "<name>:" and up to eight frames "<image letter><offset in the image, hex>": the interrupted pc, the link register (the caller of a leaf
// such as a system call), then the frame-pointer chain. The thread is not stopped: a blocked thread's state is exact, a running one's is a
// sample. Memory is read with vm_read_overwrite, so a bad frame pointer ends the walk instead of faulting. Offsets go back to symbols
// offline (the unstripped plugin in the build directory; the shared cache's images for the rest).
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
static bool beside(const char *name) {
    Dl_info info = {};
    if (!dladdr(reinterpret_cast<const void *>(&beside), &info) || !info.dli_fname) return false;
    char path[1024];
    snprintf(path, sizeof path, "%s", info.dli_fname);   // <dir>/NativeAdrenoMetal.bundle/NativeAdrenoMetal
    for (int i = 0; i < 2; ++i) {
        char *slash = strrchr(path, '/');
        if (!slash) return false;
        *slash = 0;
    }
    strlcat(path, "/", sizeof path);
    strlcat(path, name, sizeof path);
    return access(path, R_OK) == 0;
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
static id gDevice;
// backboardd's one device. Markers as upstream: 6800 entered, 6801 Turnip found, 6810+i the cache directory that opened it, 6802 device made,
// 6803 returned to the discovery; 68f1 no Turnip, 68f2 no device (with "PUB no device: <reason>" on the stream).
static id deviceForPort(unsigned port) {
    static bool tried;
    @synchronized([NativeAdrenoMetalDevice class]) {
        if (tried) return gDevice;
        tried = true;
        // A8xx: Turnip renders in system memory and without concurrent binning (pandora's A840 needed both; T17 passed with them).
        setenv("TU_DEBUG", "sysmem,nocb", 1);
        // The discovery releases the accelerator port when it is done; the status and scanout threads keep using it.
        void *iokit = dlopen("/System/Library/Frameworks/IOKit.framework/IOKit", RTLD_NOW | RTLD_LOCAL);
        if (!iokit) return nil;
        if (auto retain = (int (*)(unsigned))dlsym(iokit, "IOObjectRetain")) retain(port);
        Gpu *gpu = makeGpu(port);
        if (!gpu) return nil;
        gpu->mark(0x6800);
        char turnipPath[1024];
        if (!turnip::libraryPath(turnipPath, sizeof turnipPath) || access(turnipPath, R_OK) != 0) {gpu->mark(0x68f1); return nil;}
        gpu->mark(0x6801);
        uint64_t registryID = 0;
        if (auto entryID = (int (*)(unsigned, uint64_t *))dlsym(iokit, "IORegistryEntryGetRegistryEntryID")) entryID(port, &registryID);
        // The render server compresses its framebuffer (an AGX layout this GPU does not write) unless told not to; the display takes plain rows.
        setenv("CA_DISABLE_FRAMEBUFFER_COMPRESSION", "1", 1);
        CFPreferencesSetAppValue(CFSTR("CAEnableFramebufferCompression"), kCFBooleanFalse, kCFPreferencesCurrentApplication);
        CFPreferencesSetAppValue(CFSTR("CAEnableIOSurfaceCompression"), kCFBooleanFalse, kCFPreferencesCurrentApplication);
        // The runtime needs a writable shader-cache directory; which ones backboardd's sandbox allows is not known here. In order: the user's
        // Library/Caches (kept across boots with sheng-data-persist=2), the user cache and temp directories, /tmp, /private/var/tmp.
        char candidates[5][512] = {};
        if (const passwd *user = getpwuid(getuid()); user && user->pw_dir && user->pw_dir[0]) {
            snprintf(candidates[0], sizeof candidates[0] - 16, "%s/Library/Caches/com.apple.backboardd", user->pw_dir);
            mkdir(candidates[0], 0700);
            strlcat(candidates[0], "/", sizeof candidates[0]);
        }
        if (!confstr(_CS_DARWIN_USER_CACHE_DIR, candidates[1], sizeof candidates[1] - 16)) candidates[1][0] = 0;
        if (!confstr(_CS_DARWIN_USER_TEMP_DIR, candidates[2], sizeof candidates[2] - 16)) candidates[2][0] = 0;
        strlcpy(candidates[3], "/tmp/", sizeof candidates[3]);
        strlcpy(candidates[4], "/private/var/tmp/", sizeof candidates[4]);
        for (unsigned i = 0; i < 5 && !gDevice; ++i) {
            if (!candidates[i][0]) continue;
            strlcat(candidates[i], "m2v", sizeof candidates[i]);
            mkdir(candidates[i], 0700);
            if (access(candidates[i], W_OK) != 0) continue;
            @try {
                gDevice = native_bridge_metal_open(turnipPath, candidates[i], registryID, nullptr);
            } @catch (NSException *exception) {
                gDevice = nil;
            }
            if (gDevice) gpu->mark(0x6810 + i);
        }
        if (!gDevice) {
            gpu->mark(0x68f2);
            char text[64];
            snprintf(text, sizeof text, "PUB no device: %.40s", nmb_last_error());
            streamText(*gpu, text);
            return nil;
        }
        gpu->mark(0x6802);
        pthread_t status, scanout;
        pthread_attr_t attributes;
        pthread_attr_init(&attributes);
        pthread_attr_setdetachstate(&attributes, PTHREAD_CREATE_DETACHED);
        pthread_attr_setstacksize(&attributes, 1u << 20);
        // Every ten seconds one line on the stream (the kernel's watcher prints it). One round: the formats asked for, the status (counters
        // and the last refusals) in up to four 57-character pieces, then the render server's stacks (sampleThreads, taken once per round) in
        // up to eight.
        pthread_create(&status, &attributes, [](void *arg) -> void * {
            Gpu *g = static_cast<Gpu *>(arg);
            gStatusThread = mach_thread_self();
            char full[420], stacks[460] = "";
            unsigned statusPieces = 1, stackPieces = 0;
            for (unsigned step = 0;; ++step) {
                sleep(10);
                char part[64], text[64];
                if (step >= 1 + statusPieces + stackPieces) step = 0;
                if (step == 0) {
                    native_bridge_metal_status(full, sizeof full);
                    sampleThreads(stacks, sizeof stacks);
                    statusPieces = (unsigned)((strlen(full) + 56) / 57);
                    statusPieces = statusPieces < 1 ? 1 : statusPieces > 4 ? 4 : statusPieces;
                    stackPieces = (unsigned)((strlen(stacks) + 56) / 57);
                    stackPieces = stackPieces > 8 ? 8 : stackPieces;
                    native_bridge_metal_formats(part, sizeof part);
                    snprintf(text, sizeof text, "PUB %.58s", part);
                } else if (step <= statusPieces) {
                    snprintf(text, sizeof text, "ST%u %.57s", step - 1, full + (size_t)(step - 1) * 57);
                } else {
                    const unsigned piece = step - 1 - statusPieces;
                    snprintf(text, sizeof text, "BT%u %.57s", piece, stacks + (size_t)piece * 57);
                }
                streamText(*g, text);
            }
            return nullptr;
        }, gpu);
        // The display hand-off: "P<step> e<hex>" then "PF ..." lines on GPU7 (the kernel's report line).
        pthread_create(&scanout, &attributes, [](void *arg) -> void * {
            Gpu *g = static_cast<Gpu *>(arg);
            gScanoutThread = mach_thread_self();
            sleep(3);
            @try {
                bridgephase::serveScanout(*g, gDevice);
            } @catch (NSException *exception) {
                g->mark(0x68f3);
            }
            return nullptr;
        }, gpu);
        pthread_attr_destroy(&attributes);
        gpu->mark(0x6803);
        return gDevice;
    }
}
}  // namespace publish
#endif

@implementation NativeAdrenoMetalDevice
- (id)initWithAcceleratorPort:(unsigned)port {
    self = [super init];
    if (!self || !port || strcmp(getprogname(), "backboardd") != 0) return nil;
#ifdef ASTEROIDS_METAL_PUBLISH
    if (publish::beside("NativeAdrenoMetal.publish")) return publish::deviceForPort(port);
#endif
    static bool attempted;
    if (attempted) return nil;
    attempted = true;
    // The discovery releases the accelerator port as soon as this returns nil, before the thread starts (UniXNU's plugin lost its first
    // marker that way): keep one extra reference, and let the thread look the service up itself as well.
    void *iokit = dlopen("/System/Library/Frameworks/IOKit.framework/IOKit", RTLD_NOW | RTLD_LOCAL);
    if (!iokit) return nil;
    if (auto retain = (int (*)(unsigned))dlsym(iokit, "IOObjectRetain")) retain(port);
    struct Job {unsigned port; void *iokit;};
    Job *job = new Job{port, iokit};
    pthread_t thread;
    pthread_attr_t attributes;
    pthread_attr_init(&attributes);
    pthread_attr_setdetachstate(&attributes, PTHREAD_CREATE_DETACHED);
    // On its own thread: a kernel call that never returns then stalls only this thread, not the render server.
    if (pthread_create(&thread, &attributes, [](void *arg) -> void * {
            Job *j = static_cast<Job *>(arg);
            unsigned port = j->port;
            auto matching = (void *(*)(const char *))dlsym(j->iokit, "IOServiceMatching");
            auto getService = (unsigned (*)(unsigned, void *))dlsym(j->iokit, "IOServiceGetMatchingService");
            if (matching && getService) {
                if (const unsigned own = getService(0, matching("IOAcceleratorES"))) port = own;
            }
            // A8xx: Turnip renders in system memory and without command-buffer reuse (pandora's A840 needed both). NativeTurnipSmoke.h sets
            // TU_DARWIN_SERVICE to IOAcceleratorES when unset, which is this accelerator.
            setenv("TU_DEBUG", "sysmem,nocb", 1);
            adrenoUserTest(port, "backboardd");
            delete j;
            return nullptr;
        }, job) != 0) delete job;
    pthread_attr_destroy(&attributes);
    return nil;
}
@end
