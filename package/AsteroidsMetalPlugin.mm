// SPDX-License-Identifier: MIT
// asteroids T17: the first, test-only stage of the Metal plugin. The original discovery in backboardd (IOServiceMatching("IOAcceleratorES")
// -> MetalPluginName "NativeAdrenoMetal" -> /System/Library/Extensions/NativeAdrenoMetal.bundle -> principal class -> initWithAcceleratorPort:)
// reaches this class through the GPU addon's Extensions overlay; GPU power mode 19 publishes the accelerator it reads. Like the default stage of
// UniXNU's NativeAdrenoMetalPlugin.mm, it runs the shared user test (NativeAdrenoUserTest.h: two rounds of three CP_MEM_WRITEs through its own
// mappings, then the Turnip smoke test with the libvulkan_freedreno.dylib beside it) on a thread of its own and returns nil, so no device is
// registered and the software renderer keeps running. Unlike it, it tests only in backboardd (the one process the kernel grants the open, see
// NativeSandboxGrantProcess) and has no metal2vulkan bridge phase; that comes with the published stage.
#import <Foundation/Foundation.h>
#include <dlfcn.h>
#include <pthread.h>
#include <stdlib.h>
#include <string.h>
#include "NativeAdrenoUserTest.h"
#ifdef NATIVE_TURNIP_REPLAY
#error "TurnipReplay.h is an Adreno 740 recording: nothing recorded on another GPU is replayed on the A810"
#endif

@interface _MTLDevice : NSObject
@end
@interface NativeAdrenoMetalDevice : _MTLDevice
- (id)initWithAcceleratorPort:(unsigned)port;
@end

@implementation NativeAdrenoMetalDevice
- (id)initWithAcceleratorPort:(unsigned)port {
    self = [super init];
    if (!self || !port || strcmp(getprogname(), "backboardd") != 0) return nil;
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
