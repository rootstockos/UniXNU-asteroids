// SPDX-License-Identifier: MIT
// The 3a's first user-space GPU test (T16, sheng-gpu-power=18 with sheng-gpu-addon=1). launchd starts it from the GPU addon
// (/private/var/ngpu/bin, its jobs in the addon's union view over /System/Library/LaunchDaemons), outside every sandbox profile.
// It waits for the NativeAdrenoGpu service the board probe publishes, then runs UniXNU's shared user test
// (drivers/NativeMetal/NativeAdrenoUserTest.h): two rounds of three CP_MEM_WRITEs from an IB it writes through its own mappings,
// uncached then write-combined, and, when both rounds verify, the Turnip smoke test (NativeTurnipSmoke.h: Vulkan instance and
// device on the A810, a compute fill and a triangle) with libvulkan_freedreno.dylib from the same directory. Every step reaches
// the kernel as the driver's progress markers and SetMetadata lines, which the board probe prints.
//
// Differences from UniXNU's probe daemon (NativeAdrenoProbeDaemon.cpp): the service is matched by its class here
// (NativeAdrenoGpuPublished: the IOAcceleratorES-derived AGXAccelerator needs the original IOGPUFamily kext, which this boot
// does not load), Turnip is pointed at the same class, it waits ten minutes instead of two, it runs once per boot although three
// launchd jobs start it, and it does not export the system log. TurnipReplay.h (a submission recorded on the A740) is never on
// this build's include path, so nothing recorded on another GPU is replayed on the A810.
#include <fcntl.h>
#include <stdlib.h>
#include <sys/file.h>
#include <sys/sysctl.h>
#include <sys/time.h>
#include "NativeAdrenoUserTest.h"

#ifdef NATIVE_TURNIP_REPLAY
#error "TurnipReplay.h is an A740 recording and must not be built into the A810 test"
#endif

using MatchingFn = void *(*)(const char *);
using GetServiceFn = unsigned (*)(unsigned, void *);

// The jobs start together; the first takes the lock and the boot's time stamp, the others leave.
static bool firstThisBoot() {
    static const char kStamp[] = "/private/var/ngpu/asteroidsgpud.boot";
    const int lock = open("/private/var/ngpu/asteroidsgpud.lock", O_CREAT | O_RDWR, 0600);
    if (lock < 0 || flock(lock, LOCK_EX | LOCK_NB)) return false;
    struct timeval boot = {};
    size_t size = sizeof boot;
    if (sysctlbyname("kern.boottime", &boot, &size, nullptr, 0)) return false;
    char now[32], seen[32] = {};
    const int length = snprintf(now, sizeof now, "%ld\n", (long)boot.tv_sec);
    const int in = open(kStamp, O_RDONLY);
    if (in >= 0) {
        const ssize_t got = read(in, seen, sizeof seen - 1);
        close(in);
        if (got == length && !memcmp(seen, now, size_t(length))) return false;
    }
    const int out = open(kStamp, O_CREAT | O_TRUNC | O_WRONLY, 0600);
    if (out >= 0) {
        if (write(out, now, size_t(length)) != length) {}
        close(out);
    }
    return true;   // the lock stays held until exit
}

int main() {
    if (!firstThisBoot()) return 0;
    void *iokit = dlopen("/System/Library/Frameworks/IOKit.framework/IOKit", RTLD_NOW | RTLD_LOCAL);
    if (!iokit) return 1;
    auto matching = (MatchingFn)dlsym(iokit, "IOServiceMatching");
    auto getService = (GetServiceFn)dlsym(iokit, "IOServiceGetMatchingService");
    if (!matching || !getService) return 2;
    // NativeTurnipSmoke.h sets these only when unset.
    setenv("TU_DARWIN_SERVICE", "NativeAdrenoGpuPublished", 1);
    // The probe publishes about a minute after the kernel starts (power, GMU, zap, the kernel client's rounds); wait ten minutes.
    unsigned port = 0;
    for (unsigned i = 0; i < 1200 && !port; ++i) {
        port = getService(0, matching("NativeAdrenoGpuPublished"));
        if (!port) usleep(500000);
    }
    return !port ? 3 : adrenoUserTest(port, "asteroidsgpud") >= 0 ? 0 : 4;
}
