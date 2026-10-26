// SPDX-License-Identifier: BSD-2-Clause-Patent
// asteroids (T37): the iOS brightness service for the 3a's AMOLED panel (asteroids-panel=3, started by PanelProbe.cpp). The service side
// is sheng's AppleARMBacklight.cpp (UniXNU-sheng 08cde20): CoreBrightness (backboardd) finds a service with backlight-control = true that
// conforms to AppleARMBacklight, reads min and max from IODisplayParameters brightness, and sets {brightness, FadePeriod (16.16 s)} through
// setProperties; a fade is stepped every 16 ms by a timer on the service's own work loop. The hardware write differs: sheng drives two
// KTZ8866 backlight chips over I2C, the 3a sends DCS 0x51 to the panel (PanelProbe.cpp's asteroids_panel_level). The 3a has no backlight
// to turn off, so level 0 turns the panel off (display off, sleep in) and the next level above 0 turns it on before writing the level.
// Kept from sheng, with its reasons: the class answers user space's conformance query for AppleARMBacklight (impersonate) instead of
// being called that, and NativeSandboxGrantProcess lets backboardd set its properties (UniXNU patches zzw/zzwb sandbox grants). Without
// the conformance, CoreBrightness's libEDR server never starts and QuartzCore's synchronous send to it blocks the display server (sheng int-236).
#include <IOKit/IOService.h>
#include <IOKit/IOLib.h>
#include <IOKit/IOWorkLoop.h>
#include <IOKit/IOTimerEventSource.h>
#include <IOKit/IOPlatformExpert.h>
#include <libkern/c++/OSNumber.h>
#include <libkern/c++/OSDictionary.h>
#include <pexpert/pexpert.h>
#include <kern/clock.h>
#include <mach/mach_time.h>

extern "C" {extern volatile int32_t native_panel_stats[16];}
extern "C" int asteroids_panel_level(uint32_t level);
extern "C" int asteroids_panel_power(bool on);

namespace {
constexpr uint32_t kMaxLevel = 4095, kStepMs = 16;
// The newest brightness asked for: goal << 16 | fade milliseconds, with bit 63 set while the work loop has not taken it.
volatile uint64_t gRequest;
}  // namespace

class AsteroidsBacklight : public IOService {
    OSDeclareDefaultStructors(AsteroidsBacklight)
    IOWorkLoop *loop = nullptr;
    IOTimerEventSource *fade = nullptr, *request = nullptr;
    uint32_t level = 0, from = 0, to = 0;
    uint64_t start = 0, period = 0;
    bool panelOn = true;

    static uint64_t now() {return mach_absolute_time();}

    // On the work loop: level 0 turns the panel off; any other level turns it on first when it is off.
    void setLevel(uint32_t value) {
        if (!value) {
            if (panelOn) {asteroids_panel_power(false); panelOn = false;}
            return;
        }
        if (!panelOn) {asteroids_panel_power(true); panelOn = true;}
        asteroids_panel_level(value);
    }
    static void fadeStep(OSObject *owner, IOTimerEventSource *sender) {
        AsteroidsBacklight *self = static_cast<AsteroidsBacklight *>(owner);   // see impersonate()
        if (!self) return;
        const uint64_t elapsed = now() - self->start;
        uint32_t next = self->to;
        if (elapsed < self->period)
            next = (uint32_t)((int64_t)self->from + ((int64_t)self->to - (int64_t)self->from) * (int64_t)elapsed / (int64_t)self->period);
        // A fade up from 0 starts at the panel's minimum (1 becomes 10) instead of leaving it off for its first step.
        if (!next && self->to) next = 1;
        if (next != self->level) {self->setLevel(next); self->level = next;}
        if (next != self->to) sender->setTimeoutMS(kStepMs);
    }
    void beginFade(uint32_t goal, uint32_t milliseconds) {
        fade->cancelTimeout();
        from = level;
        to = goal;
        start = now();
        nanoseconds_to_absolutetime((uint64_t)milliseconds * NSEC_PER_MSEC, &period);
        fadeStep(this, fade);
    }
    // A brightness request on the work loop (setProperties arms this timer at once): the newest one asked for, if not yet taken.
    static void requestFired(OSObject *owner, IOTimerEventSource *) {
        AsteroidsBacklight *self = static_cast<AsteroidsBacklight *>(owner);
        if (!self) return;
        const uint64_t taken = __atomic_exchange_n(&gRequest, 0ull, __ATOMIC_ACQUIRE);
        if (taken >> 63) self->beginFade((uint32_t)(taken >> 16) & 0xffff, (uint32_t)taken & 0xffff);
    }
    void publish(uint32_t value) {
        OSDictionary *brightness = OSDictionary::withCapacity(3), *parameters = OSDictionary::withCapacity(1);
        OSNumber *min = OSNumber::withNumber(0ULL, 32), *max = OSNumber::withNumber(kMaxLevel, 32), *current = OSNumber::withNumber(value, 32);
        if (brightness && parameters && min && max && current) {
            brightness->setObject("min", min); brightness->setObject("max", max); brightness->setObject("value", current);
            parameters->setObject("brightness", brightness);
            setProperty("IODisplayParameters", parameters);
        }
        OSSafeReleaseNULL(min); OSSafeReleaseNULL(max); OSSafeReleaseNULL(current); OSSafeReleaseNULL(brightness); OSSafeReleaseNULL(parameters);
    }
    // As sheng's: the vtable is copied and getMetaClass (slot 7) answers AppleARMBacklight's metaclass, so IOObjectConformsTo and the
    // sandbox's class check see AppleARMBacklight, while OSDynamicCast to IOService still works; the actions therefore cast statically.
    static inline const OSMetaClass *gAppleClass = nullptr;
    static const OSMetaClass *appleMetaClass(const OSMetaClassBase *) {return gAppleClass;}
    bool impersonate() {
        enum {kSlots = 320, kGetMetaClass = 7};
        const OSSymbol *name = OSSymbol::withCString("AppleARMBacklight");
        gAppleClass = name ? OSMetaClass::getMetaClassWithName(name) : nullptr;
        OSSafeReleaseNULL(name);
        if (!gAppleClass) return false;
        uintptr_t *copy = static_cast<uintptr_t *>(IOMalloc(kSlots * sizeof(uintptr_t)));
        if (!copy) return false;
        memcpy(copy, *reinterpret_cast<uintptr_t **>(this), kSlots * sizeof(uintptr_t));
        copy[kGetMetaClass] = reinterpret_cast<uintptr_t>(&appleMetaClass);
        __asm__ volatile("dmb ish" ::: "memory");
        *reinterpret_cast<uintptr_t **>(this) = copy;
        return true;
    }
public:
    bool startBacklight() {
        loop = IOWorkLoop::workLoop();
        fade = IOTimerEventSource::timerEventSource(this, &AsteroidsBacklight::fadeStep);
        request = IOTimerEventSource::timerEventSource(this, &AsteroidsBacklight::requestFired);
        if (!loop || !fade || !request) return false;
        if (loop->addEventSource(fade) || loop->addEventSource(request)) return false;
        // The level the boot loader left is not read back: the service starts at the middle, and CoreBrightness sets its own at once.
        level = to = kMaxLevel / 2;
        // sheng's switch: asteroids-backlight-control=0 publishes without backlight-control and the conformance.
        uint32_t control = 1;
        PE_parse_boot_argn("asteroids-backlight-control", &control, sizeof control);
        if (control == 1 && impersonate()) {setProperty("NativeSandboxGrantProcess", "backboardd"); setProperty("backlight-control", true);}
        publish(level);
        registerService();
        native_panel_stats[0] = control == 1 && gAppleClass ? 4 : 3;
        return true;
    }
    IOReturn setProperties(OSObject *properties) override {
        OSDictionary *dictionary = OSDynamicCast(OSDictionary, properties);
        if (!dictionary) return kIOReturnBadArgument;
        OSNumber *brightness = OSDynamicCast(OSNumber, dictionary->getObject("brightness"));
        if (!brightness) return kIOReturnSuccess;
        OSNumber *fadePeriod = OSDynamicCast(OSNumber, dictionary->getObject("FadePeriod"));
        const int32_t requested = (int32_t)brightness->unsigned32BitValue();
        const uint32_t goal = requested < 0 ? 0 : (uint32_t)requested > kMaxLevel ? kMaxLevel : (uint32_t)requested;
        uint64_t milliseconds = fadePeriod ? ((uint64_t)fadePeriod->unsigned32BitValue() * 1000) >> 16 : 0;
        if (milliseconds > 5000) milliseconds = 5000;
        native_panel_stats[13] = native_panel_stats[13] + 1;
        // Handed to the work loop without waiting for it (sheng int-277: a runAction here blocked backboardd's brightness thread).
        __atomic_store_n(&gRequest, 1ull << 63 | (uint64_t)goal << 16 | milliseconds, __ATOMIC_RELEASE);
        request->setTimeoutUS(1);
        publish(goal);
        return kIOReturnSuccess;
    }
};
OSDefineMetaClassAndStructors(AsteroidsBacklight, IOService)

// Called by PanelProbe.cpp once this file's classes are registered.
extern "C" bool asteroids_backlight_start(void) {
    AsteroidsBacklight *backlight = OSTypeAlloc(AsteroidsBacklight);
    if (!backlight || !backlight->init() || !backlight->attach(IOService::getPlatform()) || !backlight->startBacklight()) {
        if (backlight) backlight->release();
        return false;
    }
    backlight->release();
    return true;
}
