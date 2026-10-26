// SPDX-License-Identifier: BSD-2-Clause-Patent
// From xVanTuring/UniXNU-qemu package/TouchState.cpp, with the button sink added for asteroids. Nothing writes the touch
// state yet (no touch driver on the 3a so far); NativeHidInput then reports no contacts.
// The finger set NativeHidInput reports as its touch screen, published under a sequence lock: the producer makes
// native_touch_seq odd, writes count, identifiers and positions (x 0..30480, y 0..20320), then makes it even again.
// On sheng the touch panel probe (TouchProbe.cpp) defines and writes them; on this board VirtioInput.cpp writes them
// from QEMU's tablet, and UniXNU's live-hid.py mailbox through the debugger.
//
// NativeHidInput sends a report when told a frame is out, not on a timer: it registers its sink here
// (native_touch_set_sink), and the producer calls native_touch_published once it has made the sequence even.
#include <stdint.h>

extern "C" {
volatile uint32_t native_touch_seq, native_touch_count, native_touch_hid_reports;
volatile int native_touch_xs[12], native_touch_ys[12], native_touch_ids[12];
void native_touch_set_sink(void (*sink)(void *), void *context);
void native_touch_published(void);
}

namespace {
void (*volatile gSink)(void *);
void *volatile gSinkContext;
}

void native_touch_set_sink(void (*sink)(void *), void *context) {
    gSink = nullptr;
    __asm__ volatile("dmb ish" ::: "memory");
    gSinkContext = context;
    __asm__ volatile("dmb ish" ::: "memory");
    gSink = sink;
}

void native_touch_published(void) {
    void (*sink)(void *) = gSink;
    __asm__ volatile("dmb ish" ::: "memory");
    if (sink) sink(gSinkContext);
}

// T35: the button sink (native_buttons_set_sink) is ButtonProbe.cpp's, sheng's driver for the power and volume keys.
