// SPDX-License-Identifier: MIT
// asteroids T76: the board's own classes registered into the kernel's kmod under the kext lock, in OSKext::load's lock order
// (patches/xnu/zzyg-board-class-registration.patch). OSMetaClass::preModLoad and postModLoad called on their own deadlocked
// against a kext loading on an IOKit config thread (T75f). kOSReturnNoMemory: preModLoad gave no handle.
#pragma once
#include <libkern/OSReturn.h>
extern "C" OSReturn OSKextRegisterBoardClasses(const char *kextIdentifier, void (*constructors)(void));
