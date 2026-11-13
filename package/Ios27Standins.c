/* SPDX-License-Identifier: BSD-2-Clause-Patent
 *
 * iOS 27 (build-native-legacy-display-27.py only): definitions the board code and UniXNU's newer drivers reference, which the
 * iPadOS 17 build gets from layers this build does not have. Each is the "absent" answer its caller already handles.
 */
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/* LegacyDisplayProbe's caller stacks come from BootProgress's fault probe (BootFaultProbe.c, iPadOS 17); no frames here. */
__attribute__((weak, visibility("hidden"))) unsigned
boot_progress_caller_stack (uint64_t *out, unsigned limit, uint64_t *slide)
{
  (void)out;
  (void)limit;
  if (slide != NULL)
    *slide = 0;
  return 0;
}

/* UniXNU RamBlock releases the packed Data seed's loader memory through payload-addon-release.patch (macOS profile only), and
 * only with boot-payload-release=1: false keeps the memory, as a kernel without the patch does. */
__attribute__((weak, visibility("hidden"))) bool
ml_static_mfree_prepare (uintptr_t address, size_t size)
{
  (void)address;
  (void)size;
  return false;
}

/* The GPU addon's exec substitute hook (UniXNU patches/xnu/zzy-native-gpu-addon-mount.patch) exists in the iPadOS 17 patch
 * only; BootWatchdogPolicy.c sets it for gpu-addon-swap=1. Here the pointer has no reader, so that test switch does nothing. */
const char *(*native_gpu_addon_exec_substitute) (const char *path) = NULL;

/* UniXNU RamBlock writes Data through a cached store write when the UFS driver has one (sheng's ShengUfs); our NativeUfs fork
 * writes through, which is the stronger guarantee, so the cached write is the plain one. */
extern int native_ufs_store_write (uint64_t rel, uint32_t blocks, const void *src);

__attribute__((weak, visibility("hidden"))) int
native_ufs_store_write_cached (uint64_t rel, uint32_t blocks, const void *src)
{
  return native_ufs_store_write (rel, blocks, src);
}
