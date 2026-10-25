// SPDX-License-Identifier: BSD-2-Clause-Patent
// asteroids: status words the boot progress kit (BootProgress.c) prints that come from drivers this board does not build.
// There is no Virtio block device and no owner-root audit (CONFIG_XNU_NATIVE_VIRTIO_BLOCK and CONFIG_XNU_OWNER_ROOT_AUDIT
// are off), so the page reads zero: the root is UFS and the audit did not run.
#include <stdint.h>

volatile uint32_t native_virtio_ready, native_virtio_read_requests, native_virtio_read_failures;
volatile uint64_t native_virtio_verified_blocks;
volatile int native_owner_root_accepted;
