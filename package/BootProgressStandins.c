// SPDX-License-Identifier: BSD-2-Clause-Patent
// asteroids: values the boot progress kit (BootProgress.c) reads that come from drivers and probes this board does not
// build, all weak so a real definition wins. There is no Virtio block device and no owner-root audit
// (CONFIG_XNU_NATIVE_VIRTIO_BLOCK and CONFIG_XNU_OWNER_ROOT_AUDIT are off), NativeUfs is built without the verified-block
// check (no verified-block counters), and sheng's Adreno 740, touch, CPU frequency and UFS log probes are not built. The page
// reads zero or empty rows for them.
#include <stdint.h>

#define WEAK __attribute__((weak))

WEAK volatile uint32_t native_virtio_ready, native_virtio_read_requests, native_virtio_read_failures;
WEAK volatile uint64_t native_virtio_verified_blocks;
WEAK volatile int native_owner_root_accepted;
WEAK volatile uint64_t native_ufs_verified_blocks;
WEAK volatile uint32_t native_ufs_integrity_failures;
WEAK volatile uint64_t native_adreno_cache_stats[5];
WEAK volatile uint32_t native_adreno_coherence[8];
WEAK volatile uint32_t native_adreno_wc_distrusted;
WEAK volatile uint32_t native_adreno_wc_audit[6];
WEAK volatile uint64_t native_adreno_reassert_ticks;
WEAK volatile uint64_t native_adreno_audit_ticks;
WEAK volatile uint32_t native_adreno_audit_attr;
WEAK char native_cpufreq_row[120];
WEAK volatile uint32_t native_cpufreq_stage;
WEAK char native_touch_rows[4][120];
WEAK volatile unsigned native_touch_stage_req;

WEAK void sheng_cpufreq_poll(void) {}
// T39: the GPU row (GPUG) reads GpuPowerProbe.cpp's interrupt and clock counters.
WEAK uint32_t native_gpu_irq[6];
WEAK volatile uint32_t native_gpu_irq_count, native_gpu_mhz;
// T39: the RTC row reads ArmDTPlatform's, QcomPmicRtc's and the kernel's offset-file counters.
WEAK volatile int32_t armdt_platform_rtc_stats[5], qcom_pmic_rtc_stats[7];
WEAK volatile int32_t native_rtc_offset_status[3] = {-1};
