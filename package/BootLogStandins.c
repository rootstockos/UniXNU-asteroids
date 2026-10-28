// SPDX-License-Identifier: BSD-2-Clause-Patent
// asteroids: values and functions sheng's BootLog.c (the UFS diagnostic log, sheng-log=1) reads through weak references, for the probes
// and drivers this board does not build (sheng's audio, Wi-Fi, ADSP, battery, backlight and reset paths, the newer ShengUfs display
// window). A static kernel link needs a definition for each; these are all weak, so a real one wins, and they read as zero, empty or
// "not available" (-1, NULL), so the log prints those rows empty. Generated against BootLog.c from UniXNU-sheng 08cde20.
#include <stdint.h>
#include <stdbool.h>
#include "ShengAudioAbi.h"

#define WEAK __attribute__((weak))

// The record layouts BootLog.c declares for these arrays.
struct native_adreno_client_record {char name[20]; uint32_t opens, refused, closes, slot, peak_mib, last_refusal;};
struct native_adreno_asker_record {char name[20]; uint32_t reads;};

WEAK uint32_t cs_text_fixup_parsed;
WEAK uint32_t cs_text_fixup_applied;
WEAK volatile uint64_t native_data_persist_pages;
WEAK volatile uint64_t native_data_persist_errors;
WEAK volatile uint64_t native_data_persist_first_lba;
WEAK volatile int native_gpu_addon_files;
WEAK volatile int native_gpu_addon_written;
WEAK volatile int native_gpu_addon_same;
WEAK volatile int native_gpu_addon_failed;
WEAK volatile int native_gpu_addon_error;
WEAK volatile int native_gpu_addon_stage;
WEAK volatile int native_gpu_addon_status;
WEAK volatile int native_gpu_addon2_stage;
WEAK volatile int native_gpu_addon2_status;
WEAK volatile int native_gpu_addon3_stage;
WEAK volatile int native_gpu_addon4_stage;
WEAK volatile int native_gpu_addon4_status;
WEAK volatile int native_gpu_addon3_status;
WEAK volatile int native_cryptex_os_stage;
WEAK volatile int native_cryptex_os_status;
WEAK volatile int native_gestalt_cache_status;
WEAK volatile int native_ls_store_status;
WEAK volatile int native_staged_apps_stage;
WEAK volatile int32_t native_backlight_stats[16];
WEAK volatile int native_staged_apps_dirs;
WEAK volatile int native_staged_apps_files;
WEAK volatile int native_staged_apps_mib;
WEAK volatile int native_staged_apps_failed;
WEAK volatile int native_staged_apps_error;
WEAK char native_staged_apps_path[96];
WEAK volatile uint32_t native_adreno_stall[24];
WEAK volatile uint32_t native_adreno_stall_mem[59];
WEAK volatile uint32_t native_adreno_unstick[15];
WEAK volatile uint32_t native_adreno_fence_pending[8];
WEAK volatile uint32_t native_adreno_wptr_fence[4];
WEAK volatile int32_t native_touch_strokes[16][7];
WEAK volatile uint32_t native_touch_stroke_count;
WEAK volatile uint32_t native_wifi_stats[29];
WEAK volatile uint32_t native_wifi_dump[108];
WEAK volatile uint32_t native_wifi_pmic[472];
WEAK volatile uint32_t native_wifi_ep[16];
WEAK volatile uint32_t native_wifi_pon[12];
WEAK volatile uint32_t native_wifi_cfg[32];
WEAK volatile uint32_t native_wifi_chip[16];
WEAK volatile uint32_t native_wifi_ev[48];
WEAK volatile uint32_t native_wifi_ldo[24];
WEAK volatile uint32_t native_wifi_mhi[16];
WEAK volatile uint32_t native_wifi_mhi_ms[8];
WEAK char native_wifi_text[512][112];
WEAK volatile uint32_t native_wifi_text_n;
WEAK char native_wifi_note[128][112];
WEAK volatile uint32_t native_wifi_note_n;
WEAK volatile uint64_t nw_wifi_where[4];
WEAK volatile uint32_t native_dpu_stats[20];
WEAK volatile uint32_t native_touch_jitter[3];
WEAK volatile uint32_t native_touch_hold[14];
WEAK char native_lookup_log[160][112];
WEAK volatile unsigned native_lookup_count;
WEAK volatile uint32_t native_dpu_pipes[54];
WEAK volatile uint32_t native_dpu_more[40];
WEAK volatile uint32_t native_ufs_dpu_smmu[24];
WEAK volatile uint32_t native_ufs_dpu_check[8];
WEAK volatile uint32_t native_ufs_dpu_dump[20];
WEAK volatile int32_t native_adsp_stats[16];
WEAK volatile int32_t native_audio_stats[16];
WEAK volatile uint32_t native_audio_amps[6][4];
WEAK volatile int32_t native_adsp_audio[8];
WEAK volatile int32_t native_mic_stats[16];
WEAK volatile int32_t native_audio_service[8];
WEAK volatile uint64_t native_audio_clock[4];
WEAK volatile int32_t native_audio_sync[5];
WEAK volatile uint64_t native_audio_requests[17];
WEAK volatile int32_t native_audio_passed;
WEAK volatile uint32_t native_mic_regs[16];
WEAK volatile int32_t native_mic_capture[8];
WEAK volatile uint64_t native_audio_pa;
WEAK char native_adsp_text[96];
WEAK volatile int32_t native_battery_stats[16];
WEAK volatile int32_t native_qrtr_stats[16];
WEAK volatile int32_t native_battery_raw[10];
WEAK volatile int32_t native_charger_stats[16];
WEAK char native_qrtr_text[96];
WEAK volatile uint32_t native_swipe_stats[16];
WEAK volatile int32_t native_touch_extent[4];
WEAK volatile uint64_t native_adreno_vm_kind_us[5][3];
WEAK volatile uint32_t native_adreno_submit_ibs[4];
WEAK volatile uint32_t native_touch_edge_stats[4];
WEAK volatile uint32_t native_touch_hid_multi[3];
WEAK volatile int32_t native_backlight_request[6];
WEAK struct native_adreno_client_record native_adreno_client_log[8];
WEAK struct native_adreno_asker_record native_adreno_metal_askers[12];
WEAK char native_gpu_coherence_row[120];
WEAK volatile uint64_t native_gpu_idle_sleeps;
WEAK volatile uint64_t native_gpu_idle_timeouts;
WEAK volatile uint64_t native_gpu_idle_spins;
WEAK uint32_t native_gpu_perf_vote[2];
WEAK volatile uint32_t native_gpu_dcvs[15];
WEAK volatile uint32_t native_gpu_windows[8];

// Missed by the first generation: defined only in sources this board does not compile (NativeAdrenoService.cpp, sheng's Wi-Fi).
WEAK volatile uint64_t native_adreno_fence_phys;
WEAK struct {uint32_t stage, scans, frames, beacons, bss_seen; int last_err;} nw_wifi_stats;

WEAK uint64_t native_ufs_export_blocks(void) { return 0; }
WEAK int native_ufs_extra_fault(uint32_t *fault) { (void)fault; return 0; }
WEAK uint32_t native_ufs_dpu_live(unsigned reg) { (void)reg; return 0xdead; }
WEAK const ShengAudioDiag *sheng_audio_diag(void) { return 0; }
WEAK int boot_reset_probe(char *text, unsigned size) { (void)text; (void)size; return -1; }
WEAK int boot_reset_bootloader(char *text, unsigned size) { (void)text; (void)size; return -1; }
WEAK int boot_reset_mark_bootloader(char *text, unsigned size) { (void)text; (void)size; return -1; }
WEAK int boot_reset_watchdog(uint32_t seconds, char *text, unsigned size) { (void)seconds; (void)text; (void)size; return -1; }
