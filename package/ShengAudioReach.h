/* SPDX-License-Identifier: BSD-2-Clause-Patent */
/* The audio DSP's graph manager (APM) commands for the sheng speaker and microphone paths, as GPR packets. Two graphs per direction:
 * a device graph that owns the hardware endpoint and a stream graph that owns the shared memory endpoint; the stream graph's open
 * carries the connection into the device graph. Instance numbers follow the board topology (device subgraph 0x4005 for the speakers'
 * MI2S, 0x4007 for codec DMA TX3, stream subgraphs 0x4001 playback and 0x4003 capture). Each packet stays within one GLINK intent
 * (GPR_MAX_PACKET). Pure encoding: no I/O, no waits.
 * asteroids: from UniXNU-sheng package/ShengAudioReach.h. The speaker interface and data lines are inputs (sheng: secondary MI2S, SD1;
 * the 3a: primary MI2S), and the playback converter takes the device format, which may differ from the stream's (the 3a's amplifiers
 * take 32-bit slots, iOS writes 16-bit samples). */
#ifndef SHENG_AUDIOREACH_H
#define SHENG_AUDIOREACH_H
#include <stdint.h>
#ifdef __cplusplus
extern "C" {
#endif

#define AR_APM_IID 1u
#define AR_OP_GRAPH_OPEN 0x01001000u
#define AR_OP_GRAPH_PREPARE 0x01001001u
#define AR_OP_GRAPH_START 0x01001002u
#define AR_OP_GRAPH_STOP 0x01001003u
#define AR_OP_GRAPH_CLOSE 0x01001004u
#define AR_OP_SET_CFG 0x01001006u
#define AR_OP_MEM_MAP 0x0100100Cu
#define AR_OP_MEM_UNMAP 0x0100100Du
#define AR_OP_MEM_MAP_DONE 0x02001001u          /* payload {handle} */
#define AR_OP_WRITE 0x0400100Au                 /* DATA_CMD_WR_SH_MEM_EP_DATA_BUFFER_V2 */
#define AR_OP_WRITE_DONE 0x05001004u
#define AR_OP_READ 0x0400100Bu                  /* DATA_CMD_RD_SH_MEM_EP_DATA_BUFFER_V2 */
#define AR_OP_READ_DONE 0x05001005u
#define AR_OP_EOS_RENDERED 0x05001001u

/* Module ids and the instance ids the board topology gives them. */
#define AR_MOD_WR_SHMEM 0x07001000u
#define AR_MOD_RD_SHMEM 0x07001001u
#define AR_MOD_PCM_CNV 0x07001003u
#define AR_MOD_PCM_ENC 0x07001004u
#define AR_MOD_PCM_DEC 0x07001005u
#define AR_MOD_I2S_SINK 0x0700100Au
#define AR_MOD_CODEC_DMA_SOURCE 0x07001024u

#define AR_SG_PLAY 0x4001u
#define AR_SG_SPEAKER 0x4005u
#define AR_SG_CAPTURE 0x4003u
#define AR_SG_MIC 0x4007u
#define AR_IID_PLAY_SHMEM 0x6001u
#define AR_IID_PLAY_DEC 0x6002u
#define AR_IID_PLAY_CNV 0x6003u
#define AR_IID_SPEAKER_I2S 0x6051u
#define AR_IID_MIC_DMA 0x6070u
#define AR_IID_CAP_CNV 0x6022u
#define AR_IID_CAP_ENC 0x6023u
#define AR_IID_CAP_SHMEM 0x6024u

typedef struct {
    uint32_t rate;           /* Hz */
    uint16_t bits;           /* 16 or 32 */
    uint16_t channels;       /* 1 or 2 */
} ArFormat;

/* The speakers' I2S port: interface index (0 primary, 1 secondary, 2 tertiary, 3 quaternary MI2S), data line mask (bit 0 SD0, bit 1
 * SD1, ...). The SoC drives the bit clock and word select. */
typedef struct {
    uint32_t intf;
    uint16_t lines;
} ArI2s;
#define AR_I2S_PRIMARY 0u
#define AR_I2S_SECONDARY 1u
#define AR_I2S_SD0 1u
#define AR_I2S_SD1 2u

/* Each builder writes one complete GPR packet into out (capacity bytes) and returns its size, or 0 when it does not fit.
 * port: the GPR port the answers come back to; token: echoed in the answer. */
uint32_t ar_open_speaker_device(uint8_t *out, uint32_t capacity, uint32_t port, uint32_t token);
/* f: the format on the wire (the converter's output). */
uint32_t ar_config_speaker_device(uint8_t *out, uint32_t capacity, uint32_t port, uint32_t token, const ArFormat *f, const ArI2s *i2s);
uint32_t ar_open_playback_stream(uint8_t *out, uint32_t capacity, uint32_t port, uint32_t token);
/* f: the stream's format (the decoder's output); device: the speaker device's (the converter's output). */
uint32_t ar_config_playback_stream(uint8_t *out, uint32_t capacity, uint32_t port, uint32_t token, const ArFormat *f, const ArFormat *device);
uint32_t ar_config_playback_shmem(uint8_t *out, uint32_t capacity, uint32_t port, uint32_t token, const ArFormat *f);
uint32_t ar_open_mic_device(uint8_t *out, uint32_t capacity, uint32_t port, uint32_t token);
/* channel_mask: codec DMA channels in use (bit 0 first channel). */
uint32_t ar_config_mic_device(uint8_t *out, uint32_t capacity, uint32_t port, uint32_t token, const ArFormat *f, uint32_t channel_mask);
uint32_t ar_open_capture_stream(uint8_t *out, uint32_t capacity, uint32_t port, uint32_t token);
uint32_t ar_config_capture_stream(uint8_t *out, uint32_t capacity, uint32_t port, uint32_t token, const ArFormat *f);
/* opcode: AR_OP_GRAPH_PREPARE, _START, _STOP or _CLOSE for the one subgraph sg. */
uint32_t ar_graph_command(uint8_t *out, uint32_t capacity, uint32_t port, uint32_t token, uint32_t opcode, uint32_t sg);
/* Maps one physically contiguous region (rounded up to 4 KiB) for the DSP; the answer is AR_OP_MEM_MAP_DONE with the handle. */
uint32_t ar_map(uint8_t *out, uint32_t capacity, uint32_t port, uint32_t token, uint64_t address, uint32_t bytes);
uint32_t ar_unmap(uint8_t *out, uint32_t capacity, uint32_t port, uint32_t token, uint32_t handle);
/* One buffer for the playback shared memory endpoint; the answer is AR_OP_WRITE_DONE with the same token. */
uint32_t ar_write(uint8_t *out, uint32_t capacity, uint32_t port, uint32_t token, uint64_t address, uint32_t handle, uint32_t bytes);
/* One empty buffer for the capture shared memory endpoint; the answer is AR_OP_READ_DONE. */
uint32_t ar_read(uint8_t *out, uint32_t capacity, uint32_t port, uint32_t token, uint64_t address, uint32_t handle, uint32_t bytes);

/* The DSP's power and resource manager (PRM, port 2): an LPASS clock at hz (0 releases it; attribute "couple no", default root), and
 * a hardware block vote (1 LPASS core macros, 2 digital codec). The answer is AR_OP_PRM_REQUEST_DONE / _RELEASE_DONE. */
#define AR_PRM_IID 2u
#define AR_OP_PRM_REQUEST 0x0100100Fu
#define AR_OP_PRM_RELEASE 0x01001010u
#define AR_OP_PRM_REQUEST_DONE 0x02001002u
#define AR_OP_PRM_RELEASE_DONE 0x02001003u
#define AR_CLK_TX_CORE_MCLK 0x30cu   /* PRM clock ids, not the device tree indices (57 was refused) */
#define AR_CLK_VA_CORE_MCLK 0x307u
#define AR_HW_MACRO 1u
#define AR_HW_DCODEC 2u
uint32_t ar_prm_clock(uint8_t *out, uint32_t capacity, uint32_t port, uint32_t token, uint32_t clock_id, uint32_t hz);
uint32_t ar_prm_clock_attr(uint8_t *out, uint32_t capacity, uint32_t port, uint32_t token, uint32_t clock_id, uint32_t hz, uint32_t attribute);
uint32_t ar_prm_vote(uint8_t *out, uint32_t capacity, uint32_t port, uint32_t token, uint32_t block, int enable);

/* Answers. */
typedef struct {
    uint32_t opcode, token, src_port;
    uint32_t result_opcode, status;  /* basic result: the command and its status (0 success); framework state: 1 ready; PRM: status */
    uint32_t handle;                 /* AR_OP_MEM_MAP_DONE */
    uint64_t address;                /* write/read done: the buffer */
    uint32_t data_bytes, offset;     /* read done: bytes filled, starting offset bytes into the buffer */
} ArAnswer;
enum {ArUnknown = 0, ArBasicResult = 1, ArMapped = 2, ArWriteDone = 3, ArReadDone = 4, ArFrameworkState = 5, ArPrmDone = 6, ArBad = -1};
int ar_parse(const uint8_t *in, uint32_t length, ArAnswer *a);

#ifdef __cplusplus
}
#endif
#endif
