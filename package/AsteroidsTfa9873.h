/* SPDX-License-Identifier: BSD-2-Clause-Patent */
/* The two NXP TFA9873 speaker amplifiers of the Nothing Phone (3a): "P" at I2C 0x34 and "S" at 0x35 on QUPv3 wrapper 1 SE6 (SE13,
 * 0x898000, pins 59/60), each with its own reset line (gpio141, gpio142; RST is active high: NXP's driver pulses it high, then holds it low). They take one I2S/TDM stream from the primary MI2S
 * (pins 15-18) as clock slaves, two 32-bit slots at 48 kHz: P plays slot 0, S slot 1. The TFA9873 has no DSP of its own here: the 3a's
 * container (tfa98xx.cnt) carries a dummy patch and register settings only, which scripts/prepare-asteroids-tfa.py turns into
 * build/asteroids-tfa/AsteroidsTfaConfig.h. Registers are 8-bit addresses with 16-bit values, most significant byte first on the wire.
 * The register map and start-up order follow NXP's TFA9873 description (its N1B0 field list and the TFA2 state sequence); the code is
 * this board's own.
 *
 * As ShengCs35l43.h does for sheng's amplifiers, a program is a list of steps; the runner turns each step into one bus transfer (a
 * read-modify-write takes two, a poll repeats its read) and never waits itself: the owner performs the transfer and feeds the result
 * back. */
#ifndef ASTEROIDS_TFA9873_H
#define ASTEROIDS_TFA9873_H
#include <stdint.h>
#ifdef __cplusplus
extern "C" {
#endif

#define TFA_AMPS 2
#define TFA_ADDRESS_P 0x34
#define TFA_ADDRESS_S 0x35
#define TFA_RESET_P 141
#define TFA_RESET_S 142

/* Registers and bitfields used directly (16-bit codes: register << 8 | first bit << 4 | width - 1). */
#define TFA_REG_SYS_CONTROL0 0x00
#define TFA_REG_REVISION 0x03
#define TFA_REG_STATUS0 0x10   /* VDDS, OTDS, OCDS, UVDS, MANALARM, CLKS (bit 6), MTPB, NOCLK, TDMERR */
#define TFA_REG_STATUS1 0x11   /* PLLS (bit 6), SWS (bit 13), AMPS (bit 14) */
#define TFA_BF_PWDN 0x0000
#define TFA_BF_I2CR 0x0010
#define TFA_BF_AMPE 0x0030
#define TFA_BF_MANSCONF 0x0120
#define TFA_BF_FSSYNCEN 0x0480
#define TFA_BF_MTPK 0xa107
#define TFA_STATUS0_CLKS (1u << 6)
#define TFA_STATUS0_ERRORS ((1u << 1) | (1u << 2) | (1u << 3) | (1u << 4))   /* OTDS, OCDS, UVDS, MANALARM (sticky ones clear on read) */
#define TFA_REVISIONS_KNOWN(r) (((r) & 0xff) == 0x73)

enum {TfaWrite = 1, TfaUpdate = 2, TfaRead = 3, TfaDelayMs = 4, TfaPoll = 5, TfaCopyXor = 6};
/* Write: reg = value. Update: reg = (reg & ~mask) | value. Read: keep the value (at most 4 per run). Poll: read until (reg & mask) ==
 * value, waiting 1 ms between reads, at most `tries` reads; a poll that runs out sets the run's failed flag and goes on. CopyXor: read
 * reg, write register `mask` with the value read ^ value (the first unlock key). */
typedef struct {uint8_t op, reg; uint16_t mask, value, tries;} TfaStep;

/* Programs, for amplifier 0 (P) or 1 (S). Each returns its step count and writes at most TFA_MAX_STEPS steps.
 * - identify: read the revision register (low byte 0x73 on a TFA9873).
 * - init: I2C reset, the revision's hidden settings (behind the two keys), frame sync off, the container's device bitfields, then the
 *   "speaker" profile's entry bitfields. Leaves the amplifier powered down.
 * - power up: configured (MANSCONF), out of power-down, wait for its clocks (the I2S bit clock must run), then the amplifier on.
 * - power down: amplifier off, then power-down, then the container's standby bitfields.
 * - status: read STATUS0 and STATUS1 (reads 0 and 1). */
#define TFA_MAX_STEPS 64
unsigned tfa_identify_program(TfaStep *out);
unsigned tfa_init_program(TfaStep *out, unsigned amp, uint16_t revision);
unsigned tfa_power_up_program(TfaStep *out);
unsigned tfa_power_down_program(TfaStep *out);
unsigned tfa_status_program(TfaStep *out);
static inline uint8_t tfa_address(unsigned amp) {return amp ? TFA_ADDRESS_S : TFA_ADDRESS_P;}
static inline unsigned tfa_reset_gpio(unsigned amp) {return amp ? TFA_RESET_S : TFA_RESET_P;}

/* The runner. */
typedef struct {
    const TfaStep *steps;
    unsigned count, index;
    uint8_t phase;                   /* 0 next step, 1 read of a read-modify-write (or of a CopyXor) in flight, 2 write in flight,
                                        3 read in flight, 4 the write of a read-modify-write is next, 5 poll read in flight */
    uint8_t target;                  /* the register phase 4 writes */
    uint16_t pending, tries;
    uint16_t reads[4];
    unsigned read_count;
    uint8_t failed;                  /* a poll ran out */
} TfaRun;
enum {TfaTransfer = 1, TfaWait = 2, TfaFinished = 3};
typedef struct {
    uint8_t bytes[3];
    uint32_t length;                 /* bytes to write */
    uint32_t read_length;            /* 0 or 2: then a read follows the write */
    uint32_t delay_ms;               /* TfaWait */
} TfaAction;
void tfa_run_start(TfaRun *r, const TfaStep *steps, unsigned count);
/* What to do next: TfaTransfer (perform a, then call tfa_run_result), TfaWait (wait a.delay_ms, then call tfa_run_next again) or
 * TfaFinished. */
int tfa_run_next(TfaRun *r, TfaAction *a);
/* The transfer finished; value: the 16-bit value read (when one was read), as the register value. */
void tfa_run_result(TfaRun *r, uint16_t value);
/* Two bytes read on the wire (the first in bits 7:0) as the register value. */
static inline uint16_t tfa_wire_value(uint32_t rx) {return (uint16_t)(((rx & 0xff) << 8) | ((rx >> 8) & 0xff));}

#ifdef __cplusplus
}
#endif
#endif
