/* SPDX-License-Identifier: BSD-2-Clause-Patent */
/* The 3a's TFA9873 speaker amplifiers: programs and their runner (AsteroidsTfa9873.h). The settings come from the 3a's own container
 * through the generated AsteroidsTfaConfig.h. */
#include "AsteroidsTfa9873.h"
#include "AsteroidsTfaConfig.h"

#define COUNT(a) (sizeof(a) / sizeof((a)[0]))

static unsigned put(TfaStep *out, unsigned n, uint8_t op, uint8_t reg, uint16_t mask, uint16_t value, uint16_t tries)
{
    if (n < TFA_MAX_STEPS) {out[n].op = op; out[n].reg = reg; out[n].mask = mask; out[n].value = value; out[n].tries = tries;}
    return n + 1;
}

/* A bitfield code as a read-modify-write of its register. */
static unsigned field(TfaStep *out, unsigned n, uint16_t code, uint16_t value)
{
    const unsigned first = (code >> 4) & 0xf, width = (code & 0xf) + 1;
    const uint16_t mask = (uint16_t)(((1u << width) - 1) << first);
    return put(out, n, TfaUpdate, (uint8_t)(code >> 8), mask, (uint16_t)((value << first) & mask), 0);
}

/* NXP's hidden settings for each TFA9873 revision, written behind the two keys before anything else (register, value). The N1B0
 * container of the 3a goes with revision 0x0b73. */
static const uint16_t kRevA73[][2] = {
    {0x02, 0x0628}, {0x4c, 0x00e9}, {0x52, 0x17d0}, {0x56, 0x0011}, {0x58, 0x0200}, {0x59, 0x0001}, {0x5f, 0x0180}, {0x61, 0x0183},
    {0x63, 0x055a}, {0x65, 0x0542}, {0x6f, 0x00a3}, {0x70, 0xa3fb}, {0x71, 0x007e}, {0x83, 0x009a}, {0x84, 0x0211}, {0x85, 0x0382},
    {0x8c, 0x0210}, {0xd5, 0x0000},
};
static const uint16_t kRevB73[][2] = {
    {0x02, 0x0628}, {0x61, 0x0183}, {0x63, 0x005a}, {0x6f, 0x0082}, {0x70, 0xa3eb}, {0x73, 0x0187}, {0x83, 0x071c}, {0x85, 0x0380},
    {0xd5, 0x004d},
};

unsigned tfa_identify_program(TfaStep *out)
{
    return put(out, 0, TfaRead, TFA_REG_REVISION, 0, 0, 0);
}

unsigned tfa_init_program(TfaStep *out, unsigned amp, uint16_t revision)
{
    unsigned n = 0;
    /* I2C reset: every register back to its default; the bit clears itself. */
    n = put(out, n, TfaWrite, TFA_REG_SYS_CONTROL0, 0, 1u << 1, 0);
    n = put(out, n, TfaDelayMs, 0, 0, 2, 0);
    /* Key 1 (0x0F = 0x5A6B, then 0xA0 = 0xFB ^ 0x5A), key 2 (MTPK = 0x5A behind 0x0F, which closes again), then the revision's
     * settings. */
    n = put(out, n, TfaWrite, 0x0f, 0, 0x5a6b, 0);
    n = put(out, n, TfaCopyXor, 0xfb, 0xa0, 0x005a, 0);
    n = put(out, n, TfaWrite, 0x0f, 0, 0x5a6b, 0);
    n = field(out, n, TFA_BF_MTPK, 0x5a);
    n = put(out, n, TfaWrite, 0x0f, 0, 0, 0);
    const uint16_t (*rev)[2] = revision == 0x0a73 ? kRevA73 : revision == 0x0b73 ? kRevB73 : 0;
    const unsigned revCount = revision == 0x0a73 ? COUNT(kRevA73) : revision == 0x0b73 ? COUNT(kRevB73) : 0;
    for (unsigned i = 0; i < revCount; ++i) n = put(out, n, TfaWrite, (uint8_t)rev[i][0], 0, rev[i][1], 0);
    n = field(out, n, TFA_BF_FSSYNCEN, 0);
    /* The container: this device's settings (TDM slots and size, sample rate, boost and DC protection), then the speaker profile. */
    const uint16_t (*dev)[2] = amp ? tfa_cfg_device_s : tfa_cfg_device_p;
    const unsigned devCount = amp ? COUNT(tfa_cfg_device_s) : COUNT(tfa_cfg_device_p);
    for (unsigned i = 0; i < devCount; ++i) n = field(out, n, dev[i][0], dev[i][1]);
    for (unsigned i = 0; i < COUNT(tfa_cfg_speaker); ++i) n = field(out, n, tfa_cfg_speaker[i][0], tfa_cfg_speaker[i][1]);
    return n;
}

unsigned tfa_power_up_program(TfaStep *out)
{
    unsigned n = 0;
    n = field(out, n, TFA_BF_MANSCONF, 1);
    n = field(out, n, TFA_BF_PWDN, 0);
    /* Clocks stable needs the I2S bit clock: up to 50 ms. */
    n = put(out, n, TfaPoll, TFA_REG_STATUS0, TFA_STATUS0_CLKS, TFA_STATUS0_CLKS, 50);
    n = field(out, n, TFA_BF_AMPE, 1);
    return n;
}

unsigned tfa_power_down_program(TfaStep *out)
{
    unsigned n = 0;
    n = field(out, n, TFA_BF_AMPE, 0);
    n = field(out, n, TFA_BF_PWDN, 1);
    for (unsigned i = 0; i < COUNT(tfa_cfg_standby); ++i) n = field(out, n, tfa_cfg_standby[i][0], tfa_cfg_standby[i][1]);
    return n;
}

unsigned tfa_status_program(TfaStep *out)
{
    unsigned n = 0;
    n = put(out, n, TfaRead, TFA_REG_STATUS0, 0, 0, 0);
    n = put(out, n, TfaRead, TFA_REG_STATUS1, 0, 0, 0);
    return n;
}

void tfa_run_start(TfaRun *r, const TfaStep *steps, unsigned count)
{
    r->steps = steps; r->count = count; r->index = 0; r->phase = 0; r->pending = 0; r->tries = 0; r->target = 0;
    r->read_count = 0; r->failed = 0;
    for (unsigned i = 0; i < 4; ++i) r->reads[i] = 0;
}

static void write16(TfaAction *a, uint8_t reg, uint16_t value)
{
    a->bytes[0] = reg; a->bytes[1] = (uint8_t)(value >> 8); a->bytes[2] = (uint8_t)value; a->length = 3;
}

int tfa_run_next(TfaRun *r, TfaAction *a)
{
    a->length = a->read_length = a->delay_ms = 0;
    if (r->index >= r->count) return TfaFinished;
    const TfaStep *s = &r->steps[r->index];
    if (r->phase == 4) {   /* the write half of a read-modify-write or a CopyXor */
        write16(a, r->target, r->pending);
        r->phase = 2;
        return TfaTransfer;
    }
    switch (s->op) {
    case TfaDelayMs:
        a->delay_ms = s->value;
        r->index++;
        return TfaWait;
    case TfaWrite:
        write16(a, s->reg, s->value);
        r->phase = 2;
        return TfaTransfer;
    case TfaUpdate:
    case TfaCopyXor:
    case TfaRead:
    case TfaPoll:
        if (s->op == TfaPoll && r->tries) {   /* a poll's next read comes after a 1 ms wait */
            if (r->phase != 6) {r->phase = 6; a->delay_ms = 1; return TfaWait;}
        }
        a->bytes[0] = s->reg; a->length = 1; a->read_length = 2;
        r->phase = s->op == TfaRead ? 3 : s->op == TfaPoll ? 5 : 1;
        return TfaTransfer;
    default:
        r->index = r->count;
        return TfaFinished;
    }
}

void tfa_run_result(TfaRun *r, uint16_t value)
{
    if (r->index >= r->count) return;
    const TfaStep *s = &r->steps[r->index];
    if (r->phase == 1) {
        if (s->op == TfaCopyXor) {
            r->target = (uint8_t)s->mask; r->pending = (uint16_t)(value ^ s->value); r->phase = 4;
            return;
        }
        /* The read half of a read-modify-write: the write half is next, unless the bits are already as wanted. */
        r->target = s->reg; r->pending = (uint16_t)((value & ~s->mask) | s->value);
        if (r->pending == value) {r->phase = 0; r->index++;} else r->phase = 4;
        return;
    }
    if (r->phase == 5) {
        r->tries++;
        if ((value & s->mask) == s->value || r->tries >= s->tries) {
            if ((value & s->mask) != s->value) r->failed = 1;
            r->tries = 0; r->phase = 0; r->index++;
        } else {
            r->phase = 0;   /* tfa_run_next waits 1 ms, then reads again */
        }
        return;
    }
    if (r->phase == 3 && r->read_count < 4) r->reads[r->read_count++] = value;
    r->phase = 0;
    r->index++;
}
