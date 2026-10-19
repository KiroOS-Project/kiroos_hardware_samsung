/*
 * Copyright (C) 2026 KiroOS
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef KIRO_SPKPROT_H
#define KIRO_SPKPROT_H

#include <stddef.h>
#include <stdint.h>

#define KIRO_SPK_LOOKAHEAD 96   /* frames, 2 ms at 48 kHz */

typedef struct {
    float b0, b1, b2, a1, a2;
} kiro_bq_t;

/* KiroOS speaker protection, one per output stream (see kiro_spkprot.c) */
struct kiro_spk {
    unsigned int rate;
    int channels;
    int ready;
    float xmax;             /* bass band level the speaker takes before it cracks */
    float limit;            /* peak ceiling */
    kiro_bq_t hp[2];        /* high pass: what the speaker cannot play */
    kiro_bq_t lp[2];        /* the bass band, where the cone runs out of travel */
    float hpZ[2][2][2];     /* [stage][channel][state] */
    float lpZ[2][2][2];
    float env;              /* level of the bass band */
    float bassGain;
    float limGain;
    float envAtk, envRel, bassAtk, bassRel, limAtk, limRel;
    float delay[KIRO_SPK_LOOKAHEAD][2];
    int delayPos;
    /* what it caught, for the log */
    float worstBass, worstPeak;
    unsigned clipped;
    int64_t lastLog;
    int16_t *buf;
    size_t bufSize;
};

/* processes a 16-bit interleaved buffer for the speaker; returns the processed copy (the
 * input is never written: AudioFlinger's buffer is const) */
const void *kiro_spk_process(struct kiro_spk *s, const void *in, size_t bytes, int channels,
        unsigned int rate);
void kiro_spk_reset(struct kiro_spk *s);
void kiro_spk_release(struct kiro_spk *s);

#endif
