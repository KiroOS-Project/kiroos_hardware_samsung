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

/* the Kiro Speaker Engine, one per output stream (see kiro_spkprot.c) */
struct kiro_spk {
    unsigned int rate;
    int channels;
    int ready;
    int64_t lastProps;

    /* settings (properties, read again every second) */
    int on;
    float xmax;             /* bass band level the speaker takes before it cracks */
    float limit;            /* peak ceiling */
    float vbass;            /* virtual bass amount, 0..1.5 */
    float shelfDb;          /* upper bass lift where the speaker still plays */
    float loudDb;           /* loudness: makeup gain before the compressor */
    float compThr;          /* the compressor's threshold (linear) */
    int adapt;              /* follows the content and backs off on distortion */

    /* filters */
    kiro_bq_t hp[2];        /* high pass: what the speaker cannot play */
    kiro_bq_t lp[2];        /* the bass band, where the cone runs out of travel */
    kiro_bq_t shelf;        /* the upper bass lift */
    kiro_bq_t vbIn[4];      /* the low notes the harmonics are made from (40..220 Hz, steep:
                             * any voice left in it would mix with the bass in the harmonics) */
    kiro_bq_t vbOut[4];     /* the harmonics kept: where the speaker plays them (180..800 Hz) */
    kiro_bq_t mid[2];       /* the voice band, for the content analysis */
    float hpZ[2][2][2];     /* [stage][channel][state] */
    float lpZ[2][2][2];
    float shelfZ[2][2];
    float vbInZ[4][2];
    float vbOutZ[4][2];
    float midZ[2][2];

    /* dynamics */
    float vbEnv;            /* level of the low notes */
    float env;              /* level of the bass band */
    float bassGain;
    float compEnv;
    float compGain;
    float limGain;
    float holdTarget;       /* the lowest gain a peak still in the look-ahead needs */
    int holdLeft;
    /* the bass and loudness levels, held 25 ms so the gains do not follow single waves */
    int holdFrames;
    float bassHold, compHold;
    int bassHoldLeft, compHoldLeft;
    float envAtk, envRel, bassAtk, bassRel, limAtk, limRel, vbAtk, vbRel, compAtk, compRel;
    float delay[KIRO_SPK_LOOKAHEAD][2];
    int delayPos;

    /* adaptation */
    float speech;           /* 0 music .. 1 speech, slow */
    float drive;            /* backoff on distortion, linear gain <= 1 */
    float grAvg;            /* how hard the limiter works (dB, slow) */
    float bassAvg;          /* how hard the excursion limiter works (gain, slow) */
    float midBlock, allBlock;
    int blockFrames;
    float midHist[64];      /* voice band level per 31 ms block, the last 2 s */
    int midPos;
    unsigned clippedRecent;

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
