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

/*
 * KiroOS speaker protection (Beta 4.1), on everything the primary output sends to the
 * loudspeaker: ringtones, alarms, notifications, music, games.
 *
 * A phone's micro speaker cracks for two reasons, and this recognises both as they happen:
 *  - the cone runs out of travel: low notes move it the most, and below its resonance it
 *    plays little sound for a lot of movement. The bass band (below ~700 Hz, where the travel
 *    is) is followed, and when it goes past what the speaker takes (persist.audio.kiro.spk_xmax,
 *    thousandths of full scale) that band alone is turned down, for as long as needed. The
 *    rest of the sound is untouched: out = in - (1 - gain) * bass, so with nothing to catch the
 *    output is exactly the input;
 *  - the amplifier clips: a peak limiter with 2 ms of look-ahead brings the peaks under the
 *    ceiling before they arrive (no hard edge, nothing to crackle).
 * What the speaker cannot play at all (below 200 Hz) is filtered out first: it only moved the
 * cone. Already clipped audio (full scale samples in a row) is counted for the log.
 * Off with: setprop persist.audio.kiro.spk 0 (read when a stream starts).
 */

#define LOG_TAG "kiro_spkprot"

#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#include <cutils/log.h>
#include <cutils/properties.h>

#include "kiro_spkprot.h"

#define HP_HZ 200.0f
#define BASS_HZ 700.0f
#define XMAX_DEFAULT "220"      /* 0.22 of full scale in the bass band */
#define LIMIT 0.84f             /* -1.5 dBFS */

static void butter(kiro_bq_t *q, float rate, float hz, int high)
{
    const float w0 = 2.0f * (float) M_PI * hz / rate;
    const float cw = cosf(w0);
    const float alpha = sinf(w0) / (2.0f * 0.70710678f);
    const float a0 = 1.0f + alpha;
    if (high) {
        q->b0 = (1.0f + cw) / 2.0f / a0;
        q->b1 = -(1.0f + cw) / a0;
    } else {
        q->b0 = (1.0f - cw) / 2.0f / a0;
        q->b1 = (1.0f - cw) / a0;
    }
    q->b2 = q->b0;
    q->a1 = -2.0f * cw / a0;
    q->a2 = (1.0f - alpha) / a0;
}

static inline float run(const kiro_bq_t *q, float *z, float v)
{
    const float y = q->b0 * v + z[0];
    z[0] = q->b1 * v - q->a1 * y + z[1];
    z[1] = q->b2 * v - q->a2 * y;
    return y;
}

static float coef(float seconds, float rate)
{
    return 1.0f - expf(-1.0f / (seconds * rate));
}

static int64_t nowMs(void)
{
    struct timespec t;
    clock_gettime(CLOCK_MONOTONIC, &t);
    return t.tv_sec * 1000LL + t.tv_nsec / 1000000;
}

static void setup(struct kiro_spk *s, int channels, unsigned int rate)
{
    char v[PROPERTY_VALUE_MAX];
    const float r = rate > 0 ? (float) rate : 48000.0f;
    s->rate = rate;
    s->channels = channels;
    property_get("persist.audio.kiro.spk", v, "1");
    s->ready = atoi(v) != 0 ? 1 : -1;
    property_get("persist.audio.kiro.spk_xmax", v, XMAX_DEFAULT);
    s->xmax = atoi(v) / 1000.0f;
    if (s->xmax < 0.05f || s->xmax > 1.0f) s->xmax = 0.22f;
    s->limit = LIMIT;
    butter(&s->hp[0], r, HP_HZ, 1);
    butter(&s->hp[1], r, HP_HZ, 1);
    butter(&s->lp[0], r, BASS_HZ, 0);
    butter(&s->lp[1], r, BASS_HZ, 0);
    /* bass: followed in 1 ms / 60 ms, turned down in 2 ms, back over 250 ms (not heard
     * pumping); limiter: 0.4 ms attack inside the 2 ms look-ahead, 80 ms release */
    s->envAtk = coef(0.001f, r);
    s->envRel = coef(0.060f, r);
    s->bassAtk = coef(0.002f, r);
    s->bassRel = coef(0.250f, r);
    s->limAtk = coef(0.0004f, r);
    s->limRel = coef(0.080f, r);
    /* a clean start (the stream is new, or it comes back from standby) */
    memset(s->hpZ, 0, sizeof(s->hpZ));
    memset(s->lpZ, 0, sizeof(s->lpZ));
    memset(s->delay, 0, sizeof(s->delay));
    s->delayPos = 0;
    s->env = 0;
    s->bassGain = 1.0f;
    s->limGain = 1.0f;
    s->worstBass = 1.0f;
    s->worstPeak = 1.0f;
    s->clipped = 0;
    ALOGD("speaker protection %s (bass limit %.2f, ceiling %.2f)",
            s->ready > 0 ? "on" : "off", s->xmax, s->limit);
}

void kiro_spk_reset(struct kiro_spk *s)
{
    s->ready = 0;       /* a clean start and the properties read again at the next write */
}

void kiro_spk_release(struct kiro_spk *s)
{
    free(s->buf);
    s->buf = NULL;
    s->bufSize = 0;
}

static void report(struct kiro_spk *s)
{
    const int64_t now = nowMs();
    if (now - s->lastLog < 5000) return;
    if (s->worstBass < 0.9f || s->worstPeak < 0.9f || s->clipped > 0) {
        ALOGI("speaker protection: bass down %.1f dB, peaks down %.1f dB, clipped input %u",
                -20.0f * log10f(s->worstBass), -20.0f * log10f(s->worstPeak), s->clipped);
    }
    s->worstBass = 1.0f;
    s->worstPeak = 1.0f;
    s->clipped = 0;
    s->lastLog = now;
}

const void *kiro_spk_process(struct kiro_spk *s, const void *in, size_t bytes, int channels,
        unsigned int rate)
{
    if (channels < 1 || channels > 2) return in;
    if (s->ready == 0 || s->rate != rate || s->channels != channels) {
        setup(s, channels, rate);
    }
    if (s->ready < 0) return in;
    if (s->bufSize < bytes) {
        int16_t *b = (int16_t *) realloc(s->buf, bytes);
        if (b == NULL) return in;
        s->buf = b;
        s->bufSize = bytes;
    }
    const int16_t *src = (const int16_t *) in;
    int16_t *dst = s->buf;
    const size_t frames = bytes / (sizeof(int16_t) * channels);
    unsigned run16 = 0;

    for (size_t i = 0; i < frames; i++) {
        float h[2], l[2], y[2];
        float mono = 0;
        for (int ch = 0; ch < channels; ch++) {
            const int16_t raw = src[i * channels + ch];
            if (raw >= 32767 || raw <= -32768) {
                if (++run16 == 3) s->clipped++;     /* three full-scale samples in a row */
            } else if (ch == 0) {
                run16 = 0;
            }
            /* the tiny offset keeps the filters out of denormals in silence */
            float v = raw * (1.0f / 32768.0f) + 1e-20f;
            v = run(&s->hp[0], s->hpZ[0][ch], v);
            v = run(&s->hp[1], s->hpZ[1][ch], v);
            h[ch] = v;
            float b = run(&s->lp[0], s->lpZ[0][ch], v);
            b = run(&s->lp[1], s->lpZ[1][ch], b);
            l[ch] = b;
            mono += b;
        }
        mono = fabsf(mono / channels);
        s->env += (mono - s->env) * (mono > s->env ? s->envAtk : s->envRel);
        const float wantB = s->env > s->xmax ? s->xmax / s->env : 1.0f;
        s->bassGain += (wantB - s->bassGain) * (wantB < s->bassGain ? s->bassAtk : s->bassRel);
        if (s->bassGain < s->worstBass) s->worstBass = s->bassGain;

        float peak = 0;
        for (int ch = 0; ch < channels; ch++) {
            y[ch] = h[ch] - (1.0f - s->bassGain) * l[ch];
            if (fabsf(y[ch]) > peak) peak = fabsf(y[ch]);
        }
        const float wantL = peak > s->limit ? s->limit / peak : 1.0f;
        s->limGain += (wantL - s->limGain) * (wantL < s->limGain ? s->limAtk : s->limRel);
        if (s->limGain < s->worstPeak) s->worstPeak = s->limGain;

        /* the gain was set on the sample entering the look-ahead, applied to the one leaving */
        for (int ch = 0; ch < channels; ch++) {
            const float d = s->delay[s->delayPos][ch];
            s->delay[s->delayPos][ch] = y[ch];
            float o = d * s->limGain;
            const float ao = fabsf(o);
            if (ao > s->limit) {
                /* soft knee: never past full scale, no hard edge */
                const float k = 1.0f - s->limit;
                o = copysignf(s->limit + k * tanhf((ao - s->limit) / k), o);
            }
            const long r = lrintf(o * 32767.0f);
            dst[i * channels + ch] = (int16_t) (r > 32767 ? 32767 : r < -32768 ? -32768 : r);
        }
        if (++s->delayPos >= KIRO_SPK_LOOKAHEAD) s->delayPos = 0;
    }
    report(s);
    return s->buf;
}
