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
 * The Kiro Speaker Engine (Beta 5), on everything the primary output sends to the loudspeaker
 * (ringtones, alarms, notifications, music, games; not calls). In order:
 *
 *  1. Bass you can hear. A phone's speaker plays almost nothing under ~200 Hz, so the low
 *     notes are rebuilt from their harmonics (the "missing fundamental": hearing 2f and 3f, the
 *     ear puts the note at f). The 40..250 Hz band is followed, turned into its 2nd and 3rd
 *     harmonics (Chebyshev polynomials of the band divided by its own envelope, so the
 *     harmonics are clean at any level), kept where the speaker plays them (180..900 Hz) and
 *     added back. Then the upper bass the speaker can still play is lifted (a low shelf).
 *  2. What it cannot play at all (below 200 Hz) is filtered out: it only moved the cone.
 *  3. The cone's travel: the bass band (below ~700 Hz) is followed, and past what the speaker
 *     takes (persist.audio.kiro.spk_xmax) that band alone is turned down for as long as needed.
 *  4. Louder: a makeup gain, then a 3:1 compressor just under the ceiling, so the music is
 *     louder at the same peaks (the same gain at every volume step: the volume keeps working).
 *     The bass and loudness levels are held 25 ms, so no gain follows single bass waves.
 *  5. The amplifier never clips: a peak limiter with 2 ms of look-ahead and a soft knee.
 *
 * Adaptive ("persist.audio.kiro.adapt", on by default):
 *  - it tells speech from music (speech pauses between syllables and lives in the voice band;
 *    music is continuous): with speech (videos, podcasts, voice notes) the bass is held back so
 *    voices stay clear, with music it is all there;
 *  - it hears distortion coming: when the limiter has to work hard for a while (the speaker
 *    would crackle) or the source itself is clipped, the drive goes down by itself and comes
 *    back slowly; when the cone limiter works hard, less virtual bass is made (no pumping).
 *
 * Tuning, live (read again every second): persist.audio.kiro.spk 0/1, .vbass (0..150, % of
 * the harmonics), .shelf (tenths of dB), .loud (tenths of dB), .comp (compressor threshold,
 * tenths of dBFS), .spk_xmax (thousandths of full scale), .limit (thousandths of full scale),
 * .adapt 0/1.
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
#define SHELF_HZ 380.0f
#define COMP_RATIO 3.0f
/* measured offline (scratch stress test, bass max + "bass" preset): +8 dB of makeup and the
 * compressor just under the ceiling played a song 5.5 dB louder above 200 Hz, with the
 * intermodulation of a 60 Hz + 1 kHz test at -88 dB; a low threshold only made it quieter */
#define COMP_DEFAULT "-30"      /* threshold, tenths of dBFS */

#define XMAX_DEFAULT "220"
#define LIMIT_DEFAULT "880"     /* -1.1 dBFS */
#define VBASS_DEFAULT "70"
#define SHELF_DEFAULT "40"      /* +4.0 dB */
#define LOUD_DEFAULT "80"       /* +8.0 dB */

/* --- filters -------------------------------------------------------------------------------- */

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

/* RBJ low shelf, slope 1 */
static void lowShelf(kiro_bq_t *q, float rate, float hz, float db)
{
    const float A = powf(10.0f, db / 40.0f);
    const float w0 = 2.0f * (float) M_PI * hz / rate;
    const float cw = cosf(w0);
    const float alpha = sinf(w0) / 2.0f * sqrtf(2.0f);
    const float sa = 2.0f * sqrtf(A) * alpha;
    const float a0 = (A + 1) + (A - 1) * cw + sa;
    q->b0 = A * ((A + 1) - (A - 1) * cw + sa) / a0;
    q->b1 = 2 * A * ((A - 1) - (A + 1) * cw) / a0;
    q->b2 = A * ((A + 1) - (A - 1) * cw - sa) / a0;
    q->a1 = -2 * ((A - 1) + (A + 1) * cw) / a0;
    q->a2 = ((A + 1) + (A - 1) * cw - sa) / a0;
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

static int propInt(const char *name, const char *def)
{
    char v[PROPERTY_VALUE_MAX];
    property_get(name, v, def);
    return atoi(v);
}

static float clampf(float v, float lo, float hi)
{
    return v < lo ? lo : v > hi ? hi : v;
}

/* --- settings --------------------------------------------------------------------------------- */

static void readProps(struct kiro_spk *s)
{
    const float r = s->rate > 0 ? (float) s->rate : 48000.0f;
    const int wasOn = s->on;
    s->on = propInt("persist.audio.kiro.spk", "1") != 0;
    s->xmax = clampf(propInt("persist.audio.kiro.spk_xmax", XMAX_DEFAULT) / 1000.0f, 0.05f, 1.0f);
    s->limit = clampf(propInt("persist.audio.kiro.limit", LIMIT_DEFAULT) / 1000.0f, 0.5f, 0.97f);
    s->vbass = clampf(propInt("persist.audio.kiro.vbass", VBASS_DEFAULT) / 100.0f, 0.0f, 1.5f);
    s->loudDb = clampf(propInt("persist.audio.kiro.loud", LOUD_DEFAULT) / 10.0f, 0.0f, 12.0f);
    s->adapt = propInt("persist.audio.kiro.adapt", "1") != 0;
    s->compThr = powf(10.0f, clampf(propInt("persist.audio.kiro.comp", COMP_DEFAULT) / 10.0f,
            -30.0f, 0.0f) / 20.0f);
    const float shelf = clampf(propInt("persist.audio.kiro.shelf", SHELF_DEFAULT) / 10.0f,
            0.0f, 9.0f);
    if (shelf != s->shelfDb || s->lastProps == 0) {
        s->shelfDb = shelf;
        lowShelf(&s->shelf, r, SHELF_HZ, shelf);
    }
    if (s->on && !wasOn && s->lastProps != 0) {
        /* turned back on: a clean start, no burst from old filter state */
        memset(s->hpZ, 0, sizeof(s->hpZ));
        memset(s->lpZ, 0, sizeof(s->lpZ));
        memset(s->shelfZ, 0, sizeof(s->shelfZ));
        memset(s->vbInZ, 0, sizeof(s->vbInZ));
        memset(s->vbOutZ, 0, sizeof(s->vbOutZ));
        memset(s->delay, 0, sizeof(s->delay));
    }
}

static void setup(struct kiro_spk *s, int channels, unsigned int rate)
{
    const float r = rate > 0 ? (float) rate : 48000.0f;
    s->rate = rate;
    s->channels = channels;
    s->lastProps = 0;
    s->on = 0;
    readProps(s);
    s->lastProps = nowMs();
    butter(&s->hp[0], r, HP_HZ, 1);
    butter(&s->hp[1], r, HP_HZ, 1);
    butter(&s->lp[0], r, BASS_HZ, 0);
    butter(&s->lp[1], r, BASS_HZ, 0);
    /* the low notes: 40..250 Hz; the harmonics kept: 180..900 Hz */
    butter(&s->vbIn[0], r, 40.0f, 1);
    butter(&s->vbIn[1], r, 220.0f, 0);
    butter(&s->vbIn[2], r, 220.0f, 0);
    butter(&s->vbIn[3], r, 220.0f, 0);
    butter(&s->vbOut[0], r, 180.0f, 1);
    butter(&s->vbOut[1], r, 180.0f, 1);
    /* steep at the top: the higher harmonics of a low note are heard as a buzz, not bass */
    butter(&s->vbOut[2], r, 800.0f, 0);
    butter(&s->vbOut[3], r, 800.0f, 0);
    /* the voice band, for telling speech from music */
    butter(&s->mid[0], r, 300.0f, 1);
    butter(&s->mid[1], r, 3400.0f, 0);
    s->envAtk = coef(0.001f, r);
    s->envRel = coef(0.060f, r);
    s->bassAtk = coef(0.002f, r);
    s->bassRel = coef(0.250f, r);
    s->limAtk = coef(0.0004f, r);
    s->limRel = coef(0.080f, r);
    s->vbAtk = coef(0.002f, r);
    s->vbRel = coef(0.060f, r);
    s->compAtk = coef(0.008f, r);
    s->compRel = coef(0.250f, r);
    s->holdFrames = (int) (r * 0.025f);
    s->bassHold = s->compHold = 0;
    s->bassHoldLeft = s->compHoldLeft = 0;
    memset(s->hpZ, 0, sizeof(s->hpZ));
    memset(s->lpZ, 0, sizeof(s->lpZ));
    memset(s->shelfZ, 0, sizeof(s->shelfZ));
    memset(s->vbInZ, 0, sizeof(s->vbInZ));
    memset(s->vbOutZ, 0, sizeof(s->vbOutZ));
    memset(s->midZ, 0, sizeof(s->midZ));
    memset(s->delay, 0, sizeof(s->delay));
    memset(s->midHist, 0, sizeof(s->midHist));
    s->midPos = 0;
    s->delayPos = 0;
    s->vbEnv = 0;
    s->env = 0;
    s->bassGain = 1.0f;
    s->compEnv = 0;
    s->compGain = 1.0f;
    s->limGain = 1.0f;
    s->holdTarget = 1.0f;
    s->holdLeft = 0;
    s->speech = 0;
    s->drive = 1.0f;
    s->grAvg = 0;
    s->bassAvg = 1.0f;
    s->midBlock = s->allBlock = 0;
    s->blockFrames = 0;
    s->clippedRecent = 0;
    s->worstBass = 1.0f;
    s->worstPeak = 1.0f;
    s->clipped = 0;
    s->ready = 1;
    ALOGD("speaker engine %s: vbass %.2f, shelf %.1f dB, loud %.1f dB, xmax %.2f, ceiling %.2f, "
            "adaptive %d", s->on ? "on" : "off", s->vbass, s->shelfDb, s->loudDb, s->xmax,
            s->limit, s->adapt);
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

/* --- adaptation, once per ~31 ms block ------------------------------------------------------- */

#define HIST 64

static void endBlock(struct kiro_spk *s, int frames)
{
    const float mid = sqrtf(s->midBlock / frames);
    const float all = sqrtf(s->allBlock / frames);
    s->midBlock = s->allBlock = 0;
    s->midHist[s->midPos] = mid;
    s->midPos = (s->midPos + 1) % HIST;

    /* speech: the voice band dips between syllables (gaps), and holds most of the sound */
    float peak = 0;
    for (int i = 0; i < HIST; i++) if (s->midHist[i] > peak) peak = s->midHist[i];
    float target = 0;
    if (peak > 0.003f) {
        int gaps = 0;
        for (int i = 0; i < HIST; i++) if (s->midHist[i] < peak * 0.12f) gaps++;
        const float gapScore = clampf(((float) gaps / HIST - 0.06f) / 0.18f, 0.0f, 1.0f);
        const float voiceShare = all > 1e-5f ? mid / all : 0.0f;
        const float shareScore = clampf((voiceShare - 0.45f) / 0.3f, 0.0f, 1.0f);
        target = gapScore * (0.4f + 0.6f * shareScore);
    }
    s->speech += (target - s->speech) * 0.02f;     /* ~1.5 s to change its mind */

    /* distortion: how hard the limiter works, slowly */
    const float gr = -20.0f * log10f(s->limGain > 1e-4f ? s->limGain : 1e-4f);
    s->grAvg += (gr - s->grAvg) * 0.1f;
    s->bassAvg += (s->bassGain - s->bassAvg) * 0.05f;
    float backDb = 0;
    if (s->adapt) {
        backDb = -0.8f * (s->grAvg > 2.5f ? s->grAvg - 2.5f : 0.0f);
        if (s->clippedRecent > 3) backDb -= 2.0f;   /* the source itself is clipped */
        backDb = clampf(backDb, -8.0f, 0.0f);
    }
    const float want = powf(10.0f, backDb / 20.0f);
    /* down quickly (crackle is heard at once), back up slowly (not heard pumping) */
    s->drive += (want - s->drive) * (want < s->drive ? 0.25f : 0.012f);
    s->clippedRecent = s->clippedRecent > 0 ? s->clippedRecent - 1 : 0;
}

static void report(struct kiro_spk *s)
{
    const int64_t now = nowMs();
    if (now - s->lastLog < 5000) return;
    ALOGI("speaker engine: speech %.2f, drive %.1f dB, limiter %.1f dB (worst %.1f), bass down "
            "%.1f dB, clipped input %u", s->speech, 20.0f * log10f(s->drive), s->grAvg,
            -20.0f * log10f(s->worstPeak), -20.0f * log10f(s->worstBass), s->clipped);
    s->worstBass = 1.0f;
    s->worstPeak = 1.0f;
    s->clipped = 0;
    s->lastLog = now;
}

/* --- the engine ------------------------------------------------------------------------------- */

const void *kiro_spk_process(struct kiro_spk *s, const void *in, size_t bytes, int channels,
        unsigned int rate)
{
    if (channels < 1 || channels > 2) return in;
    if (s->ready == 0 || s->rate != rate || s->channels != channels) {
        setup(s, channels, rate);
    }
    const int64_t now = nowMs();
    if (now - s->lastProps >= 1000) {
        readProps(s);
        s->lastProps = now;
    }
    if (!s->on) return in;
    if (s->bufSize < bytes) {
        int16_t *b = (int16_t *) realloc(s->buf, bytes);
        if (b == NULL) return in;
        s->buf = b;
        s->bufSize = bytes;
    }
    const int16_t *src = (const int16_t *) in;
    int16_t *dst = s->buf;
    const size_t frames = bytes / (sizeof(int16_t) * channels);
    const int block = (int) (s->rate * 0.031f);
    const float makeup = powf(10.0f, s->loudDb / 20.0f);
    unsigned run16 = 0;

    for (size_t i = 0; i < frames; i++) {
        float x[2], y[2];
        float mono = 0;
        for (int ch = 0; ch < channels; ch++) {
            const int16_t raw = src[i * channels + ch];
            if (raw >= 32767 || raw <= -32768) {
                if (++run16 == 3) {                 /* three full-scale samples in a row */
                    s->clipped++;
                    s->clippedRecent++;
                }
            } else if (ch == 0) {
                run16 = 0;
            }
            /* the tiny offset keeps the filters out of denormals in silence */
            x[ch] = raw * (1.0f / 32768.0f) + 1e-20f;
            mono += x[ch];
        }
        mono /= channels;

        /* content: the voice band against everything */
        float m = run(&s->mid[0], s->midZ[0], mono);
        m = run(&s->mid[1], s->midZ[1], m);
        s->midBlock += m * m;
        s->allBlock += mono * mono;
        if (++s->blockFrames >= block) {
            endBlock(s, s->blockFrames);
            s->blockFrames = 0;
        }
        /* with speech the bass is held back, and less of it when the cone limiter is busy */
        const float bassAmount = (1.0f - 0.65f * s->speech)
                * (s->adapt ? clampf(s->bassAvg * 1.3f, 0.4f, 1.0f) : 1.0f);

        /* 1. virtual bass: the 2nd and 3rd harmonics of the low notes */
        float low = run(&s->vbIn[0], s->vbInZ[0], mono);
        low = run(&s->vbIn[1], s->vbInZ[1], low);
        low = run(&s->vbIn[2], s->vbInZ[2], low);
        low = run(&s->vbIn[3], s->vbInZ[3], low);
        const float al = fabsf(low);
        s->vbEnv += (al - s->vbEnv) * (al > s->vbEnv ? s->vbAtk : s->vbRel);
        float harm = 0;
        if (s->vbEnv > 1e-4f && s->vbass > 0) {
            /* a little margin over the envelope: the clamp (sharp corners, a buzz) is
             * reached only when the envelope lags a sudden note */
            const float n = clampf(low / (s->vbEnv * 1.08f), -1.0f, 1.0f);
            const float t2 = 2.0f * n * n - 1.0f;          /* 2nd harmonic */
            const float t3 = n * (4.0f * n * n - 3.0f);   /* 3rd harmonic */
            harm = (0.6f * t2 + 0.4f * t3) * s->vbEnv;
            harm = run(&s->vbOut[0], s->vbOutZ[0], harm);
            harm = run(&s->vbOut[1], s->vbOutZ[1], harm);
            harm = run(&s->vbOut[2], s->vbOutZ[2], harm);
            harm = run(&s->vbOut[3], s->vbOutZ[3], harm);
            harm *= s->vbass * bassAmount;
        }

        float bass = 0;
        for (int ch = 0; ch < channels; ch++) {
            /* 2. what the speaker cannot play goes; the harmonics come in */
            float v = run(&s->hp[0], s->hpZ[0][ch], x[ch]);
            v = run(&s->hp[1], s->hpZ[1][ch], v);
            v += harm;
            /* the upper bass lift, as much as the content wants */
            const float sh = run(&s->shelf, s->shelfZ[ch], v);
            v += (sh - v) * bassAmount;
            y[ch] = v;
            float b = run(&s->lp[0], s->lpZ[0][ch], v);
            b = run(&s->lp[1], s->lpZ[1][ch], b);
            x[ch] = b;      /* reused: the bass band of this channel */
            bass += b;
        }

        /* 3. the cone's travel. The level is held for 25 ms (longer than a 50 Hz wave): a
         * follower that rose and fell inside every wave turned the gain up and down with it,
         * which is distortion of its own */
        bass = fabsf(bass / channels);
        if (bass >= s->bassHold || s->bassHoldLeft <= 0) {
            s->bassHold = bass;
            s->bassHoldLeft = s->holdFrames;
        } else {
            s->bassHoldLeft--;
        }
        bass = s->bassHold;
        s->env += (bass - s->env) * (bass > s->env ? s->envAtk : s->envRel);
        const float wantB = s->env > s->xmax ? s->xmax / s->env : 1.0f;
        s->bassGain += (wantB - s->bassGain) * (wantB < s->bassGain ? s->bassAtk : s->bassRel);
        if (s->bassGain < s->worstBass) s->worstBass = s->bassGain;

        /* 4. louder: makeup (backed off on distortion), then 3:1 over the threshold */
        const float g = makeup * s->drive;
        float level = 0;
        for (int ch = 0; ch < channels; ch++) {
            y[ch] = (y[ch] - (1.0f - s->bassGain) * x[ch]) * g;
            if (fabsf(y[ch]) > level) level = fabsf(y[ch]);
        }
        /* the same 25 ms hold: the gain follows the music's loudness, not each wave of it */
        if (level >= s->compHold || s->compHoldLeft <= 0) {
            s->compHold = level;
            s->compHoldLeft = s->holdFrames;
        } else {
            s->compHoldLeft--;
        }
        level = s->compHold;
        s->compEnv += (level - s->compEnv) * (level > s->compEnv ? s->compAtk : s->compRel);
        const float wantC = s->compEnv > s->compThr
                ? powf(s->compThr / s->compEnv, 1.0f - 1.0f / COMP_RATIO) : 1.0f;
        s->compGain += (wantC - s->compGain) * (wantC < s->compGain ? s->compAtk : s->compRel);

        /* 5. the peak limiter with look-ahead */
        float peak = 0;
        for (int ch = 0; ch < channels; ch++) {
            y[ch] *= s->compGain;
            if (fabsf(y[ch]) > peak) peak = fabsf(y[ch]);
        }
        /* the gain a peak needs is held for the whole look-ahead, so it is reached by the time
         * that peak leaves (chased only while the peak was there, it was barely touched) */
        const float wantNow = peak > s->limit ? s->limit / peak : 1.0f;
        if (wantNow <= s->holdTarget || s->holdLeft <= 0) {
            if (wantNow <= s->holdTarget) s->holdLeft = KIRO_SPK_LOOKAHEAD;
            s->holdTarget = wantNow;
        } else {
            s->holdLeft--;
        }
        const float wantL = s->holdTarget;
        s->limGain += (wantL - s->limGain) * (wantL < s->limGain ? s->limAtk : s->limRel);
        if (s->limGain < s->worstPeak) s->worstPeak = s->limGain;

        /* the gain was set on the sample entering the look-ahead, applied to the one leaving */
        for (int ch = 0; ch < channels; ch++) {
            const float d = s->delay[s->delayPos][ch];
            s->delay[s->delayPos][ch] = y[ch];
            float o = d * s->limGain;
            const float ao = fabsf(o);
#ifdef KIRO_SPK_TEST
            if (ao > s->limit * 1.2f) {
                fprintf(stderr, "over: d %.3f gain %.3f wantL %.3f peakIn %.3f\n", d, s->limGain,
                        wantL, peak);
            }
#endif
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
