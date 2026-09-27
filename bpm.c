#ifndef _WIN32
#define _POSIX_C_SOURCE 200809L
#endif
/*
 * Standalone BPM and leading-silence analyser.
 * Build: Windows gcc -O2 bpm.c -lm -lwinmm -lshell32 -o bpm.exe
 *        macOS/Linux cc -O2 bpm.c -lm -o bpm
 * Input: PCM WAV (8/16/24/32-bit integer or 32-bit IEEE float).
 */
#define _CRT_SECURE_NO_WARNINGS
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <math.h>
#ifdef _WIN32
#include <windows.h>
#include <mmsystem.h>
#include <shellapi.h>
#else
#include <unistd.h>
#include <sys/wait.h>
#include <sys/ioctl.h>
#include <sys/select.h>
#include <termios.h>
#include <signal.h>
#include <time.h>
#include <fcntl.h>
#include <errno.h>
#endif

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

typedef struct { double re, im; } Complex;
typedef struct {
    unsigned char *data;
    size_t size;
    unsigned channels, rate, bits, format;
} Wave;

/* Extracted from the three sample-rate coefficient banks in LibCpp.dll. */
static const double BIQUAD[3][16][5] = {
  { /* 48 kHz */
    {2,1,-1.9357148371211979,.94170045160372695,.001496403620632246}, {2,1,-1.8590762659582099,.86482489876726276,.0014371582022632194},
    {0,-1,-1.9731491828203316,.97953721714347519,.038683376541251063}, {0,-1,-1.9383006635720235,.96137281475624847,.038683376541251063},
    {0,-1,-1.9305470566393397,.93953993813607839,.037909869457216396}, {0,-1,-1.9047367208661594,.92047699481829581,.037909869457216396},
    {0,-1,-1.9341140031258925,.95936683579188464,.076211068056843939}, {0,-1,-1.834546897615803,.92460252591545578,.076211068056843939},
    {0,-1,-1.8474800548242554,.88234057507713604,.07333821798839002}, {0,-1,-1.786708494362891,.84711889379273642,.07333821798839002},
    {-2,1,-1.7009643319435259,.78849973981529797,.87236601793970592}, {-2,1,-1.4796742169311932,.55582154328248878,.75887394005342046},
    {0,0,-1.9522250000000001,.95230941015625004,.0441}, {2,1,-1.9688774973857579,.97039660175711517,.00037977609283935493},
    {2,1,-1.9285084850826344,.9299964423952546,.0003719893281551018}, {0,0,-1.4,.48,.2}
  },
  { /* 32 kHz */
    {2,1,-1.9006465638071275,.91391293369153381,.0033165924711015399}, {2,1,-1.791587696777784,.80409284398316283,.0031262868013446823},
    {0,-1,-1.9551331294901764,.96942359724192628,.057589864308760577}, {0,-1,-1.8914384351956604,.94273848531403681,.057589864308760577},
    {0,-1,-1.8906481524757157,.91056795618768371,.0559132077540236}, {0,-1,-1.8483764376432956,.88306736308808476,.0559132077540236},
    {0,-1,-1.8832665479670578,.93936089151864399,.11261473189959464}, {0,-1,-1.6928128227129573,.88994695879396346,.11261473189959464},
    {0,-1,-1.75188517392731,.82786888860462982,.10660246744674093}, {0,-1,-1.6489134150085212,.77932148942151369,.10660246744674093},
    {-2,1,-1.5182418440638745,.7039626566672621,.80555112518278416}, {-2,1,-1.2554404734849929,.40901378318031245,.66611356416632628},
    {0,0,-1.9283375,.92852742285156253,.06615}, {2,1,-1.9525426196393316,.95593497333644439,.00084808842427817996},
    {2,1,-1.8935423413365597,.8968321877643215,.0008224616069403773}, {0,0,-1.4,.48,.2}
  },
  { /* 44.1 kHz */
    {2,1,-1.9296472648815026,.93671950987931574,.0017680612494532478}, {2,1,-1.8470012302151446,.8537705737366641,.0016923358803798848},
    {0,-1,-1.9701832899735581,.9777435661450361,.042048320411797346}, {0,-1,-1.9307644878934767,.95804211074743828,.042048320411797346},
    {0,-1,-1.923734068386191,.93435845766892844,.041138010165536872}, {0,-1,-1.8951712655794619,.9137505704894946,.041138010165536872},
    {0,-1,-1.925968651733853,.9558199793811436,.082730627558081263}, {0,-1,-1.8121187013381126,.91831227931931725,.082730627558081263},
    {0,-1,-1.8314525533284556,.87252152856112386,.079370925545494644}, {0,-1,-1.7636927184274693,.83473849906751341,.079370925545494644},
    {-2,1,-1.6699250371362808,.77254617806529502,.86061780380039399}, {-2,1,-1.438556103531466,.52695903501152652,.74137878463574813},
    {0,0,-1.948,.94810000000000005,.048}, {2,1,-1.9660249635383409,.96782223970722425,.00044931904222082926},
    {2,1,-1.9222869522443087,.9240442445437952,.0004393230748716104}, {0,0,-1.4,.48,.2}
  }
};

typedef struct {
    double start, end;
} MeasureRange;

static uint16_t u16le(const unsigned char *p) { return (uint16_t)(p[0] | ((uint16_t)p[1] << 8)); }
static uint32_t u32le(const unsigned char *p) { return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24); }

#ifdef _WIN32
static wchar_t *wide_from_utf8(const char *text) {
    int length = MultiByteToWideChar(CP_UTF8, MB_ERR_INVALID_CHARS, text, -1, NULL, 0);
    wchar_t *wide;
    if (!length) return NULL;
    wide = (wchar_t *)malloc((size_t)length * sizeof(wchar_t));
    if (!wide || !MultiByteToWideChar(CP_UTF8, MB_ERR_INVALID_CHARS,
                                      text, -1, wide, length)) {
        free(wide);
        return NULL;
    }
    return wide;
}

static char *utf8_from_wide(const wchar_t *wide) {
    int length = WideCharToMultiByte(CP_UTF8, WC_ERR_INVALID_CHARS,
                                     wide, -1, NULL, 0, NULL, NULL);
    char *text;
    if (!length) return NULL;
    text = (char *)malloc((size_t)length);
    if (!text || !WideCharToMultiByte(CP_UTF8, WC_ERR_INVALID_CHARS,
                                      wide, -1, text, length, NULL, NULL)) {
        free(text);
        return NULL;
    }
    return text;
}
#endif

static int read_wave(const char *path, Wave *w) {
    FILE *f;
#ifdef _WIN32
    wchar_t *wide_path = wide_from_utf8(path);
    f = wide_path ? _wfopen(wide_path, L"rb") : NULL;
    free(wide_path);
#else
    f = fopen(path, "rb");
#endif
    unsigned char head[12], ch[8];
    int have_fmt = 0, have_data = 0;
    memset(w, 0, sizeof(*w));
    if (!f) return 0;
    if (fread(head, 1, 12, f) != 12 || memcmp(head, "RIFF", 4) || memcmp(head + 8, "WAVE", 4)) {
        fclose(f); return 0;
    }
    while (fread(ch, 1, 8, f) == 8) {
        uint32_t n = u32le(ch + 4);
        long start = ftell(f);
        if (!memcmp(ch, "fmt ", 4) && n >= 16) {
            unsigned char fmt[40]; size_t take = n < sizeof(fmt) ? n : sizeof(fmt);
            if (fread(fmt, 1, take, f) != take) break;
            w->format = u16le(fmt); w->channels = u16le(fmt + 2); w->rate = u32le(fmt + 4); w->bits = u16le(fmt + 14);
            if (w->format == 0xfffe && take >= 40) w->format = u16le(fmt + 24);
            have_fmt = 1;
        } else if (!memcmp(ch, "data", 4)) {
            w->data = (unsigned char *)malloc(n ? n : 1);
            if (!w->data || fread(w->data, 1, n, f) != n) { free(w->data); w->data = NULL; break; }
            w->size = n; have_data = 1;
        }
        if (fseek(f, start + (long)n + (n & 1u), SEEK_SET) != 0) break;
    }
    fclose(f);
    if (!have_fmt || !have_data || !w->channels || !w->rate || (w->format != 1 && w->format != 3)) {
        free(w->data); w->data = NULL; return 0;
    }
    if ((w->format == 3 && w->bits != 32) || (w->format == 1 && w->bits != 8 && w->bits != 16 && w->bits != 24 && w->bits != 32)) {
        free(w->data); w->data = NULL; return 0;
    }
    return 1;
}

#ifdef _WIN32
static int append_command_char(wchar_t *command, size_t capacity,
                               size_t *length, wchar_t value) {
    if (*length + 1 >= capacity) return 0;
    command[(*length)++] = value;
    command[*length] = L'\0';
    return 1;
}

/* CreateProcessW expects the executable's usual Windows argument quoting. */
static int append_quoted_argument(wchar_t *command, size_t capacity,
                                  size_t *length, const wchar_t *argument) {
    if (!append_command_char(command, capacity, length, L'"')) return 0;
    while (*argument) {
        size_t slashes = 0, copies;
        while (*argument == L'\\') { ++slashes; ++argument; }
        copies = *argument == L'"' || !*argument ? slashes * 2 : slashes;
        if (*argument == L'"') ++copies;
        while (copies--)
            if (!append_command_char(command, capacity, length, L'\\')) return 0;
        if (!*argument) break;
        if (!append_command_char(command, capacity, length, *argument++)) return 0;
    }
    return append_command_char(command, capacity, length, L'"') &&
           append_command_char(command, capacity, length, L' ');
}
#endif

/* Ask ffmpeg to decode anything this small WAV reader cannot handle. */
static int ffmpeg_decode(const char *input, char *output, size_t output_cap) {
#ifdef _WIN32
    wchar_t temp_dir[MAX_PATH], temp_path[MAX_PATH], command[32768];
    wchar_t *wide_input = wide_from_utf8(input);
    char *utf8_temp = NULL;
    STARTUPINFOW si;
    PROCESS_INFORMATION pi;
    size_t pos = 0, i;
    DWORD n;
    const wchar_t *args[] = {
        L"ffmpeg", L"-nostdin", L"-v", L"error", L"-y", L"-i", wide_input,
        L"-ac", L"2", L"-ar", L"44100", L"-c:a", L"pcm_s16le",
        L"-f", L"wav", temp_path
    };
    output[0] = '\0';
    if (!wide_input) return 0;
    n = GetTempPathW(MAX_PATH, temp_dir);
    if (!n || n >= MAX_PATH || !GetTempFileNameW(temp_dir, L"bpm", 0, temp_path)) {
        free(wide_input);
        return 0;
    }
    utf8_temp = utf8_from_wide(temp_path);
    if (!utf8_temp || strlen(utf8_temp) + 1 > output_cap) goto fail_win;
    strcpy(output, utf8_temp);
    command[0] = L'\0';
    for (i = 0; i < sizeof(args) / sizeof(args[0]); ++i)
        if (!append_quoted_argument(command, sizeof(command) / sizeof(command[0]),
                                    &pos, args[i])) goto fail_win;
    memset(&si, 0, sizeof(si)); memset(&pi, 0, sizeof(pi)); si.cb = sizeof(si); si.dwFlags = STARTF_USESHOWWINDOW; si.wShowWindow = SW_HIDE;
    if (!CreateProcessW(NULL, command, NULL, NULL, FALSE, CREATE_NO_WINDOW,
                        NULL, NULL, &si, &pi)) goto fail_win;
    WaitForSingleObject(pi.hProcess, INFINITE);
    {
        DWORD code = 1;
        GetExitCodeProcess(pi.hProcess, &code);
        CloseHandle(pi.hThread);
        CloseHandle(pi.hProcess);
        if (code == 0) { free(wide_input); free(utf8_temp); return 1; }
    }
fail_win:
    DeleteFileW(temp_path);
    output[0] = '\0';
    free(wide_input);
    free(utf8_temp);
    return 0;
#else
    char template[] = "/tmp/bpm-audio-XXXXXX";
    int fd = mkstemp(template), status;
    pid_t child;
    if (fd < 0) return 0;
    close(fd); unlink(template);
    if (strlen(template) + 1 > output_cap) return 0;
    strcpy(output, template);
    child = fork();
    if (child == 0) {
        execlp("ffmpeg", "ffmpeg", "-nostdin", "-v", "error", "-y", "-i", input,
               "-ac", "2", "-ar", "44100", "-c:a", "pcm_s16le", "-f", "wav", output, (char *)NULL);
        _exit(127);
    }
    if (child < 0) { unlink(output); return 0; }
    if (waitpid(child, &status, 0) < 0 || !WIFEXITED(status) || WEXITSTATUS(status) != 0) { unlink(output); return 0; }
    return 1;
#endif
}

static void remove_temp(const char *path) {
#ifdef _WIN32
    if (path && *path) {
        wchar_t *wide_path = wide_from_utf8(path);
        if (wide_path) DeleteFileW(wide_path);
        free(wide_path);
    }
#else
    if (path && *path) unlink(path);
#endif
}

static void print_help(const char *program) {
    printf("Usage: %s [--tui] [--offset|-offset|-o|o] [audio-file]\n", program);
    printf("Estimate BPM from WAV audio; unsupported formats are decoded with ffmpeg when available.\n");
    printf("Options:\n  --offset, -offset, -o, o  Also print leading-silence offset\n  --h, --help, -h           Show this help\n");
    printf("  --tui                     Open interactive waveform range analyser\n");
#ifndef _WIN32
#ifdef __APPLE__
    printf("TUI playback uses afplay (or ffplay/aplay if available).\n");
#else
    printf("TUI playback uses ffplay or aplay when available.\n");
#endif
#endif
}

static double sample_at(const unsigned char *p, unsigned format, unsigned bits) {
    if (format == 3) { float f; memcpy(&f, p, 4); return isfinite(f) ? f : 0.0; }
    if (bits == 8) return ((int)p[0] - 128) / 128.0;
    if (bits == 16) return (int16_t)u16le(p) / 32768.0;
    if (bits == 24) { int32_t v = (int32_t)(p[0] | (p[1] << 8) | (p[2] << 16)); if (v & 0x800000) v |= (int32_t)0xff000000; return v / 8388608.0; }
    return (int32_t)u32le(p) / 2147483648.0;
}

static void fft(Complex *a, size_t n, int inverse) {
    size_t i, j, len;
    for (i = 1, j = 0; i < n; ++i) {
        size_t bit = n >> 1;
        for (; j & bit; bit >>= 1) j ^= bit;
        j ^= bit;
        if (i < j) { Complex t = a[i]; a[i] = a[j]; a[j] = t; }
    }
    for (len = 2; len <= n; len <<= 1) {
        double angle = (inverse ? 2.0 : -2.0) * M_PI / (double)len;
        Complex root = { cos(angle), sin(angle) };
        size_t base;
        for (base = 0; base < n; base += len) {
            Complex w = { 1.0, 0.0 };
            for (j = 0; j < len / 2; ++j) {
                Complex u = a[base + j], v = a[base + j + len / 2];
                double vr = v.re * w.re - v.im * w.im, vi = v.re * w.im + v.im * w.re;
                a[base + j].re = u.re + vr; a[base + j].im = u.im + vi;
                a[base + j + len / 2].re = u.re - vr; a[base + j + len / 2].im = u.im - vi;
                { double nr = w.re * root.re - w.im * root.im; w.im = w.re * root.im + w.im * root.re; w.re = nr; }
            }
        }
        if (len > n / 2) break;
    }
    if (inverse) for (i = 0; i < n; ++i) { a[i].re /= n; a[i].im /= n; }
}

static int coefficient_bank(unsigned rate) {
    if (rate == 48000) return 0;
    if (rate == 32000) return 1;
    if (rate == 44100) return 2;
    return -1;
}

static void run_biquads(float *x, size_t n, int bank, unsigned first, unsigned count) {
    unsigned stage;
    for (stage = 0; stage < count; ++stage) {
        const double *coefficient = BIQUAD[bank][first + stage];
        /* Stored denominator coefficients have their sign bit complemented. */
        float b0 = (float)coefficient[0], b1 = (float)coefficient[1];
        float a1 = -(float)coefficient[2], a2 = -(float)coefficient[3];
        float b2 = (float)coefficient[4], state1 = 0.0f, state2 = 0.0f;
        size_t i;
        for (i = 0; i < n; ++i) {
            float v = x[i];
            float state = state1 * a1 + b2 * v + state2 * a2;
            x[i] = state1 * b0 + state + state2 * b1;
            state2 = state1;
            state1 = state;
        }
    }
}

static unsigned float_histogram_bin(float value) {
    union { float f; uint32_t u; } bits;
    bits.f = fabsf(value);
    return (unsigned)((bits.u + 0x40000u) >> 19);
}

/* Decode once, retaining the 1 ms RMS values only until offset is known. */
static float *decode_mono(const Wave *wave, size_t frames, size_t milliseconds,
                          double *offset_ms) {
    const unsigned bytes_per_sample = wave->bits / 8;
    float *mono = (float *)malloc(frames * sizeof(float));
    double *rms = (double *)calloc(milliseconds, sizeof(double));
    double peak = 0.0;
    size_t frame, ms;
    if (!mono || !rms) { free(mono); free(rms); return NULL; }

    for (frame = 0; frame < frames; ++frame) {
        double value = 0.0;
        unsigned channel;
        for (channel = 0; channel < wave->channels; ++channel) {
            size_t index = (frame * wave->channels + channel) * bytes_per_sample;
            value += sample_at(wave->data + index, wave->format, wave->bits);
        }
        mono[frame] = (float)(value / wave->channels);
        ms = frame * 1000u / wave->rate;
        if (ms < milliseconds) rms[ms] += (double)mono[frame] * mono[frame];
    }
    for (ms = 0; ms < milliseconds; ++ms) {
        size_t first = ms * wave->rate / 1000u;
        size_t last = (ms + 1) * wave->rate / 1000u;
        size_t count = last > first ? last - first : 1;
        rms[ms] = sqrt(rms[ms] / count);
        if (rms[ms] > peak) peak = rms[ms];
    }
    if (peak <= 1e-9) { free(mono); free(rms); return NULL; }
    *offset_ms = 0.0;
    for (ms = 0; ms < milliseconds; ++ms) {
        if (rms[ms] > peak * 0.02) { *offset_ms = (double)ms; break; }
    }
    free(rms);
    return mono;
}

/* Port of the four native frequency bands and their median thresholds. */
static void accumulate_band_scores(const float *mono, float *band, float *score,
                                   size_t frames, int bank) {
    static const unsigned starts[4] = {0, 2, 6, 10};
    static const unsigned counts[4] = {2, 4, 4, 2};
    static const int shifts[4] = {-320, -64, -32, 0};
    static const float weights[4] = {0.3f, 0.2f, 0.2f, 0.3f};
    unsigned frequency_band;

    for (frequency_band = 0; frequency_band < 4; ++frequency_band) {
        uint32_t histogram[4097] = {0};
        unsigned median;
        double cumulative = 0.0;
        float threshold, scale, weight;
        uint32_t raw;
        size_t frame, advance = shifts[frequency_band] < 0 ?
                                (size_t)-shifts[frequency_band] : 0;

        memcpy(band, mono, frames * sizeof(float));
        run_biquads(band, frames, bank, starts[frequency_band], counts[frequency_band]);
        for (frame = 0; frame < frames; ++frame) band[frame] *= band[frame];
        if (frequency_band == 0) run_biquads(band, frames, bank, 12, 1);
        for (frame = 0; frame < frames; ++frame) {
            unsigned bin = float_histogram_bin(band[frame]);
            if (bin > 4096) bin = 4096;
            ++histogram[bin];
        }
        for (median = 0; median < 4096 && cumulative < (double)(frames >> 1); ++median)
            cumulative += histogram[median];

        raw = median << 19;
        memcpy(&threshold, &raw, sizeof(threshold));
        if (!(threshold > 0.0f)) threshold = 1e-30f;
        scale = 2.0f / threshold;
        weight = weights[frequency_band] * 8.262958317573066e-8f;
        for (frame = 0; frame < frames; ++frame) {
            size_t source = frame + advance;
            float quantized;
            if (source >= frames) continue;
            quantized = (float)((int)(band[source] * scale + 1.0f) - 1);
            score[frame] += quantized * weight;
        }
    }
}

static int smooth_envelope(double *envelope, size_t milliseconds, int bank) {
    float *forward = (float *)malloc(milliseconds * sizeof(float));
    float *reverse = (float *)malloc(milliseconds * sizeof(float));
    double mean = 0.0;
    size_t ms;
    if (!forward || !reverse) { free(forward); free(reverse); return 0; }
    for (ms = 0; ms < milliseconds; ++ms) {
        forward[ms] = (float)envelope[ms];
        reverse[ms] = (float)envelope[milliseconds - 1 - ms];
    }
    run_biquads(forward, milliseconds, bank, 15, 1);
    run_biquads(reverse, milliseconds, bank, 15, 1);
    envelope[milliseconds - 1] = -forward[milliseconds - 2];
    for (ms = 1; ms < milliseconds - 1; ++ms)
        envelope[ms] = (double)reverse[milliseconds - 2 - ms] - forward[ms - 1];
    envelope[0] = reverse[milliseconds - 2];
    free(forward);
    free(reverse);

    for (ms = 0; ms < milliseconds; ++ms) mean += envelope[ms];
    mean /= milliseconds;
    for (ms = 0; ms < milliseconds; ++ms) envelope[ms] -= mean;
    return 1;
}

/* Native signal path: frequency bands, 1 kHz sampling, then smoothing. */
static double *make_envelope(const Wave *wave, size_t *out_n, double *offset_ms) {
    const unsigned bytes_per_frame = (wave->bits / 8) * wave->channels;
    const size_t frames = wave->size / bytes_per_frame;
    const size_t milliseconds = (frames * 1000u + wave->rate - 1u) / wave->rate;
    const int bank = coefficient_bank(wave->rate);
    float *mono, *band, *score;
    double *envelope;
    size_t ms;
    if (bank < 0 || frames < 2000 || milliseconds < 2000) return NULL;

    mono = decode_mono(wave, frames, milliseconds, offset_ms);
    if (!mono) return NULL;
    band = (float *)malloc(frames * sizeof(float));
    score = (float *)calloc(frames, sizeof(float));
    envelope = (double *)calloc(milliseconds, sizeof(double));
    if (!band || !score || !envelope) {
        free(mono); free(band); free(score); free(envelope);
        return NULL;
    }

    accumulate_band_scores(mono, band, score, frames, bank);
    free(mono);
    free(band);
    run_biquads(score, frames, bank, 13, 2);
    /* The native code picks the nearest source sample at each millisecond. */
    for (ms = 0; ms < milliseconds; ++ms) {
        size_t source = (size_t)llround((double)ms * wave->rate / 1000.0);
        if (source < frames) envelope[ms] = score[source];
    }
    free(score);
    if (!smooth_envelope(envelope, milliseconds, bank)) { free(envelope); return NULL; }
    *out_n = milliseconds;
    return envelope;
}

static size_t local_envelope_peak(const double *envelope, size_t n, size_t center) {
    size_t first = center > 10 ? center - 10 : 0;
    size_t last = center + 10 < n ? center + 10 : n - 1;
    size_t index, best = first;
    for (index = first + 1; index <= last; ++index)
        if (envelope[index] > envelope[best]) best = index;
    return best;
}

static double fit_peak_period_once(const double *envelope, size_t n, double initial_period,
                                   double *residual_ms, double *relative_period_error) {
    size_t period = (size_t)llround(initial_period);
    size_t phase, best_phase = 0, beat, count = 0;
    double best_score = -1.0;
    double sx = 0.0, sy = 0.0, sxx = 0.0, sxy = 0.0, syy = 0.0;
    double centered_xx, centered_xy, centered_yy, slope, sse, sigma, slope_error;

    if (period < 180 || period > 2000) {
        if (residual_ms) *residual_ms = 1e9;
        if (relative_period_error) *relative_period_error = 1.0;
        return initial_period;
    }
    /* Align the beat grid to maxima within +/-10 ms, matching the native code. */
    for (phase = 0; phase < period; ++phase) {
        double score = 0.0;
        for (beat = 0; ; ++beat) {
            double position = phase + beat * initial_period;
            size_t center;
            if (position >= n) break;
            center = (size_t)llround(position);
            if (center >= n) break;
            score += envelope[local_envelope_peak(envelope, n, center)];
        }
        if (score > best_score) { best_score = score; best_phase = phase; }
    }

    for (beat = 0; (double)best_phase + beat * initial_period < n; ++beat) {
        size_t center = (size_t)llround(best_phase + beat * initial_period);
        if (center >= n) break;
        size_t peak = local_envelope_peak(envelope, n, center);
        if (envelope[peak] > 0.0) {
            double x = (double)beat, y = (double)peak;
            sx += x;
            sy += y;
            sxx += x * x;
            sxy += x * y;
            syy += y * y;
            ++count;
        }
    }
    if (count < 4 || count * sxx <= sx * sx) {
        if (residual_ms) *residual_ms = 1e9;
        if (relative_period_error) *relative_period_error = 1.0;
        return initial_period;
    }

    centered_xx = sxx - sx * sx / count;
    centered_xy = sxy - sx * sy / count;
    centered_yy = syy - sy * sy / count;
    slope = centered_xy / centered_xx;
    sse = centered_yy - centered_xy * centered_xy / centered_xx;
    sigma = sqrt(fmax(0.0, sse) / (count - 2));
    slope_error = sigma / sqrt(centered_xx);
    if (residual_ms) *residual_ms = sqrt(fmax(0.0, sse) / (count - 1));
    if (relative_period_error)
        *relative_period_error = slope > 0.0 ? slope_error / slope : 1.0;
    return slope > 0.0 ? slope : initial_period;
}

static double fit_peak_period(const double *envelope, size_t n, double initial_period,
                              double *residual_ms, double *relative_period_error) {
    double first_residual, first_error, second_residual, second_error;
    double first = fit_peak_period_once(envelope, n, initial_period,
                                        &first_residual, &first_error);
    double second = fit_peak_period_once(envelope, n, first,
                                         &second_residual, &second_error);
    if (fabs(second - first) < 1.0 && second_residual <= first_residual + 0.5) {
        if (residual_ms) *residual_ms = second_residual;
        if (relative_period_error) *relative_period_error = second_error;
        return second;
    }
    if (residual_ms) *residual_ms = first_residual;
    if (relative_period_error) *relative_period_error = first_error;
    return first;
}

static size_t best_autocorrelation_lag(const Complex *correlation, size_t n, size_t fft_n) {
    size_t lag, best = 0;
    double best_score = -1e300;
    /* Prefer a local peak supported by at least four harmonic peaks. */
    for (lag = 180; lag <= 2000 && lag < n / 2; ++lag) {
        const double peak = correlation[lag].re;
        size_t neighbor, harmonic, hits = 0;
        double sum = 0.0, average;
        int is_local_peak = 1;

        for (neighbor = lag > 16 ? lag - 16 : 1;
             neighbor <= lag + 16 && neighbor < fft_n / 2; ++neighbor) {
            if (neighbor != lag && correlation[neighbor].re > peak) {
                is_local_peak = 0;
                break;
            }
        }
        if (!is_local_peak) continue;
        for (harmonic = 1; harmonic * lag <= 2000 && harmonic * lag < n / 2; ++harmonic) {
            size_t center = harmonic * lag;
            size_t first = center > 10 ? center - 10 : 1;
            size_t last = center + 10, index, found = center;
            double value = -1e300;
            if (last >= n / 2) last = n / 2 - 1;
            for (index = first; index <= last; ++index) {
                double candidate = correlation[index].re;
                if (candidate >= correlation[index - 1].re &&
                    candidate >= correlation[index + 1].re && candidate > value) {
                    value = candidate;
                    found = index;
                }
            }
            if (value > -1e299 &&
                (harmonic == 1 || (found > harmonic * lag - 11 && found < harmonic * lag + 11))) {
                sum += value;
                ++hits;
            }
        }
        if (hits < 4) continue;
        average = sum / hits;
        if (peak * 0.7 >= average) continue;
        if (average > best_score) { best_score = average; best = lag; }
    }
    /* Sparse or short material falls back to the largest single peak. */
    if (!best) {
        for (lag = 180; lag <= 2000 && lag < n / 2; ++lag) {
            if (correlation[lag].re > best_score) {
                best_score = correlation[lag].re;
                best = lag;
            }
        }
    }
    return best;
}

/* A chart can count a strong subdivision as its beat.  Keep the native
 * autocorrelation choice as the anchor, then inspect nearby faster pulses.
 * Only promote a subdivision when it has its own distinct, strong peak. */
static size_t supported_subdivision_lag(const Complex *correlation, size_t lag) {
    static const struct { double fraction, min_bpm, max_bpm, strength; } options[] = {
        {0.5,       220.0, 270.0, 0.85},
        {2.0 / 3.0, 270.0, 340.0, 0.70}
    };
    double anchor = correlation[lag].re;
    size_t option;
    if (anchor <= 0.0) return lag;
    for (option = 0; option < sizeof(options) / sizeof(options[0]); ++option) {
        size_t center = (size_t)llround(lag * options[option].fraction);
        size_t candidate, best = 0;
        double peak = 0.0, bpm;
        if (center < 180) continue;
        for (candidate = center - 4; candidate <= center + 4; ++candidate) {
            double value = correlation[candidate].re;
            if (value > peak && value >= correlation[candidate - 1].re &&
                value >= correlation[candidate + 1].re) {
                peak = value;
                best = candidate;
            }
        }
        if (!best || peak < anchor * options[option].strength) continue;
        bpm = 60000.0 / best;
        if (bpm >= options[option].min_bpm && bpm <= options[option].max_bpm)
            return best;
    }
    return lag;
}

/* Several distant autocorrelation peaks resolve sub-millisecond periods more
 * reliably than interpolating one broad peak in the filtered envelope. */
static double harmonic_peak_period(const Complex *correlation, size_t n,
                                   size_t lag, int *reliable) {
    size_t harmonic, count = 0;
    double sum_xy = 0.0, sum_xx = 0.0, sum_weight = 0.0, error = 0.0;
    double positions[12], weights[12], indices[12], period;
    size_t limit = n / 2 > 2000 ? 2000 : n / 2 - 1;
    *reliable = 0;
    for (harmonic = 1; harmonic <= 11 && harmonic * lag <= limit; ++harmonic) {
        size_t center = harmonic * lag, first = center > 10 ? center - 10 : 1;
        size_t last = center + 10, index, best = 0;
        double peak = 0.0;
        if (last > limit) last = limit;
        for (index = first; index <= last; ++index) {
            double value = correlation[index].re;
            if (value > peak && value >= correlation[index - 1].re &&
                value >= correlation[index + 1].re) {
                peak = value;
                best = index;
            }
        }
        if (!best || peak < correlation[lag].re * 0.35) continue;
        positions[count] = (double)best;
        weights[count] = peak;
        indices[count] = (double)harmonic;
        sum_xy += peak * harmonic * best;
        sum_xx += peak * harmonic * harmonic;
        sum_weight += peak;
        ++count;
    }
    if (count < 3 || sum_xx <= 0.0) return (double)lag;
    period = sum_xy / sum_xx;
    for (harmonic = 0; harmonic < count; ++harmonic) {
        double expected = indices[harmonic] * period;
        double difference = positions[harmonic] - expected;
        error += weights[harmonic] * difference * difference;
    }
    *reliable = sqrt(error / sum_weight) <= 1.5;
    return period;
}

static double estimate_bpm(const double *envelope, size_t n,
                           double *fit_residual, double *relative_error) {
    size_t fft_n = 1, index, lag;
    Complex *correlation;
    double left, center, right, denominator, shift, period, harmonic_period;
    int harmonic_reliable;
    while (fft_n < n * 2) {
        if (fft_n > ((size_t)-1) / 2) return 0.0;
        fft_n <<= 1;
    }
    correlation = (Complex *)calloc(fft_n, sizeof(Complex));
    if (!correlation) return 0.0;
    for (index = 0; index < n; ++index) correlation[index].re = envelope[index];
    fft(correlation, fft_n, 0);
    for (index = 0; index < fft_n; ++index) {
        correlation[index].re = correlation[index].re * correlation[index].re +
                                correlation[index].im * correlation[index].im;
        correlation[index].im = 0.0;
    }
    fft(correlation, fft_n, 1);
    lag = best_autocorrelation_lag(correlation, n, fft_n);
    if (!lag) { free(correlation); return 0.0; }
    lag = supported_subdivision_lag(correlation, lag);
    harmonic_period = harmonic_peak_period(correlation, n, lag, &harmonic_reliable);

    /* Parabolic interpolation gives a sub-millisecond starting period. */
    left = correlation[lag - 1].re;
    center = correlation[lag].re;
    right = correlation[lag + 1].re;
    denominator = left - 2.0 * center + right;
    shift = fabs(denominator) > 1e-20 ? 0.5 * (left - right) / denominator : 0.0;
    period = harmonic_reliable ? harmonic_period :
             lag + fmax(-0.5, fmin(0.5, shift));
    free(correlation);
    {
        double fitted = fit_peak_period(envelope, n, period, fit_residual, relative_error);
        if (!harmonic_reliable || fabs(fitted - period) > 0.5) period = fitted;
    }
    return 60000.0 / period;
}

static int fluctuates(const double *env, size_t n, double whole) {
    static const double beat_factors[] = {0.5, 2.0 / 3.0, 0.75, 1.0,
                                          4.0 / 3.0, 1.5, 2.0, 3.0};
    const size_t window = 16000, step = 8000;
    const double tolerance = fmax(3.0, whole * 0.03);
    size_t start, factor;
    int windows = 0, outliers = 0, streak = 0, longest_streak = 0;

    if (n < window * 2) return 0;
    for (start = 0; start + window <= n; start += step) {
        double local = estimate_bpm(env + start, window, NULL, NULL);
        double distance;
        if (local <= 0.0) {
            streak = 0;
            continue;
        }

        distance = fabs(local - whole);
        for (factor = 0; factor < sizeof(beat_factors) / sizeof(beat_factors[0]); ++factor) {
            double candidate = local * beat_factors[factor];
            double difference = fabs(candidate - whole);
            if (difference < distance) distance = difference;
        }

        ++windows;
        if (distance > tolerance) {
            ++outliers;
            ++streak;
            if (streak > longest_streak) longest_streak = streak;
        } else {
            streak = 0;
        }
    }

    /* A single mistaken beat subdivision does not establish a tempo change. */
    return windows >= 3 && outliers >= 3 && outliers * 5 >= windows && longest_streak >= 3;
}

static double native_grid_quantize(double bpm, double bpm_error) {
    static const double factors[7]={1.0,2.0,3.0,10.0,20.0,100.0,200.0};
    static const double tolerances[7]={3.0,2.5,2.0,2.0,1.5,1.5,1.0};
    unsigned i;
    for(i=0;i<7;++i) {
        double grid=1.0/factors[i];
        if(fabs(remainder(bpm,grid))<=bpm_error*tolerances[i])
            return round(bpm*factors[i])/factors[i];
    }
    return bpm;
}

#define MAX_RANGES 128
#define TUI_MAX_WIDTH 2000

typedef struct {
    const Wave *wave;
    const char *name, *status, *command;
    const MeasureRange *ranges;
    size_t frames;
    int range_count, pending, mouse_x;
    double pending_time, current_time, span, duration;
} TuiView;

typedef struct {
    int width, height, graph_height, visible_ranges, visible_complete;
} TuiLayout;

typedef struct {
    int top, bottom, glyph, filled;
} TuiColumn;

typedef void (*TuiWriteLine)(void *context, int row, const char *line);

static TuiLayout tui_layout(int width, int height, int range_count, int pending) {
    TuiLayout layout;
    int max_range_rows;
    if (width < 20) width = 20;
    if (width > TUI_MAX_WIDTH) width = TUI_MAX_WIDTH;
    if (width > 20) --width; /* Leave the last column empty to avoid wrapping. */
    if (height < 8) height = 8;

    layout.width = width;
    layout.height = height;
    max_range_rows = height - 11;
    if (max_range_rows < 0) max_range_rows = 0;
    layout.visible_ranges = range_count + (pending ? 1 : 0);
    if (layout.visible_ranges > max_range_rows) layout.visible_ranges = max_range_rows;
    layout.visible_complete = range_count < layout.visible_ranges ? range_count : layout.visible_ranges;
    layout.graph_height = (height - 5 - layout.visible_ranges) / 2;
    if (layout.graph_height < 3) layout.graph_height = 3;
    return layout;
}

static const char *tui_basename(const char *path) {
    const char *slash, *backslash;
    if (!path) return "audio";
    slash = strrchr(path, '/');
    backslash = strrchr(path, '\\');
    if (backslash && (!slash || backslash > slash)) return backslash + 1;
    return slash ? slash + 1 : path;
}

static int tui_time_x(const TuiView *view, TuiLayout layout, double time) {
    return layout.width / 2 + (int)((time - view->current_time) *
                                    (layout.width - 3) / view->span);
}

static TuiColumn tui_column(const TuiView *view, TuiLayout layout, int x) {
    const Wave *wave = view->wave;
    const double bin_seconds = view->span / (layout.width - 3);
    const double center = view->current_time + (x - layout.width / 2) * bin_seconds;
    const double left = (center - bin_seconds * 0.5) * wave->rate;
    const double right = (center + bin_seconds * 0.5) * wave->rate;
    long long low = (long long)floor(left), high = (long long)ceil(right);
    double sum_squares = 0.0, peak[3] = {0.0, 0.0, 0.0};
    size_t first, last, frame;
    int amplitude, glyph = 0;
    TuiColumn column;

    if (low < 0) low = 0;
    if (high < 0) high = 0;
    if (low > (long long)view->frames) low = (long long)view->frames;
    if (high > (long long)view->frames) high = (long long)view->frames;
    first = (size_t)low;
    last = (size_t)high;
    for (frame = first; frame < last; ++frame) {
        double sample = 0.0;
        unsigned channel;
        size_t segment = ((frame - first) * 3) / (last - first);
        if (segment > 2) segment = 2;
        for (channel = 0; channel < wave->channels; ++channel) {
            const size_t index = (frame * wave->channels + channel) * (wave->bits / 8);
            sample += sample_at(wave->data + index, wave->format, wave->bits);
        }
        sample /= wave->channels;
        sum_squares += sample * sample;
        if (fabs(sample) > peak[segment]) peak[segment] = fabs(sample);
    }
    if (peak[1] > peak[glyph]) glyph = 1;
    if (peak[2] > peak[glyph]) glyph = 2;
    amplitude = last > first ? (int)(sqrt(sum_squares / (last - first)) *
                                     (layout.graph_height - 2) * 4.0) : 0;
    column.top = layout.graph_height / 2 - amplitude / 2;
    column.bottom = layout.graph_height / 2 + amplitude / 2;
    if (column.top < 0) column.top = 0;
    if (column.bottom >= layout.graph_height) column.bottom = layout.graph_height - 1;
    column.glyph = glyph;
    column.filled = amplitude > 0;
    return column;
}

static void tui_draw_range(const TuiView *view, TuiLayout layout, int range_index,
                           TuiWriteLine write, void *context) {
    char row[TUI_MAX_WIDTH + 1], label[24];
    int x, start = tui_time_x(view, layout, view->ranges[range_index].start);
    int end = tui_time_x(view, layout, view->ranges[range_index].end);
    memset(row, ' ', (size_t)layout.width);
    if (end >= 1 && start < layout.width - 1) {
        int label_start, label_length;
        if (start < 1) start = 1;
        if (end >= layout.width - 1) end = layout.width - 2;
        if (start <= end) {
            row[start] = '[';
            row[end] = ']';
            for (x = start + 1; x < end; ++x) row[x] = '-';
            snprintf(label, sizeof(label), "<%d>", range_index + 1);
            label_length = (int)strlen(label);
            label_start = start + 1 + (end - start - 1 - label_length) / 2;
            if (label_start > start && label_start + label_length < end)
                memcpy(row + label_start, label, (size_t)label_length);
        }
    }
    row[layout.width] = 0;
    write(context, layout.graph_height + 4 + range_index, row);
}

static void tui_draw_pending(const TuiView *view, TuiLayout layout,
                             TuiWriteLine write, void *context) {
    char row[TUI_MAX_WIDTH + 1];
    int x, start = tui_time_x(view, layout, view->pending_time), end = view->mouse_x;
    memset(row, ' ', (size_t)layout.width);
    if (start < 1) start = 1;
    if (start > layout.width - 2) start = layout.width - 2;
    if (end < 1) end = 1;
    if (end > layout.width - 2) end = layout.width - 2;
    row[start] = '[';
    if (start < end) for (x = start + 1; x <= end; ++x) row[x] = '-';
    else for (x = end; x < start; ++x) row[x] = '-';
    row[layout.width] = 0;
    write(context, layout.graph_height + 4 + layout.visible_complete, row);
}

static void tui_render(const TuiView *view, TuiLayout layout, TuiLayout *previous,
                       TuiWriteLine write, void *context) {
    static const char *glyphs[3] = {"\xE2\x96\x8F", "\xE2\x94\x82", "\xE2\x96\x95"};
    TuiColumn columns[TUI_MAX_WIDTH];
    char row[TUI_MAX_WIDTH * 3 + 4], message[512];
    int x, y;

    if (previous->width != layout.width || previous->height != layout.height ||
        previous->graph_height != layout.graph_height ||
        previous->visible_ranges != layout.visible_ranges) {
        for (y = 2; y <= layout.height - 3; ++y) write(context, y, "");
    }
    *previous = layout;

    snprintf(message, sizeof(message), "%s  Time: %.2f / %.2f sec   ranges:%d",
             tui_basename(view->name), view->current_time, view->duration, view->range_count);
    message[layout.width < (int)sizeof(message) ? layout.width : (int)sizeof(message) - 1] = 0;
    write(context, 1, message);

    memset(row, '-', (size_t)layout.width);
    row[0] = '+';
    row[layout.width - 1] = '+';
    row[layout.width] = 0;
    write(context, 2, row);
    for (x = 1; x < layout.width - 1; ++x) columns[x] = tui_column(view, layout, x);
    for (y = 0; y < layout.graph_height; ++y) {
        size_t used = 0;
        row[used++] = '|';
        for (x = 1; x < layout.width - 1; ++x) {
            if (x == layout.width / 2) row[used++] = '#';
            else if (columns[x].filled && y >= columns[x].top && y <= columns[x].bottom) {
                const char *glyph = glyphs[columns[x].glyph];
                while (*glyph) row[used++] = *glyph++;
            } else row[used++] = ' ';
        }
        row[used++] = '|';
        row[used] = 0;
        write(context, y + 3, row);
    }
    memset(row, '-', (size_t)layout.width);
    row[0] = '+';
    row[layout.width - 1] = '+';
    row[layout.width] = 0;
    write(context, layout.graph_height + 3, row);

    for (y = 0; y < layout.visible_complete; ++y)
        tui_draw_range(view, layout, y, write, context);
    if (view->pending && layout.visible_ranges > layout.visible_complete)
        tui_draw_pending(view, layout, write, context);

    if (layout.height >= 9)
        write(context, layout.height - 2, "Ctrl+C exit | Space play/pause | Wheel seek (stops playback)");
    snprintf(message, sizeof(message),
             "Left click x2 range | Right click delete | get+Enter results | %s", view->status);
    message[layout.width < (int)sizeof(message) ? layout.width : (int)sizeof(message) - 1] = 0;
    write(context, layout.height - 1, message);
    snprintf(message, sizeof(message), "Command: %s", view->command);
    message[layout.width < (int)sizeof(message) ? layout.width : (int)sizeof(message) - 1] = 0;
    write(context, layout.height, message);
}

static void tui_print_results(const Wave *wave, const MeasureRange *ranges, int count) {
    int index;
    for (index = 0; index < count; ++index) {
        const size_t bytes_per_frame = (wave->bits / 8) * wave->channels;
        const size_t first = (size_t)(ranges[index].start * wave->rate);
        const size_t last = (size_t)(ranges[index].end * wave->rate);
        Wave part = *wave;
        double *envelope, bpm = 0.0, offset = 0.0, residual = 0.0, error = 1.0;
        size_t envelope_length;
        part.data = wave->data + first * bytes_per_frame;
        part.size = (last - first) * bytes_per_frame;
        if (last > first && (envelope = make_envelope(&part, &envelope_length, &offset)) != NULL) {
            bpm = estimate_bpm(envelope, envelope_length, &residual, &error);
            if (bpm > 0.0 && error <= 0.00005)
                bpm = native_grid_quantize(bpm, bpm * error);
            free(envelope);
        }
        printf("%d.BPM:%.1f,offset:%.1f ms;\n", index + 1, bpm, offset);
    }
}

static double tui_time_at_x(int x, int width, double current_time,
                            double span, double duration) {
    double time = current_time + (x - width / 2) * span / (width - 3);
    if (time < 0.0) time = 0.0;
    if (time > duration) time = duration;
    return time;
}

static void tui_select_range(MeasureRange *ranges, int *count, int *pending,
                             double *pending_time, double time, char *status) {
    if (!*pending) {
        *pending_time = time;
        *pending = 1;
        strcpy(status, "Start selected; click end");
    } else if (*count < MAX_RANGES) {
        ranges[*count].start = fmin(*pending_time, time);
        ranges[*count].end = fmax(*pending_time, time);
        ++*count;
        *pending = 0;
        strcpy(status, "Range added");
    }
}

static void tui_delete_range(MeasureRange *ranges, int *count, double time,
                             int mouse_y, int graph_height, char *status) {
    int index;
    for (index = 0; index < *count; ++index) {
        int on_wave = mouse_y >= 2 && mouse_y <= graph_height + 1;
        int on_range_row = mouse_y == graph_height + 3 + index;
        if (time >= ranges[index].start && time <= ranges[index].end &&
            (on_wave || on_range_row)) {
            memmove(&ranges[index], &ranges[index + 1],
                    (size_t)(*count - index - 1) * sizeof(ranges[0]));
            --*count;
            strcpy(status, "Range deleted");
            return;
        }
    }
}

static int tui_submit_command(const Wave *wave, char *command,
                              char *status, size_t status_cap) {
    if (!strcmp(command, "get")) return 1;
    if (!strcmp(command, "offset")) {
        size_t envelope_length;
        double offset;
        double *envelope = make_envelope(wave, &envelope_length, &offset);
        if (envelope) {
            snprintf(status, status_cap, "Offset: %.1f ms", offset);
            free(envelope);
        }
    }
    command[0] = 0;
    return 0;
}

#ifdef _WIN32
static volatile LONG tui_quit_requested=0;

static BOOL WINAPI tui_ctrl_handler(DWORD event) {
    if(event==CTRL_C_EVENT||event==CTRL_BREAK_EVENT){
        InterlockedExchange(&tui_quit_requested,1);
        return TRUE;
    }
    return FALSE;
}

static void stop_tui_audio(HWAVEOUT *out, WAVEHDR *hdr, short **pcm, int *playing) {
    if (*out) {
        waveOutReset(*out);
        if (hdr->dwFlags & WHDR_PREPARED) waveOutUnprepareHeader(*out, hdr, sizeof(*hdr));
        waveOutClose(*out);
        *out = NULL;
    }
    free(*pcm); *pcm = NULL; *playing = 0;
}

static int start_tui_audio(Wave *w, size_t frames, double *current_time,
                           HWAVEOUT *out, WAVEHDR *hdr, short **pcm,
                           size_t *pcm_count, double *playback_base) {
    WAVEFORMATEX fmt; MMRESULT mr; size_t start, q;
    start=(size_t)(*current_time*w->rate);
    if(start>=frames) start=0;
    *pcm_count=(frames-start)*w->channels;
    *pcm=(short*)malloc(*pcm_count*sizeof(short));
    if(!*pcm) return 0;
    for(q=0;q<*pcm_count;q++){
        double v=sample_at(w->data+(start*w->channels+q)*(w->bits/8),w->format,w->bits);
        if(v>1)v=1;
        if(v< -1)v=-1;
        (*pcm)[q]=(short)(v*32767);
    }
    memset(&fmt,0,sizeof(fmt)); fmt.wFormatTag=WAVE_FORMAT_PCM;
    fmt.nChannels=(WORD)w->channels; fmt.nSamplesPerSec=w->rate; fmt.wBitsPerSample=16;
    fmt.nBlockAlign=(WORD)(2*w->channels); fmt.nAvgBytesPerSec=fmt.nSamplesPerSec*fmt.nBlockAlign;
    memset(hdr,0,sizeof(*hdr)); hdr->lpData=(LPSTR)*pcm;
    hdr->dwBufferLength=(DWORD)(*pcm_count*sizeof(short));
    mr=waveOutOpen(out,WAVE_MAPPER,&fmt,0,0,CALLBACK_NULL);
    if(mr!=MMSYSERR_NOERROR){free(*pcm);*pcm=NULL;return 0;}
    mr=waveOutPrepareHeader(*out,hdr,sizeof(*hdr));
    if(mr==MMSYSERR_NOERROR)mr=waveOutWrite(*out,hdr,sizeof(*hdr));
    if(mr!=MMSYSERR_NOERROR){stop_tui_audio(out,hdr,pcm,&(int){1});return 0;}
    *current_time=(double)start/w->rate; *playback_base=*current_time;
    return 1;
}

static double tui_audio_position(HWAVEOUT out, double playback_base, unsigned rate, unsigned channels, double fallback) {
    MMTIME mt; memset(&mt,0,sizeof(mt)); mt.wType=TIME_BYTES;
    if(waveOutGetPosition(out,&mt,sizeof(mt))==MMSYSERR_NOERROR&&mt.wType==TIME_BYTES&&channels)
        return playback_base+(double)mt.u.cb/(2.0*channels*rate);
    return fallback;
}

static void tui_write(HANDLE hout, const char *text) {
    DWORD written=0; WriteConsoleA(hout,text,(DWORD)strlen(text),&written,NULL);
}

static void tui_write_at(HANDLE hout, int row, const char *text) {
    char seq[32];
    snprintf(seq,sizeof(seq),"\x1b[%d;1H\x1b[2K",row);
    tui_write(hout,seq); tui_write(hout,text);
}

static void tui_write_line(void *context, int row, const char *text) {
    tui_write_at((HANDLE)context, row, text);
}

static int run_tui(Wave *w, const char *name) {
    HANDLE hin=GetStdHandle(STD_INPUT_HANDLE), hout=GetStdHandle(STD_OUTPUT_HANDLE);
    DWORD oldmode=0, mode, oldout=0, last_buttons=0; UINT oldcp; CONSOLE_SCREEN_BUFFER_INFO cs; INPUT_RECORD ev; DWORD got;
    MeasureRange ranges[MAX_RANGES]; int nr=0,pending=0,play=0,mouse_x=40,redraw=1; double pending_t=0,span,current_time=0;
    TuiLayout previous={.width=-1};
    char cmd[128]="", status[256]=""; size_t framebytes=(w->bits/8)*w->channels, frames=w->size/framebytes;
    HWAVEOUT out=NULL; WAVEHDR hdr; short *pcm=NULL; size_t pcm_count=0; double playback_base=0,duration=(double)frames/w->rate;
    if(!GetConsoleMode(hin,&oldmode)||!GetConsoleMode(hout,&oldout)){fprintf(stderr,"TUI requires a Windows console.\n");return -1;}
    InterlockedExchange(&tui_quit_requested,0);
    if(!SetConsoleCtrlHandler(tui_ctrl_handler,TRUE)){fprintf(stderr,"TUI cannot register Ctrl+C handling.\n");return -1;}
    oldcp=GetConsoleOutputCP();SetConsoleOutputCP(CP_UTF8);
    mode=(oldmode&~(ENABLE_LINE_INPUT|ENABLE_ECHO_INPUT|ENABLE_QUICK_EDIT_MODE))|ENABLE_MOUSE_INPUT|ENABLE_EXTENDED_FLAGS|ENABLE_WINDOW_INPUT;
    SetConsoleMode(hin,mode);
    if(!SetConsoleMode(hout,oldout|ENABLE_PROCESSED_OUTPUT|ENABLE_VIRTUAL_TERMINAL_PROCESSING)){
        SetConsoleMode(hin,oldmode);SetConsoleOutputCP(oldcp);SetConsoleCtrlHandler(tui_ctrl_handler,FALSE);fprintf(stderr,"TUI requires Windows Terminal or VT console support.\n");return -1;
    }
    tui_write(hout,"\x1b[?25l\x1b[2J\x1b[H");
    memset(ranges,0,sizeof(ranges)); span=fmin(duration,24.0); if(span<1)span=duration;
    for(;;){
        if(InterlockedCompareExchange(&tui_quit_requested,0,0))break;
        if(play&&out){current_time=tui_audio_position(out,playback_base,w->rate,w->channels,current_time);if(current_time>duration)current_time=duration;
            if(hdr.dwFlags&WHDR_DONE){current_time=duration;stop_tui_audio(&out,&hdr,&pcm,&play);strcpy(status,"Playback finished");redraw=1;}}
        int width = 80, height = 25, graph_h;
        TuiLayout layout;
        GetConsoleScreenBufferInfo(hout, &cs);
        width = cs.srWindow.Right - cs.srWindow.Left + 1;
        height = cs.srWindow.Bottom - cs.srWindow.Top + 1;
        if (current_time < 0.0) current_time = 0.0;
        if (current_time > duration) current_time = duration;
        layout = tui_layout(width, height, nr, pending);
        width = layout.width;
        graph_h = layout.graph_height;
        if (redraw || play) {
            TuiView view = {
                .wave=w, .name=name, .status=status, .command=cmd, .ranges=ranges,
                .frames=frames, .range_count=nr, .pending=pending, .mouse_x=mouse_x,
                .pending_time=pending_t, .current_time=current_time,
                .span=span, .duration=duration
            };
            tui_render(&view, layout, &previous, tui_write_line, hout);
        }
        if(WaitForSingleObject(hin,50)!=WAIT_OBJECT_0){redraw=0;continue;}
        if(!ReadConsoleInputA(hin,&ev,1,&got)||!got){redraw=0;continue;}
        redraw=1;
        if(ev.EventType==WINDOW_BUFFER_SIZE_EVENT)continue;
        if(ev.EventType==KEY_EVENT&&ev.Event.KeyEvent.bKeyDown){char c=ev.Event.KeyEvent.uChar.AsciiChar;
            if(c==27||c==3||(ev.Event.KeyEvent.wVirtualKeyCode=='C'&&(ev.Event.KeyEvent.dwControlKeyState&(LEFT_CTRL_PRESSED|RIGHT_CTRL_PRESSED))))break;
            if(ev.Event.KeyEvent.wVirtualKeyCode==VK_HOME||ev.Event.KeyEvent.wVirtualKeyCode==VK_END){
                current_time=ev.Event.KeyEvent.wVirtualKeyCode==VK_HOME?0:duration;
                if(play){stop_tui_audio(&out,&hdr,&pcm,&play);if(current_time<duration&&start_tui_audio(w,frames,&current_time,&out,&hdr,&pcm,&pcm_count,&playback_base))play=1;}
            }
            else if(c==' '){
                if(!play){if(start_tui_audio(w,frames,&current_time,&out,&hdr,&pcm,&pcm_count,&playback_base)){play=1;strcpy(status,"Playing");}else strcpy(status,"Playback could not start");}
                else{current_time=tui_audio_position(out,playback_base,w->rate,w->channels,current_time);if(current_time>duration)current_time=duration;stop_tui_audio(&out,&hdr,&pcm,&play);strcpy(status,"Paused");}
            }
            else if(c=='\r'){
                if(tui_submit_command(w,cmd,status,sizeof(status)))break;
            }
            else if(c==8){size_t n=strlen(cmd);if(n)cmd[n-1]=0;}
            else if(c>=32&&strlen(cmd)<sizeof(cmd)-2){size_t n=strlen(cmd);cmd[n]=c;cmd[n+1]=0;}
        } else if(ev.EventType==MOUSE_EVENT){MOUSE_EVENT_RECORD m=ev.Event.MouseEvent;
            int mx=m.dwMousePosition.X-cs.srWindow.Left,my=m.dwMousePosition.Y-cs.srWindow.Top;
            if(mx<1)mx=1;
            if(mx>width-2)mx=width-2;
            mouse_x=mx;
            if(m.dwEventFlags==MOUSE_WHEELED){short d=(short)HIWORD(m.dwButtonState);if(play&&out){current_time=tui_audio_position(out,playback_base,w->rate,w->channels,current_time);stop_tui_audio(&out,&hdr,&pcm,&play);strcpy(status,"Playback stopped");}current_time+=(double)d/120.0*span*.12;if(current_time<0)current_time=0;if(current_time>duration)current_time=duration;}
            else {
                double t=tui_time_at_x(mx,width,current_time,span,duration);
                DWORD buttons=m.dwButtonState;
                if((buttons&FROM_LEFT_1ST_BUTTON_PRESSED)&&!(last_buttons&FROM_LEFT_1ST_BUTTON_PRESSED)&&my>=2&&my<=graph_h+1){
                    tui_select_range(ranges,&nr,&pending,&pending_t,t,status);
                }
                if((buttons&RIGHTMOST_BUTTON_PRESSED)&&!(last_buttons&RIGHTMOST_BUTTON_PRESSED)){
                    tui_delete_range(ranges,&nr,t,my,graph_h,status);
                }
                last_buttons=buttons;
            }
        }
    }
    if(play&&out)stop_tui_audio(&out,&hdr,&pcm,&play);
    tui_write(hout,"\x1b[0m\x1b[?25h\x1b[2J\x1b[H");
    SetConsoleMode(hin,oldmode);SetConsoleMode(hout,oldout);SetConsoleOutputCP(oldcp);
    SetConsoleCtrlHandler(tui_ctrl_handler,FALSE);
    if(!strcmp(cmd,"get")) tui_print_results(w, ranges, nr);
    return 0;
}
#else
typedef struct {
    pid_t pid;
    char path[4096];
    double base, started;
} PosixPlayback;

static double tui_now(void) {
    struct timespec ts;
    if (clock_gettime(CLOCK_MONOTONIC, &ts) != 0) return 0.0;
    return (double)ts.tv_sec + (double)ts.tv_nsec / 1000000000.0;
}

static int tui_find_program(const char *program) {
    const char *path = getenv("PATH"), *p;
    if (!path) return 0;
    p = path;
    do {
        const char *end = strchr(p, ':');
        size_t dir_len = end ? (size_t)(end - p) : strlen(p), name_len = strlen(program);
        char candidate[4096];
        if (dir_len + name_len + 2 < sizeof(candidate)) {
            if (dir_len) memcpy(candidate, p, dir_len);
            else { candidate[0] = '.'; dir_len = 1; }
            candidate[dir_len] = '/';
            memcpy(candidate + dir_len + 1, program, name_len + 1);
            if (access(candidate, X_OK) == 0) return 1;
        }
        if (!end) break;
        p = end + 1;
    } while (1);
    return 0;
}

static void tui_write_u16(FILE *f, uint16_t n) {
    unsigned char bytes[2] = {(unsigned char)n, (unsigned char)(n >> 8)};
    fwrite(bytes, 1, 2, f);
}

static void tui_write_u32(FILE *f, uint32_t n) {
    unsigned char bytes[4] = {(unsigned char)n, (unsigned char)(n >> 8),
                              (unsigned char)(n >> 16), (unsigned char)(n >> 24)};
    fwrite(bytes, 1, 4, f);
}

/* A fresh PCM WAV makes seeking work with afplay, ffplay, and aplay alike. */
static int tui_write_playback_wave(const Wave *w, size_t start, size_t frames,
                                   char *path, size_t cap) {
    char template[] = "/tmp/bpm-play-XXXXXX";
    unsigned channels = w->channels > 1 ? 2 : 1;
    size_t remaining = frames - start, i, bytes = (w->bits / 8) * w->channels;
    FILE *f;
    int fd = mkstemp(template), ok = 1;
    unsigned char buffer[16384];
    size_t buffered = 0;
    if (fd < 0) return 0;
    if (strlen(template) + 5 > cap || remaining > (UINT32_MAX - 36u) / (channels * 2u)) {
        close(fd); unlink(template); return 0;
    }
    snprintf(path, cap, "%s.wav", template);
    if (rename(template, path) != 0) { close(fd); unlink(template); return 0; }
    f = fdopen(fd, "wb");
    if (!f) { close(fd); unlink(path); path[0] = 0; return 0; }
    fwrite("RIFF", 1, 4, f); tui_write_u32(f, 36u + (uint32_t)(remaining * channels * 2u));
    fwrite("WAVEfmt ", 1, 8, f); tui_write_u32(f, 16);
    tui_write_u16(f, 1); tui_write_u16(f, (uint16_t)channels);
    tui_write_u32(f, w->rate); tui_write_u32(f, w->rate * channels * 2u);
    tui_write_u16(f, (uint16_t)(channels * 2u)); tui_write_u16(f, 16);
    fwrite("data", 1, 4, f); tui_write_u32(f, (uint32_t)(remaining * channels * 2u));
    for (i = start; i < frames; ++i) {
        unsigned c;
        for (c = 0; c < channels; ++c) {
            double v = sample_at(w->data + i * bytes + c * (w->bits / 8), w->format, w->bits);
            int16_t pcm;
            if (v > 1.0) v = 1.0;
            if (v < -1.0) v = -1.0;
            pcm = (int16_t)lrint(v * 32767.0);
            buffer[buffered++] = (unsigned char)pcm;
            buffer[buffered++] = (unsigned char)((uint16_t)pcm >> 8);
            if (buffered == sizeof(buffer)) {
                if (fwrite(buffer, 1, buffered, f) != buffered) ok = 0;
                buffered = 0;
            }
        }
        if (!ok) break;
    }
    if (buffered && fwrite(buffer, 1, buffered, f) != buffered) ok = 0;
    if (fclose(f) != 0) ok = 0;
    if (!ok) { unlink(path); path[0] = 0; }
    return ok;
}

static void tui_stop_audio(PosixPlayback *audio) {
    if (audio->pid > 0) {
        int status;
        kill(audio->pid, SIGTERM);
        while (waitpid(audio->pid, &status, 0) < 0 && errno == EINTR) { }
        audio->pid = 0;
    }
    if (audio->path[0]) { unlink(audio->path); audio->path[0] = 0; }
}

static int tui_start_audio(const Wave *w, size_t frames, double *current_time,
                           PosixPlayback *audio) {
    const char *player = NULL;
    pid_t child;
    size_t start = (size_t)(*current_time * w->rate);
#ifdef __APPLE__
    if (tui_find_program("afplay")) player = "afplay";
#endif
    if (!player && tui_find_program("ffplay")) player = "ffplay";
    if (!player && tui_find_program("aplay")) player = "aplay";
    if (!player) return 0;
    if (start >= frames) start = 0;
    if (!tui_write_playback_wave(w, start, frames, audio->path, sizeof(audio->path))) return 0;
    child = fork();
    if (child == 0) {
        int nullfd = open("/dev/null", O_RDWR);
        if (nullfd >= 0) {
            dup2(nullfd, STDIN_FILENO); dup2(nullfd, STDOUT_FILENO); dup2(nullfd, STDERR_FILENO);
            if (nullfd > STDERR_FILENO) close(nullfd);
        }
        if (!strcmp(player, "ffplay"))
            execlp(player, player, "-nodisp", "-autoexit", "-loglevel", "quiet", audio->path, (char *)NULL);
        else if (!strcmp(player, "aplay"))
            execlp(player, player, "-q", audio->path, (char *)NULL);
        else execlp(player, player, audio->path, (char *)NULL);
        _exit(127);
    }
    if (child < 0) { tui_stop_audio(audio); return 0; }
    audio->pid = child;
    *current_time = (double)start / w->rate;
    audio->base = *current_time;
    audio->started = tui_now();
    return 1;
}

static double tui_audio_position(const PosixPlayback *audio, double duration) {
    double position = audio->base + tui_now() - audio->started;
    return position > duration ? duration : position;
}

static void tui_write_at(int row, const char *line) {
    printf("\x1b[%d;1H\x1b[2K%s", row, line);
}

static void tui_write_line(void *context, int row, const char *line) {
    (void)context;
    tui_write_at(row, line);
}

static int run_tui(Wave *w, const char *name) {
    struct termios old_term, raw_term;
    MeasureRange ranges[MAX_RANGES];
    PosixPlayback audio = {0};
    int nr=0, pending=0, play=0, mouse_x=40, redraw=1;
    int done=0;
    TuiLayout previous={.width=-1};
    double pending_t=0, current_time=0;
    char cmd[128]="", status[256]="";
    size_t framebytes=(w->bits/8)*w->channels, frames=w->size/framebytes;
    double duration=(double)frames/w->rate, span=fmin(duration,24.0);
    char escape[64]; size_t escape_n=0;
    if (!isatty(STDIN_FILENO) || !isatty(STDOUT_FILENO) || tcgetattr(STDIN_FILENO,&old_term) != 0) {
        fprintf(stderr,"TUI requires an interactive terminal.\n"); return -1;
    }
    raw_term=old_term;
    raw_term.c_lflag &= (tcflag_t)~(ICANON|ECHO|ISIG);
    raw_term.c_iflag &= (tcflag_t)~(IXON|ICRNL);
    raw_term.c_cc[VMIN]=0; raw_term.c_cc[VTIME]=0;
    if (tcsetattr(STDIN_FILENO,TCSANOW,&raw_term) != 0) {
        fprintf(stderr,"TUI could not configure the terminal.\n"); return -1;
    }
    fputs("\x1b[?1049h\x1b[?25l\x1b[?1000h\x1b[?1003h\x1b[?1006h\x1b[2J\x1b[H",stdout);
    fflush(stdout);
    memset(ranges,0,sizeof(ranges));
    if(span<1) span=duration;
    while(!done) {
        struct winsize ws;
        int width=80,height=25,graph_h;
        TuiLayout layout;
        if(play && audio.pid>0) {
            int child_status;
            pid_t ended=waitpid(audio.pid,&child_status,WNOHANG);
            current_time=tui_audio_position(&audio,duration);
            if(ended==audio.pid) {
                audio.pid=0;
                if(audio.path[0]) { unlink(audio.path); audio.path[0]=0; }
                if(current_time>=duration-0.5) current_time=duration;
                play=0; strcpy(status,"Playback finished"); redraw=1;
            }
        }
        if(ioctl(STDOUT_FILENO,TIOCGWINSZ,&ws)==0) {width=ws.ws_col;height=ws.ws_row;}
        if (current_time < 0.0) current_time = 0.0;
        if (current_time > duration) current_time = duration;
        layout = tui_layout(width, height, nr, pending);
        width = layout.width;
        graph_h = layout.graph_height;
        if (redraw || play) {
            TuiView view = {
                .wave=w, .name=name, .status=status, .command=cmd, .ranges=ranges,
                .frames=frames, .range_count=nr, .pending=pending, .mouse_x=mouse_x,
                .pending_time=pending_t, .current_time=current_time,
                .span=span, .duration=duration
            };
            tui_render(&view, layout, &previous, tui_write_line, NULL);
            fflush(stdout);
        }
        {fd_set readfds;struct timeval timeout={0,50000};int ready;unsigned char input[128];ssize_t got,k;
            FD_ZERO(&readfds);FD_SET(STDIN_FILENO,&readfds);
            ready=select(STDIN_FILENO+1,&readfds,NULL,NULL,&timeout);
            if(ready<=0){redraw=0;continue;}
            got=read(STDIN_FILENO,input,sizeof(input));
            if(got<=0){redraw=0;continue;}
            redraw=1;
            for(k=0;k<got&&!done;++k) {
                unsigned char c=input[k];
                if(escape_n) {
                    if(escape_n<sizeof(escape)-1)escape[escape_n++]=(char)c;
                    else {escape_n=0;continue;}
                    escape[escape_n]=0;
                    if(escape_n==2&&c!='['){escape_n=0;continue;}
                    if(c>='@'&&c<='~'&&escape_n>=3) {
                        int button,mx,my;char end;
                        if(sscanf(escape,"\x1b[<%d;%d;%d%c",&button,&mx,&my,&end)==4&&(end=='M'||end=='m')) {
                            int click_x=mx-1,click_y=my-1;
                            double t;
                            if(click_x<1)click_x=1;if(click_x>width-2)click_x=width-2;
                            mouse_x=click_x;
                            t=tui_time_at_x(click_x,width,current_time,span,duration);
                            if((button&64)&&end=='M') {
                                if(play){current_time=tui_audio_position(&audio,duration);tui_stop_audio(&audio);play=0;strcpy(status,"Playback stopped");}
                                current_time+=(button&1)?-span*.12:span*.12;
                                if(current_time<0)current_time=0;if(current_time>duration)current_time=duration;
                            } else if(end=='M'&&!(button&32)) {
                                if((button&3)==0&&click_y>=2&&click_y<=graph_h+1) {
                                    tui_select_range(ranges,&nr,&pending,&pending_t,t,status);
                                } else if((button&3)==2) {
                                    tui_delete_range(ranges,&nr,t,click_y,graph_h,status);
                                }
                            }
                        } else if(!strcmp(escape,"\x1b[H")||!strcmp(escape,"\x1b[1~")||
                                  !strcmp(escape,"\x1b[F")||!strcmp(escape,"\x1b[4~")) {
                            int home=(escape[2]=='H'||escape[2]=='1');
                            if(play){tui_stop_audio(&audio);play=0;}
                            current_time=home?0:duration;
                        }
                        escape_n=0;
                    }
                    continue;
                }
                if(c==27){escape[0]=27;escape_n=1;continue;}
                if(c==3){done=1;break;}
                if(c==' ') {
                    if(!play) {
                        if(tui_start_audio(w,frames,&current_time,&audio)){play=1;strcpy(status,"Playing");}
                        else strcpy(status,"Playback unavailable: install ffplay (Linux) or use afplay (macOS)");
                    } else {current_time=tui_audio_position(&audio,duration);tui_stop_audio(&audio);play=0;strcpy(status,"Paused");}
                } else if(c=='\r'||c=='\n') {
                    if(tui_submit_command(w,cmd,status,sizeof(status))){done=1;break;}
                } else if(c==8||c==127) {size_t n=strlen(cmd);if(n)cmd[n-1]=0;}
                else if(c>=32&&c<127&&strlen(cmd)<sizeof(cmd)-2) {size_t n=strlen(cmd);cmd[n]=(char)c;cmd[n+1]=0;}
            }
        }
    }
    if(play||audio.path[0])tui_stop_audio(&audio);
    fputs("\x1b[?1006l\x1b[?1003l\x1b[?1000l\x1b[?25h\x1b[?1049l",stdout);
    fflush(stdout);
    tcsetattr(STDIN_FILENO,TCSANOW,&old_term);
    if(!strcmp(cmd,"get")) tui_print_results(w, ranges, nr);
    return 0;
}
#endif

static int bpm_main(int argc, char **argv) {
    Wave w; size_t n; double offset, *env, bpm, fit_residual=1e9, relative_error=1.0, bpm_error;
    int show_offset = 0, tui = 0, i, loaded = 0;
    const char *input = NULL;
    char temp_path[4096] = "";
    if (argc == 1) { print_help(argv[0]); return 0; }
    for (i = 1; i < argc; ++i) {
        if (!strcmp(argv[i], "--help") || !strcmp(argv[i], "--h") || !strcmp(argv[i], "-h")) {
            print_help(argv[0]); return 0;
        } else if (!strcmp(argv[i], "--tui")) {
            tui = 1;
        } else if (!strcmp(argv[i], "--offset") || !strcmp(argv[i], "-offset") ||
                   !strcmp(argv[i], "--o") || !strcmp(argv[i], "-o") || !strcmp(argv[i], "o")) {
            show_offset = 1;
        } else if (argv[i][0] == '-') {
            fprintf(stderr, "Unknown option: %s\n", argv[i]); print_help(argv[0]); return 2;
        } else if (!input) {
            input = argv[i];
        } else {
            fprintf(stderr, "Error: only one audio file can be analyzed at a time.\n"); return 2;
        }
    }
    if (!input && tui) {
        static char pathbuf[4096];
        printf("Enter audio file path: "); fflush(stdout);
#ifdef _WIN32
        {
            HANDLE console = GetStdHandle(STD_INPUT_HANDLE);
            DWORD mode, count;
            if (GetConsoleMode(console, &mode)) {
                wchar_t wide_path[4096];
                char *utf8_path;
                if (!ReadConsoleW(console, wide_path, 4095, &count, NULL)) return 2;
                wide_path[count] = L'\0';
                utf8_path = utf8_from_wide(wide_path);
                if (!utf8_path || strlen(utf8_path) >= sizeof(pathbuf)) {
                    free(utf8_path);
                    return 2;
                }
                strcpy(pathbuf, utf8_path);
                free(utf8_path);
            } else if (!fgets(pathbuf, sizeof(pathbuf), stdin)) return 2;
        }
#else
        if (!fgets(pathbuf, sizeof(pathbuf), stdin)) return 2;
#endif
        pathbuf[strcspn(pathbuf,"\r\n")]=0;
        if(pathbuf[0]=='"'){size_t len=strlen(pathbuf);if(len>1&&pathbuf[len-1]=='"'){pathbuf[len-1]=0;memmove(pathbuf,pathbuf+1,len-1);}}
        if(!pathbuf[0])return 2;
        input=pathbuf;
    }
    if (!input) { print_help(argv[0]); return 2; }
    loaded = read_wave(input, &w);
    if (loaded && coefficient_bank(w.rate) < 0) { free(w.data); w.data=NULL; loaded=0; }
    if (!loaded) {
        if (ffmpeg_decode(input, temp_path, sizeof(temp_path))) loaded = read_wave(temp_path, &w);
        if (!loaded) {
            remove_temp(temp_path);
            fprintf(stderr, "Error: cannot read this audio; check the path and ffmpeg availability.\n");
            return 2;
        }
    }
    if(tui && !w.size) {
        free(w.data); remove_temp(temp_path);
        fprintf(stderr,"Error: cannot display an empty audio file.\n"); return 2;
    }
    if(tui) {
        int tr=run_tui(&w,input); free(w.data); remove_temp(temp_path); return tr<0?2:0;
    }
    env = make_envelope(&w, &n, &offset);
    free(w.data);
    remove_temp(temp_path);
    if (!env) { fprintf(stderr, "Error: audio is silent or shorter than 2 seconds.\n"); return 1; }
    if(show_offset)printf("Offset: %.1f ms\n",offset);
    bpm = estimate_bpm(env, n, &fit_residual, &relative_error);
    if (!(bpm > 0.0) || !isfinite(bpm)) {
        fprintf(stderr, "Error: unable to estimate BPM; the offset is still available above if requested.\n"); free(env); return 1;
    }
    bpm_error=bpm*relative_error;
    if(relative_error<=0.00005) bpm=native_grid_quantize(bpm,bpm_error);
    printf("%.1f BPM\n", bpm);
    if (relative_error>0.00005 || fluctuates(env, n, bpm))
        printf("Warning: BPM varies or the fit is unstable; this result may be inaccurate.\n");
    free(env); return 0;
}

#ifdef _WIN32
int main(void) {
    int argc, i, result;
    LPWSTR *wide_argv = CommandLineToArgvW(GetCommandLineW(), &argc);
    char **argv;
    if (!wide_argv) return 2;
    argv = (char **)calloc((size_t)argc + 1, sizeof(char *));
    if (!argv) { LocalFree(wide_argv); return 2; }
    for (i = 0; i < argc; ++i) {
        argv[i] = utf8_from_wide(wide_argv[i]);
        if (!argv[i]) {
            while (i-- > 0) free(argv[i]);
            free(argv);
            LocalFree(wide_argv);
            return 2;
        }
    }
    LocalFree(wide_argv);
    result = bpm_main(argc, argv);
    for (i = 0; i < argc; ++i) free(argv[i]);
    free(argv);
    return result;
}
#else
int main(int argc, char **argv) { return bpm_main(argc, argv); }
#endif
