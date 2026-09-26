#ifndef _WIN32
#define _POSIX_C_SOURCE 200809L
#endif
/*
 * Standalone BPM and leading-silence analyser.
 * Build: Windows gcc -O2 bpm.c -lm -lwinmm -o bpm.exe
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
    double start, end, bpm, offset;
    int complete, measured;
} MeasureRange;

static uint16_t u16le(const unsigned char *p) { return (uint16_t)(p[0] | ((uint16_t)p[1] << 8)); }
static uint32_t u32le(const unsigned char *p) { return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24); }

static int read_wave(const char *path, Wave *w) {
    FILE *f = fopen(path, "rb");
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

/* Ask ffmpeg to decode anything this small WAV reader cannot handle. */
static int ffmpeg_decode(const char *input, char *output, size_t output_cap) {
#ifdef _WIN32
    char temp_dir[MAX_PATH], temp_path[MAX_PATH], command[32768];
    STARTUPINFOA si; PROCESS_INFORMATION pi; size_t pos = 0;
    DWORD n = GetTempPathA(MAX_PATH, temp_dir);
    if (!n || n >= MAX_PATH || !GetTempFileNameA(temp_dir, "bpm", 0, temp_path)) return 0;
    if (strlen(temp_path) + 1 > output_cap) { DeleteFileA(temp_path); return 0; }
    strcpy(output, temp_path);
    /* Quote every argument using the Windows command-line escaping rules. */
    command[0] = '\0';
    {
        const char *args[13] = { "ffmpeg", "-nostdin", "-v", "error", "-y", "-i", input,
                                 "-ac", "2", "-ar", "44100", "-c:a", "pcm_s16le" };
        size_t i;
        for (i = 0; i < 13; ++i) {
            const char *p = args[i]; size_t slashes = 0;
            if (pos + 3 >= sizeof(command)) goto fail_win;
            command[pos++] = '"';
            while (*p) {
                if (*p == '\\') { ++slashes; ++p; continue; }
                if (*p == '"') { while (slashes--) { command[pos++] = '\\'; command[pos++] = '\\'; } command[pos++] = '\\'; command[pos++] = '"'; }
                else { while (slashes--) { command[pos++] = '\\'; } command[pos++] = *p; }
                slashes = 0; ++p;
                if (pos + 4 >= sizeof(command)) goto fail_win;
            }
            while (slashes--) { command[pos++] = '\\'; command[pos++] = '\\'; }
            command[pos++] = '"'; command[pos++] = ' ';
        }
        command[pos] = '\0';
        /* The WAV muxer is explicit because the temporary file has no .wav suffix. */
        {
            const char *tail[3] = { "-f", "wav", output };
            for (i = 0; i < 3; ++i) {
                const char *p = tail[i]; command[pos++] = '"';
                while (*p && pos + 4 < sizeof(command)) {
                    if (*p == '"') command[pos++] = '\\';
                    command[pos++] = *p++;
                }
                command[pos++] = '"'; command[pos++] = ' ';
            }
            command[pos] = '\0';
        }
    }
    memset(&si, 0, sizeof(si)); memset(&pi, 0, sizeof(pi)); si.cb = sizeof(si); si.dwFlags = STARTF_USESHOWWINDOW; si.wShowWindow = SW_HIDE;
    if (!CreateProcessA(NULL, command, NULL, NULL, FALSE, CREATE_NO_WINDOW, NULL, NULL, &si, &pi)) goto fail_win;
    WaitForSingleObject(pi.hProcess, INFINITE);
    { DWORD code = 1; GetExitCodeProcess(pi.hProcess, &code); CloseHandle(pi.hThread); CloseHandle(pi.hProcess); if (code == 0) return 1; }
fail_win:
    DeleteFileA(temp_path); output[0] = '\0'; return 0;
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
    if (path && *path) DeleteFileA(path);
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
    unsigned s;
    for (s = 0; s < count; ++s) {
        const double *c = BIQUAD[bank][first + s];
        /* Stored denominator coefficients have their sign bit complemented. */
        float b0=(float)c[0], b1=(float)c[1], a1=-(float)c[2], a2=-(float)c[3], b2=(float)c[4];
        float state1=0.0f, state2=0.0f;
        size_t i;
        for (i=0; i<n; ++i) {
            float v = x[i];
            float state = state1*a1 + b2*v + state2*a2;
            x[i] = state1*b0 + state + state2*b1;
            state2=state1; state1=state;
        }
    }
}

static unsigned float_histogram_bin(float value) {
    union { float f; uint32_t u; } bits;
    bits.f = fabsf(value);
    return (unsigned)((bits.u + 0x40000u) >> 19);
}

/* Port of Convert_Audio_Samples' four band filters, median thresholding,
   weighted accumulation, 1 kHz reduction, and final smoothing. */
static double *make_envelope(const Wave *w, size_t *out_n, double *offset_ms) {
    static const unsigned starts[4] = {0,2,6,10}, counts[4] = {2,4,4,2};
    static const int shifts[4] = {-320,-64,-32,0};
    static const float weights[4] = {.3f,.2f,.2f,.3f};
    unsigned bytes=w->bits/8;
    size_t frames=w->size/(bytes*w->channels), nms=(frames*1000u+w->rate-1u)/w->rate;
    int bank=coefficient_bank(w->rate);
    float *mono, *band, *score;
    double *rms, *env;
    size_t i,k;
    double raw_peak=0.0;
    if (bank < 0 || frames < 2000 || nms < 2000) return NULL;
    mono=(float*)malloc(frames*sizeof(float)); band=(float*)malloc(frames*sizeof(float));
    score=(float*)calloc(frames,sizeof(float)); rms=(double*)calloc(nms,sizeof(double));
    env=(double*)calloc(nms,sizeof(double));
    if(!mono||!band||!score||!rms||!env) { free(mono);free(band);free(score);free(rms);free(env);return NULL; }
    for(i=0;i<frames;++i) {
        double v=0.0;
        for(k=0;k<w->channels;++k) v+=sample_at(w->data+(i*w->channels+k)*bytes,w->format,w->bits);
        mono[i]=(float)(v/w->channels);
        k=i*1000u/w->rate;
        if(k<nms) rms[k]+=(double)mono[i]*mono[i];
    }
    for(k=0;k<nms;++k) {
        size_t f0=k*w->rate/1000u,f1=(k+1)*w->rate/1000u,count=f1>f0?f1-f0:1;
        rms[k]=sqrt(rms[k]/count); if(rms[k]>raw_peak)raw_peak=rms[k];
    }
    if(raw_peak<=1e-9) { free(mono);free(band);free(score);free(rms);free(env);return NULL; }
    *offset_ms=0.0;
    for(k=0;k<nms;++k)if(rms[k]>raw_peak*.02){*offset_ms=(double)k;break;}

    for(unsigned b=0;b<4;++b) {
        uint32_t hist[4097]={0};
        unsigned median=0;
        double cumulative=0.0;
        memcpy(band,mono,frames*sizeof(float));
        run_biquads(band,frames,bank,starts[b],counts[b]);
        for(i=0;i<frames;++i)band[i]*=band[i];
        if(b==0)run_biquads(band,frames,bank,12,1);
        for(i=0;i<frames;++i){unsigned h=float_histogram_bin(band[i]);if(h>4096)h=4096;++hist[h];}
        for(median=0;median<4096 && cumulative<(double)(frames>>1);++median)cumulative+=hist[median];
        if(median>4096)median=4096;
        { uint32_t raw=median<<19; float threshold; float scale,weight;
          memcpy(&threshold,&raw,sizeof(threshold));
          if(!(threshold>0.0f))threshold=1e-30f;
          scale=2.0f/threshold; weight=weights[b]*8.262958317573066e-8f;
          for(i=0;i<frames;++i){
              size_t advance=(size_t)(shifts[b]<0?-shifts[b]:0);
              size_t src=i+advance;
              if(src>=frames)continue;
              { float q=(float)((int)(band[src]*scale+1.0f)-1); score[i]+=q*weight; }
          }
        }
    }
    run_biquads(score,frames,bank,13,2);
    /* Native code picks the nearest source sample at each millisecond. */
    for(k=0;k<nms;++k){size_t src=(size_t)llround((double)k*w->rate/1000.0); if(src<frames)env[k]=score[src];}
    {
      float *forward=(float*)malloc(nms*sizeof(float));
      float *reverse=(float*)malloc(nms*sizeof(float));
      double *difference=(double*)calloc(nms,sizeof(double));
      if(!forward||!reverse||!difference){free(forward);free(reverse);free(difference);free(mono);free(band);free(score);free(rms);free(env);return NULL;}
      for(k=0;k<nms;++k){forward[k]=(float)env[k];reverse[k]=(float)env[nms-1-k];}
      run_biquads(forward,nms,bank,15,1); run_biquads(reverse,nms,bank,15,1);
      if(nms>1){
          difference[nms-1]=-forward[nms-2];
          for(k=1;k<nms-1;++k)difference[k]=(double)reverse[nms-2-k]-forward[k-1];
          difference[0]=reverse[nms-2];
      }
      memcpy(env,difference,nms*sizeof(double));
      free(forward);free(reverse);free(difference);
    }
    { double mean=0.0; for(k=0;k<nms;++k)mean+=env[k]; mean/=nms; for(k=0;k<nms;++k)env[k]-=mean; }
    free(mono);free(band);free(score);free(rms);*out_n=nms;return env;
}

static double fit_peak_period(const double *env, size_t n, double initial_period,
                              double *residual_ms, double *relative_period_error) {
    size_t period = (size_t)llround(initial_period), phase, k;
    double best_score = -1.0, best_phase = 0.0;
    double sx = 0.0, sy = 0.0, sxx = 0.0, sxy = 0.0, syy = 0.0;
    size_t count = 0;
    if (period < 200 || period > 2000) return initial_period;
    /* Native code aligns a beat grid to local maxima in a +/-10 ms window. */
    for (phase = 0; phase < period; ++phase) {
        double score = 0.0;
        for (k = phase; k < n; k += period) {
            size_t lo = k > 10 ? k - 10 : 0, hi = k + 10 < n ? k + 10 : n - 1, j, best = lo;
            for (j = lo + 1; j <= hi; ++j) if (env[j] > env[best]) best = j;
            score += env[best];
        }
        if (score > best_score) { best_score = score; best_phase = (double)phase; }
    }
    for (k = 0; best_phase + k * period < n; ++k) {
        double center = best_phase + k * period;
        size_t lo = center > 10 ? (size_t)center - 10 : 0;
        size_t hi = (size_t)center + 10 < n ? (size_t)center + 10 : n - 1, j, best = lo;
        for (j = lo + 1; j <= hi; ++j) if (env[j] > env[best]) best = j;
        if (env[best] > 0.0) {
            double x = (double)k, y = (double)best;
            sx += x; sy += y; sxx += x * x; sxy += x * y; syy += y * y; ++count;
        }
    }
    if (count < 4 || count * sxx <= sx * sx) {
        if (residual_ms) *residual_ms=1e9;
        if (relative_period_error) *relative_period_error=1.0;
        return initial_period;
    }
    {
        double sxx_centered=sxx-sx*sx/count, sxy_centered=sxy-sx*sy/count;
        double syy_centered=syy-sy*sy/count;
        double slope=sxy_centered/sxx_centered;
        double intercept=(sy-slope*sx)/count;
        double sse=syy_centered-sxy_centered*sxy_centered/sxx_centered;
        double sigma=sqrt(fmax(0.0,sse)/(count-2));
        double slope_se=sigma/sqrt(sxx_centered);
        if(residual_ms)*residual_ms=sqrt(fmax(0.0,sse)/(count-1));
        if(relative_period_error)*relative_period_error=slope>0.0?slope_se/slope:1.0;
        (void)intercept;
        return slope>0.0?slope:initial_period;
    }
}

static double estimate_bpm(const double *env, size_t n, double *fit_residual, double *relative_error) {
    size_t fft_n = 1, i, lag, best = 0;
    Complex *a; double best_score = -1e300;
    while (fft_n < n * 2) { if (fft_n > ((size_t)-1) / 2) return 0; fft_n <<= 1; }
    a = (Complex *)calloc(fft_n, sizeof(Complex)); if (!a) return 0;
    for (i = 0; i < n; ++i) a[i].re = env[i];
    fft(a, fft_n, 0);
    for (i = 0; i < fft_n; ++i) { a[i].re = a[i].re * a[i].re + a[i].im * a[i].im; a[i].im = 0.0; }
    fft(a, fft_n, 1);
    /* Find repeated autocorrelation peaks and prefer a period supported by at
       least four harmonics, as Auto_Timing_Analyze does. */
    for (lag = 200; lag <= 2000 && lag < n / 2; ++lag) {
        double peak=a[lag].re, sum=0.0, average;
        size_t j, harmonic, hits=0;
        int local=1;
        for(j=lag>16?lag-16:1;j<=lag+16 && j<fft_n/2;++j)
            if(j!=lag && a[j].re>peak){local=0;break;}
        if(!local)continue;
        for(harmonic=1;harmonic*lag<=2000 && harmonic*lag<n/2;++harmonic){
            size_t center=harmonic*lag, lo=center>10?center-10:1, hi=center+10;
            size_t q, found=center; double value=-1e300;
            if(hi>=n/2)hi=n/2-1;
            for(q=lo;q<=hi;++q)if(a[q].re>=a[q-1].re && a[q].re>=a[q+1].re && a[q].re>value){value=a[q].re;found=q;}
            if(value>-1e299 && (harmonic==1 || (found>harmonic*lag-11 && found<harmonic*lag+11))){sum+=value;++hits;}
        }
        if(hits<4)continue;
        average=sum/(double)hits;
        if(peak*.7>=average)continue;
        if(average>best_score){best_score=average;best=lag;}
    }
    /* Fall back to the strongest single peak for short or sparse material. */
    if(!best)for(lag=200;lag<=2000 && lag<n/2;++lag)if(a[lag].re>best_score){best_score=a[lag].re;best=lag;}
    if (!best) { free(a); return 0; }
    /* Parabolic sub-millisecond peak interpolation. */
    { double l = a[best - 1].re, c = a[best].re, r = a[best + 1].re;
      double den = l - 2.0 * c + r, shift = fabs(den) > 1e-20 ? 0.5 * (l - r) / den : 0.0;
      double period = best + fmax(-0.5, fmin(0.5, shift)); free(a);
      period = fit_peak_period(env, n, period, fit_residual, relative_error);
      return 60000.0 / period; }
}

static int fluctuates(const double *env, size_t n, double whole) {
    size_t win = 16000, step = 8000, start; double lo = 1e9, hi = 0.0; int count = 0;
    if (n < win * 2) return 0;
    for (start = 0; start + win <= n; start += step) {
        double bpm = estimate_bpm(env + start, win, NULL, NULL);
        if (bpm > 0) {
            double aligned=bpm, distance=fabs(bpm-whole);
            const double factors[4]={.5,1.0,2.0,3.0};
            size_t f;
            for(f=0;f<4;++f){double candidate=bpm*factors[f],d=fabs(candidate-whole);if(d<distance){distance=d;aligned=candidate;}}
            if (aligned < lo) lo = aligned;
            if (aligned > hi) hi = aligned;
            ++count;
        }
    }
    (void)whole;
    return count >= 2 && hi - lo > fmax(3.0, whole * 0.03);
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

#ifdef _WIN32
#define MAX_RANGES 128
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

static int run_tui(Wave *w, const char *name) {
    HANDLE hin=GetStdHandle(STD_INPUT_HANDLE), hout=GetStdHandle(STD_OUTPUT_HANDLE);
    DWORD oldmode=0, mode, oldout=0, last_buttons=0; UINT oldcp; CONSOLE_SCREEN_BUFFER_INFO cs; INPUT_RECORD ev; DWORD got;
    MeasureRange ranges[MAX_RANGES]; int nr=0,pending=0,play=0,mouse_x=40,redraw=1,last_range_count=-1,last_graph_h=-1,last_height=-1; double pending_t=0,span,current_time=0;
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
        int width=80,height=25,x,y,graph_h,shown_ranges,range_count,max_range_rows; double cursor=current_time;
        GetConsoleScreenBufferInfo(hout,&cs); width=cs.srWindow.Right-cs.srWindow.Left+1;height=cs.srWindow.Bottom-cs.srWindow.Top+1;
        if(width<20)width=20;
        if(width>2000)width=2000;
        if(width>20)width--;
        if(height<8)height=8;
        if(current_time<0)current_time=0;
        if(current_time>duration)current_time=duration;
        cursor=current_time;
        range_count=nr+(pending?1:0);
        max_range_rows=height-11;if(max_range_rows<0)max_range_rows=0;
        if(range_count>max_range_rows)range_count=max_range_rows;
        shown_ranges=nr<range_count?nr:range_count;
        graph_h=(height-5-range_count)/2;
        if(graph_h<3)graph_h=3;
        if(redraw||play){
        if(range_count!=last_range_count||graph_h!=last_graph_h||height!=last_height){
            for(y=2;y<=height-3;++y)tui_write_at(hout,y,"");
            last_range_count=range_count;last_graph_h=graph_h;last_height=height;
        }
        {char line[512];const char *shown=name?name:"audio",*base1=strrchr(shown,'\\'),*base2=strrchr(shown,'/');if(base1&&(!base2||base1>base2))shown=base1+1;else if(base2)shown=base2+1;snprintf(line,sizeof(line),"%s  Time: %.2f / %.2f sec   ranges:%d",shown,current_time,duration,nr);line[width<511?width:511]=0;tui_write_at(hout,1,line);}
        {char row[2048];memset(row,'-',(size_t)width);row[0]='+';row[width-1]='+';row[width]=0;tui_write_at(hout,2,row);}
        {int amps[2048],tops[2048],bots[2048],best_bins[2048];
            for(x=1;x<width-1;++x){double t=current_time+((double)x-(double)(width/2))*span/(width-3),bin_seconds=span/(width-3),sum_sq=0.0,sub_peak[3]={0,0,0};size_t f0,f1,j,count=0;int amp,top,bot,best=0;
                {double left=(t-bin_seconds*.5)*w->rate,right=(t+bin_seconds*.5)*w->rate;long long lo=(long long)floor(left),hi=(long long)ceil(right);if(lo<0)lo=0;if(lo>(long long)frames)lo=(long long)frames;if(hi<0)hi=0;if(hi>(long long)frames)hi=(long long)frames;f0=(size_t)lo;f1=(size_t)hi;}
                for(j=f0;j<f1;++j){double v=0.0;unsigned c;size_t seg=((j-f0)*3)/(f1-f0);if(seg>2)seg=2;for(c=0;c<w->channels;++c)v+=sample_at(w->data+(j*w->channels+c)*(w->bits/8),w->format,w->bits);v/=w->channels;sum_sq+=v*v;if(fabs(v)>sub_peak[seg])sub_peak[seg]=fabs(v);++count;}
                if(sub_peak[1]>sub_peak[best])best=1;
                if(sub_peak[2]>sub_peak[best])best=2;
                {double rms=count?sqrt(sum_sq/count):0.0;amp=(int)(rms*(graph_h-2)*4.0);}
                top=graph_h/2-amp/2;bot=graph_h/2+amp/2;if(top<0)top=0;if(bot>=graph_h)bot=graph_h-1;amps[x]=amp;tops[x]=top;bots[x]=bot;best_bins[x]=best;
            }
            for(y=0;y<graph_h;++y){char row[8192];size_t used=0;row[used++]='|';
                for(x=1;x<width-1;++x){if(x==width/2)row[used++]='#';else if(y>=tops[x]&&y<=bots[x]&&amps[x]>0){static const char *glyphs[3]={"\xE2\x96\x8F","\xE2\x94\x82","\xE2\x96\x95"};const char *g=glyphs[best_bins[x]];while(*g)row[used++]=*g++;}else row[used++]=' ';}
                row[used++]='|';row[used]=0;tui_write_at(hout,y+3,row);
            }
        }
        {char row[2048];memset(row,'-',(size_t)width);row[0]='+';row[width-1]='+';row[width]=0;tui_write_at(hout,graph_h+3,row);}
        for(y=0;y<shown_ranges;y++){
            char row[2048],label[24];
            int a=width/2+(int)((ranges[y].start-cursor)*(width-3)/span);
            int b=width/2+(int)((ranges[y].end-cursor)*(width-3)/span);
            memset(row,' ',(size_t)width);
            if(b>=1&&a<width-1){
                if(a<1)a=1;
                if(b>=width-1)b=width-2;
                if(a<=b){
                    int label_start,label_len;
                    row[a]='[';row[b]=']';
                    for(x=a+1;x<b;x++)row[x]='-';
                    snprintf(label,sizeof(label),"<%d>",y+1);
                    label_len=(int)strlen(label);
                    label_start=a+1+(b-a-1-label_len)/2;
                    if(label_start>a&&label_start+label_len<b)
                        memcpy(row+label_start,label,(size_t)label_len);
                }
            }
            row[width]=0;
            tui_write_at(hout,graph_h+4+y,row);
        }
        if(pending&&range_count>shown_ranges){char row[2048];int a=width/2+(int)((pending_t-cursor)*(width-3)/span),b=mouse_x;memset(row,' ',(size_t)width);if(a<1)a=1;if(a>width-2)a=width-2;if(b<1)b=1;if(b>width-2)b=width-2;row[a]='[';if(a<b)for(x=a+1;x<=b;x++)row[x]='-';else for(x=b;x<a;x++)row[x]='-';row[width]=0;tui_write_at(hout,graph_h+4+shown_ranges,row);}
        if(height>=9)tui_write_at(hout,height-2,"Ctrl+C exit | Space play/pause | Wheel seek (stops playback)");
        {char msg[512];snprintf(msg,sizeof(msg),"Left click x2 range | Right click delete | get+Enter results | %s",status);msg[width<511?width:511]=0;tui_write_at(hout,height-1,msg);}
        {char msg[160];snprintf(msg,sizeof(msg),"Command: %s",cmd);msg[width<159?width:159]=0;tui_write_at(hout,height,msg);}
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
            else if(c=='\r'){if(!strcmp(cmd,"get")){break;}else if(!strcmp(cmd,"offset")){double *e;size_t en;double off;if((e=make_envelope(w,&en,&off))!=NULL){snprintf(status,sizeof(status),"Offset: %.1f ms",off);free(e);}}cmd[0]=0;}
            else if(c==8){size_t n=strlen(cmd);if(n)cmd[n-1]=0;}
            else if(c>=32&&strlen(cmd)<sizeof(cmd)-2){size_t n=strlen(cmd);cmd[n]=c;cmd[n+1]=0;}
        } else if(ev.EventType==MOUSE_EVENT){MOUSE_EVENT_RECORD m=ev.Event.MouseEvent;
            int mx=m.dwMousePosition.X-cs.srWindow.Left,my=m.dwMousePosition.Y-cs.srWindow.Top;
            if(mx<1)mx=1;
            if(mx>width-2)mx=width-2;
            mouse_x=mx;
            if(m.dwEventFlags==MOUSE_WHEELED){short d=(short)HIWORD(m.dwButtonState);if(play&&out){current_time=tui_audio_position(out,playback_base,w->rate,w->channels,current_time);stop_tui_audio(&out,&hdr,&pcm,&play);strcpy(status,"Playback stopped");}current_time+=(double)d/120.0*span*.12;if(current_time<0)current_time=0;if(current_time>duration)current_time=duration;}
            else {
                double t=current_time+((double)mx-(double)(width/2))*span/(width-3);
                DWORD buttons=m.dwButtonState;
                if(t<0)t=0;
                if(t>duration)t=duration;
                if((buttons&FROM_LEFT_1ST_BUTTON_PRESSED)&&!(last_buttons&FROM_LEFT_1ST_BUTTON_PRESSED)&&my>=2&&my<=graph_h+1){
                    if(!pending){pending_t=t;pending=1;strcpy(status,"Start selected; click end");}
                    else if(nr<MAX_RANGES){ranges[nr].start=fmin(pending_t,t);ranges[nr].end=fmax(pending_t,t);ranges[nr].complete=1;nr++;pending=0;strcpy(status,"Range added");}
                }
                if((buttons&RIGHTMOST_BUTTON_PRESSED)&&!(last_buttons&RIGHTMOST_BUTTON_PRESSED)){
                    int r;for(r=0;r<nr;r++)if(t>=ranges[r].start&&t<=ranges[r].end&&((my>=2&&my<=graph_h+1)||my==graph_h+3+r)){
                        memmove(&ranges[r],&ranges[r+1],(size_t)(nr-r-1)*sizeof(ranges[0]));nr--;strcpy(status,"Range deleted");break;
                    }
                }
                last_buttons=buttons;
            }
        }
    }
    if(play&&out)stop_tui_audio(&out,&hdr,&pcm,&play);
    tui_write(hout,"\x1b[0m\x1b[?25h\x1b[2J\x1b[H");
    SetConsoleMode(hin,oldmode);SetConsoleMode(hout,oldout);SetConsoleOutputCP(oldcp);
    SetConsoleCtrlHandler(tui_ctrl_handler,FALSE);
    if(!strcmp(cmd,"get")){int r;for(r=0;r<nr;r++){size_t b=(size_t)(ranges[r].start*w->rate),e=(size_t)(ranges[r].end*w->rate),bytes=(w->bits/8)*w->channels,n;Wave part=*w;double *env,bpm=0,off=0,err=1,res=0;part.data=w->data+b*bytes;part.size=(e-b)*bytes;if(e>b&&(env=make_envelope(&part,&n,&off))!=NULL){bpm=estimate_bpm(env,n,&res,&err);if(bpm>0&&err<=.00005)bpm=native_grid_quantize(bpm,bpm*err);free(env);}printf("%d.BPM:%.1f,offset:%.1f ms;\n",r+1,bpm,off);}}
    return 0;
}
#else
#define MAX_RANGES 128

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

static int run_tui(Wave *w, const char *name) {
    struct termios old_term, raw_term;
    MeasureRange ranges[MAX_RANGES];
    PosixPlayback audio = {0};
    int nr=0, pending=0, play=0, mouse_x=40, redraw=1;
    int last_range_count=-1, last_graph_h=-1, last_height=-1, done=0;
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
        int width=80,height=25,x,y,graph_h,shown_ranges,range_count,max_range_rows;
        double cursor;
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
        if(width<20)width=20;
        if(width>2000)width=2000;
        if(width>20)width--;
        if(height<8)height=8;
        if(current_time<0)current_time=0;
        if(current_time>duration)current_time=duration;
        cursor=current_time;
        range_count=nr+(pending?1:0);
        max_range_rows=height-11;if(max_range_rows<0)max_range_rows=0;
        if(range_count>max_range_rows)range_count=max_range_rows;
        shown_ranges=nr<range_count?nr:range_count;
        graph_h=(height-5-range_count)/2;
        if(graph_h<3)graph_h=3;
        if(redraw||play) {
            if(range_count!=last_range_count||graph_h!=last_graph_h||height!=last_height) {
                for(y=2;y<=height-3;++y)tui_write_at(y,"");
                last_range_count=range_count;last_graph_h=graph_h;last_height=height;
            }
            {char line[512];const char *shown=name?name:"audio",*b1=strrchr(shown,'/'),*b2=strrchr(shown,'\\');
                if(b1&&(!b2||b1>b2))shown=b1+1;else if(b2)shown=b2+1;
                snprintf(line,sizeof(line),"%s  Time: %.2f / %.2f sec   ranges:%d",shown,current_time,duration,nr);
                line[width<511?width:511]=0;tui_write_at(1,line);}
            {char row[2048];memset(row,'-',(size_t)width);row[0]='+';row[width-1]='+';row[width]=0;tui_write_at(2,row);}
            {int amps[2048],tops[2048],bots[2048],best_bins[2048];
                for(x=1;x<width-1;++x) {
                    double t=current_time+((double)x-(double)(width/2))*span/(width-3);
                    double bin_seconds=span/(width-3),sum_sq=0,sub_peak[3]={0,0,0};
                    size_t f0,f1,j,count=0;int amp,top,bot,best=0;
                    {double left=(t-bin_seconds*.5)*w->rate,right=(t+bin_seconds*.5)*w->rate;
                        long long lo=(long long)floor(left),hi=(long long)ceil(right);
                        if(lo<0)lo=0;if(lo>(long long)frames)lo=(long long)frames;
                        if(hi<0)hi=0;if(hi>(long long)frames)hi=(long long)frames;
                        f0=(size_t)lo;f1=(size_t)hi;}
                    for(j=f0;j<f1;++j) {
                        double v=0;unsigned c;size_t seg=((j-f0)*3)/(f1-f0);
                        if(seg>2)seg=2;
                        for(c=0;c<w->channels;++c)v+=sample_at(w->data+(j*w->channels+c)*(w->bits/8),w->format,w->bits);
                        v/=w->channels;sum_sq+=v*v;
                        if(fabs(v)>sub_peak[seg])sub_peak[seg]=fabs(v);++count;
                    }
                    if(sub_peak[1]>sub_peak[best])best=1;
                    if(sub_peak[2]>sub_peak[best])best=2;
                    {double rms=count?sqrt(sum_sq/count):0;amp=(int)(rms*(graph_h-2)*4.0);}
                    top=graph_h/2-amp/2;bot=graph_h/2+amp/2;
                    if(top<0)top=0;if(bot>=graph_h)bot=graph_h-1;
                    amps[x]=amp;tops[x]=top;bots[x]=bot;best_bins[x]=best;
                }
                for(y=0;y<graph_h;++y) {char row[8192];size_t used=0;row[used++]='|';
                    for(x=1;x<width-1;++x) {
                        if(x==width/2)row[used++]='#';
                        else if(y>=tops[x]&&y<=bots[x]&&amps[x]>0) {
                            static const char *glyphs[3]={"\xE2\x96\x8F","\xE2\x94\x82","\xE2\x96\x95"};
                            const char *g=glyphs[best_bins[x]];while(*g)row[used++]=*g++;
                        } else row[used++]=' ';
                    }
                    row[used++]='|';row[used]=0;tui_write_at(y+3,row);
                }
            }
            {char row[2048];memset(row,'-',(size_t)width);row[0]='+';row[width-1]='+';row[width]=0;tui_write_at(graph_h+3,row);}
            for(y=0;y<shown_ranges;y++) {
                char row[2048],label[24];
                int a=width/2+(int)((ranges[y].start-cursor)*(width-3)/span);
                int b=width/2+(int)((ranges[y].end-cursor)*(width-3)/span);
                memset(row,' ',(size_t)width);
                if(b>=1&&a<width-1) {
                    if(a<1)a=1;if(b>=width-1)b=width-2;
                    if(a<=b) {int label_start,label_len;
                        row[a]='[';row[b]=']';for(x=a+1;x<b;x++)row[x]='-';
                        snprintf(label,sizeof(label),"<%d>",y+1);label_len=(int)strlen(label);
                        label_start=a+1+(b-a-1-label_len)/2;
                        if(label_start>a&&label_start+label_len<b)memcpy(row+label_start,label,(size_t)label_len);
                    }
                }
                row[width]=0;tui_write_at(graph_h+4+y,row);
            }
            if(pending&&range_count>shown_ranges) {
                char row[2048];int a=width/2+(int)((pending_t-cursor)*(width-3)/span),b=mouse_x;
                memset(row,' ',(size_t)width);
                if(a<1)a=1;if(a>width-2)a=width-2;if(b<1)b=1;if(b>width-2)b=width-2;
                row[a]='[';
                if(a<b)for(x=a+1;x<=b;x++)row[x]='-';else for(x=b;x<a;x++)row[x]='-';
                row[width]=0;tui_write_at(graph_h+4+shown_ranges,row);
            }
            if(height>=9)tui_write_at(height-2,"Ctrl+C exit | Space play/pause | Wheel seek (stops playback)");
            {char msg[512];snprintf(msg,sizeof(msg),"Left click x2 range | Right click delete | get+Enter results | %s",status);
                msg[width<511?width:511]=0;tui_write_at(height-1,msg);}
            {char msg[160];snprintf(msg,sizeof(msg),"Command: %s",cmd);
                msg[width<159?width:159]=0;tui_write_at(height,msg);}
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
                            t=current_time+((double)click_x-(double)(width/2))*span/(width-3);
                            if(t<0)t=0;if(t>duration)t=duration;
                            if((button&64)&&end=='M') {
                                if(play){current_time=tui_audio_position(&audio,duration);tui_stop_audio(&audio);play=0;strcpy(status,"Playback stopped");}
                                current_time+=(button&1)?-span*.12:span*.12;
                                if(current_time<0)current_time=0;if(current_time>duration)current_time=duration;
                            } else if(end=='M'&&!(button&32)) {
                                if((button&3)==0&&click_y>=2&&click_y<=graph_h+1) {
                                    if(!pending){pending_t=t;pending=1;strcpy(status,"Start selected; click end");}
                                    else if(nr<MAX_RANGES){ranges[nr].start=fmin(pending_t,t);ranges[nr].end=fmax(pending_t,t);ranges[nr].complete=1;nr++;pending=0;strcpy(status,"Range added");}
                                } else if((button&3)==2) {
                                    int r;for(r=0;r<nr;r++)if(t>=ranges[r].start&&t<=ranges[r].end&&
                                        ((click_y>=2&&click_y<=graph_h+1)||click_y==graph_h+3+r)) {
                                        memmove(&ranges[r],&ranges[r+1],(size_t)(nr-r-1)*sizeof(ranges[0]));
                                        nr--;strcpy(status,"Range deleted");break;
                                    }
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
                    if(!strcmp(cmd,"get")){done=1;break;}
                    if(!strcmp(cmd,"offset")) {double *e;size_t en;double off;
                        if((e=make_envelope(w,&en,&off))!=NULL){snprintf(status,sizeof(status),"Offset: %.1f ms",off);free(e);}
                    }
                    cmd[0]=0;
                } else if(c==8||c==127) {size_t n=strlen(cmd);if(n)cmd[n-1]=0;}
                else if(c>=32&&c<127&&strlen(cmd)<sizeof(cmd)-2) {size_t n=strlen(cmd);cmd[n]=(char)c;cmd[n+1]=0;}
            }
        }
    }
    if(play||audio.path[0])tui_stop_audio(&audio);
    fputs("\x1b[?1006l\x1b[?1003l\x1b[?1000l\x1b[?25h\x1b[?1049l",stdout);
    fflush(stdout);
    tcsetattr(STDIN_FILENO,TCSANOW,&old_term);
    if(!strcmp(cmd,"get")) {int r;for(r=0;r<nr;r++) {
        size_t b=(size_t)(ranges[r].start*w->rate),e=(size_t)(ranges[r].end*w->rate),bytes=(w->bits/8)*w->channels,n;
        Wave part=*w;double *env,bpm=0,off=0,err=1,res=0;
        part.data=w->data+b*bytes;part.size=(e-b)*bytes;
        if(e>b&&(env=make_envelope(&part,&n,&off))!=NULL){bpm=estimate_bpm(env,n,&res,&err);
            if(bpm>0&&err<=.00005)bpm=native_grid_quantize(bpm,bpm*err);free(env);}
        printf("%d.BPM:%.1f,offset:%.1f ms;\n",r+1,bpm,off);
    }}
    return 0;
}
#endif

int main(int argc, char **argv) {
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
        if (!fgets(pathbuf, sizeof(pathbuf), stdin)) return 2;
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
            fprintf(stderr, "Error: cannot read this audio; install ffmpeg and ensure it is on PATH.\n");
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
