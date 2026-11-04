// Reads both I2S buses, then every packet (256 new samples per mic, 62.5 per second):
//   1. streams raw audio: "SYNC" + bus A block + bus B block (unchanged, for plot/scope/peak.py)
//   2. spectra: 1024-point FFT per mic
//   3. pitch: finds the fundamental f0 from all 4 mics combined
//   4. harmonics: for each harmonic of f0 inside BAND_LO..BAND_HI, compares the 4 mics' levels
//      and gives a direction; the directions are averaged, louder harmonics weighted more
//   5. streams "DIRV" + 4 amplitudes + x + y + f0 + harmonics used (little-endian float32), for vector.py
// No clear pitch, or fewer than MIN_HARM harmonics in the band -> f0 = 0, amplitudes and vector = 0.
// Peaks are judged against a local noise floor (median of each FLOOR_BLK-bin block).

#include <Arduino.h>
#include <driver/i2s.h>
#include <esp_dsp.h>
#include <math.h>
#include <algorithm>

static const int RATE = 16000;
static const int BLK = 2048;            // bytes per bus per packet (256 stereo frames)
static const int HOP = BLK / 8;         // new samples per mic per packet (256)
static const int NFFT = 1024;           // FFT length: 64 ms of audio
static const int NB = NFFT / 2 + 1;     // one-sided bins
static const float BIN_HZ = (float)RATE / NFFT;  // 15.6 Hz per bin

// ---- Tuning: set these from the directivity sweep and calibration ----
static const float BAND_LO = 2000;  // Hz, only harmonics in this band are used for direction
static const float BAND_HI = 6000;  // Hz
static const float GAIN[4] = {1, 1, 1, 1};         // per-mic gain correction
static const float AZ_DEG[4] = {60, 300, 120, 240};  // direction each mic faces around the vertical (mic 1..4)

// ---- Pitch / harmonic detection ----
static const float F0_MIN = 80;     // Hz, lowest pitch searched
static const int NSUB = 6;          // strongest peak is tried as harmonic 1..NSUB of the pitch
static const int HMAX = 80;         // highest harmonic number considered
static const float PEAK_DB = 10;    // a harmonic counts if it is this far above the local noise floor.
                                    // 6 = fewer dropouts but ~20% false detections on room noise (10: ~2%)
static const int HFIT = 10;         // pitch check uses the first HFIT predicted harmonics
static const float MIN_FRAC = 0.6f; // at least this fraction of them must be present (rejects f0/2)
static const int MIN_HARM = 3;      // fewer harmonics than this in the band -> no output
static const int FLOOR_BLK = 32;    // bins per noise-floor block
static const float WOBBLE = 0.02f;  // harmonic search window, fraction of its frequency (pitch wobble)
static const float FMAX = 0.45f * RATE;  // highest frequency used for pitch matching

static uint8_t bufA[BLK];
static uint8_t bufB[BLK];
static float hist[4][NFFT];      // last NFFT samples per mic, oldest first, full-scale units (1.0 = full scale)
// ESP-DSP's optimised FFT needs 16-byte aligned buffers; unaligned gave garbage (~+24 dBFS on every mic)
static float win[NFFT] __attribute__((aligned(16)));          // Hann window
static float fftbuf[2 * NFFT] __attribute__((aligned(16)));   // interleaved re, im
static float P[4][NB];           // power spectrum per mic, |X|^2
static float S[NB];              // sum of the 4 power spectra
static float thr[NB];            // per-bin detection threshold: local noise floor + PEAK_DB
static float tmp[FLOOR_BLK];     // scratch for the median
static float winPow;             // sum of win^2, for Parseval scaling
static float dirX[4], dirY[4];   // unit vectors each mic faces

struct DirPacket {
  char tag[4];
  float amp[4];  // RMS of the harmonics used, per mic, full-scale units
  float x, y;    // direction vector, length 0..1
  float f0;      // Hz, 0 = no pitch
  float nharm;   // number of harmonics used
};
static_assert(sizeof(DirPacket) == 36, "no padding: vector.py reads 4 + 8 floats");

static void startBus(i2s_port_t port, int sck, int ws, int sd) {
  i2s_config_t cfg = {};
  cfg.mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX);
  cfg.sample_rate = RATE;
  cfg.bits_per_sample = I2S_BITS_PER_SAMPLE_32BIT;
  cfg.channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT;  // stereo
  cfg.communication_format = I2S_COMM_FORMAT_STAND_I2S;
  cfg.dma_buf_count = 8;
  cfg.dma_buf_len = 256;  // frames per DMA buffer; 8 x 256 frames = 128 ms of slack
  i2s_driver_install(port, &cfg, 0, nullptr);

  i2s_pin_config_t pins = {};
  pins.mck_io_num = I2S_PIN_NO_CHANGE;  // no MCLK (0 would put it on GPIO0)
  pins.bck_io_num = sck;
  pins.ws_io_num = ws;
  pins.data_out_num = I2S_PIN_NO_CHANGE;
  pins.data_in_num = sd;
  i2s_set_pin(port, &pins);
}

// 1. Shift each mic's history left by HOP and append the new samples
static void appendSamples() {
  const int32_t *a = (const int32_t *)bufA;  // frames: [left = mic 2, right = mic 1]
  const int32_t *b = (const int32_t *)bufB;  // frames: [left = mic 4, right = mic 3]
  for (int m = 0; m < 4; m++) memmove(hist[m], hist[m] + HOP, (NFFT - HOP) * sizeof(float));
  for (int f = 0; f < HOP; f++) {
    int j = NFFT - HOP + f;
    hist[0][j] = (a[2 * f + 1] >> 8) / 8388608.0f;  // 24-bit left-justified -> full-scale units
    hist[1][j] = (a[2 * f] >> 8) / 8388608.0f;
    hist[2][j] = (b[2 * f + 1] >> 8) / 8388608.0f;
    hist[3][j] = (b[2 * f] >> 8) / 8388608.0f;
  }
}

// 2. Power spectrum of each mic into P, and their sum into S
static void spectra() {
  for (int k = 0; k < NB; k++) S[k] = 0;
  for (int m = 0; m < 4; m++) {
    float mean = 0;
    for (int n = 0; n < NFFT; n++) mean += hist[m][n];
    mean /= NFFT;
    for (int n = 0; n < NFFT; n++) {
      fftbuf[2 * n] = (hist[m][n] - mean) * win[n];
      fftbuf[2 * n + 1] = 0;
    }
    dsps_fft2r_fc32(fftbuf, NFFT);
    dsps_bit_rev_fc32(fftbuf, NFFT);
    for (int k = 0; k < NB; k++) {
      P[m][k] = fftbuf[2 * k] * fftbuf[2 * k] + fftbuf[2 * k + 1] * fftbuf[2 * k + 1];
      S[k] += P[m][k];
    }
  }
}

// Detection threshold per bin: median of its FLOOR_BLK-bin block, + PEAK_DB
static void noiseFloor() {
  float g = powf(10, PEAK_DB / 10);
  for (int s = 0; s < NB; s += FLOOR_BLK) {
    int len = std::min(FLOOR_BLK, NB - s);
    for (int i = 0; i < len; i++) tmp[i] = S[s + i];
    std::nth_element(tmp, tmp + len / 2, tmp + len);
    for (int i = 0; i < len; i++) thr[s + i] = tmp[len / 2] * g;
  }
}

// Search window (bins) around harmonic frequency f: allows pitch wobble, but stays under
// a quarter of the harmonic spacing so it can't land on the neighbouring harmonic
static int tolBins(float f, float f0) {
  return std::max(1, std::min((int)ceilf(WOBBLE * f / BIN_HZ), (int)(0.25f * f0 / BIN_HZ)));
}

// Bin with the most power in S within tol bins of frequency f, or -1
static int localMax(float f, int tol) {
  int c = lroundf(f / BIN_HZ);
  int lo = std::max(1, c - tol), hi = std::min(NB - 2, c + tol);
  if (lo > hi) return -1;
  int best = lo;
  for (int k = lo + 1; k <= hi; k++)
    if (S[k] > S[best]) best = k;
  return best;
}

// 3. Pitch: the loudest peak is some harmonic n of f0. Try n = 1..NSUB; keep the f0 whose first
// HFIT predicted harmonics hold the most energy, scaled by the fraction actually present.
// Candidates with under MIN_FRAC present are rejected (stops f0/2, f0/3 winning)
static float findPitch() {
  int kmin = (int)ceilf(F0_MIN / BIN_HZ), kmax = (int)(BAND_HI / BIN_HZ);
  int k = kmin;  // loudest peak (low harmonics are loudest in natural sounds)
  for (int i = kmin + 1; i <= kmax; i++)
    if (S[i] > S[k]) k = i;
  if (S[k] < thr[k]) return 0;

  float a = logf(S[k - 1] + 1e-30f), b = logf(S[k] + 1e-30f), c = logf(S[k + 1] + 1e-30f);
  float den = a - 2 * b + c;
  float fpk = (k + (den != 0 ? 0.5f * (a - c) / den : 0)) * BIN_HZ;  // parabolic interpolation

  float best = 0, bestScore = 0;
  for (int n = 1; n <= NSUB; n++) {
    float f0 = fpk / n;
    if (f0 < F0_MIN) break;
    float e = 0;
    int pred = 0, hit = 0;
    for (int h = 1; h <= HMAX; h++) {
      float f = h * f0;
      if (f > FMAX || h > HFIT) break;
      pred++;
      int kk = localMax(f, tolBins(f, f0));
      if (kk >= 0 && S[kk] > thr[kk]) {
        hit++;
        e += S[kk];
      }
    }
    if (!pred || (float)hit / pred < MIN_FRAC) continue;
    float score = e * hit / pred;
    if (score > bestScore) {
      bestScore = score;
      best = f0;
    }
  }
  return best;
}

// 4. Direction from the harmonics of f0 inside the band. Each harmonic's per-mic amplitude
// is the power within +/-1 bin of its peak (the Hann main lobe)
static void harmonics(float f0, DirPacket &pkt) {
  float E[4] = {0, 0, 0, 0};
  float vx = 0, vy = 0, wsum = 0;
  int nh = 0;
  for (int h = 1; f0 > 0 && h <= HMAX; h++) {
    float f = h * f0;
    if (f > BAND_HI) break;
    if (f < BAND_LO) continue;
    int k = localMax(f, tolBins(f, f0));
    if (k < 0 || S[k] < thr[k]) continue;
    float a[4], asum = 0, hx = 0, hy = 0;
    for (int m = 0; m < 4; m++) {
      float e = P[m][k - 1] + P[m][k] + P[m][k + 1];
      E[m] += e;
      a[m] = GAIN[m] * sqrtf(e);
      asum += a[m];
      hx += a[m] * dirX[m];
      hy += a[m] * dirY[m];
    }
    if (asum <= 0) continue;
    vx += hx;  // sum of (this harmonic's direction x its total amplitude)
    vy += hy;
    wsum += asum;
    nh++;
  }
  if (nh < MIN_HARM) return;  // packet stays zeroed
  for (int m = 0; m < 4; m++) pkt.amp[m] = GAIN[m] * sqrtf(2 * E[m] / (NFFT * winPow));  // Parseval, window-corrected
  pkt.x = vx / wsum;
  pkt.y = vy / wsum;
  pkt.f0 = f0;
  pkt.nharm = nh;
}

void setup() {
  Serial.setTxBufferSize(16384);
  Serial.begin();
  startBus(I2S_NUM_0, 5, 6, 3);  // bus A: mics 1, 2
  startBus(I2S_NUM_1, 8, 7, 4);  // bus B: mics 3, 4

  dsps_fft2r_init_fc32(nullptr, NFFT);
  dsps_wind_hann_f32(win, NFFT);
  for (int n = 0; n < NFFT; n++) winPow += win[n] * win[n];
  for (int m = 0; m < 4; m++) {
    dirX[m] = cosf(AZ_DEG[m] * PI / 180);
    dirY[m] = sinf(AZ_DEG[m] * PI / 180);
  }
}

void loop() {
  size_t n;
  i2s_read(I2S_NUM_0, bufA, BLK, &n, portMAX_DELAY);  // blocks until a block is ready
  i2s_read(I2S_NUM_1, bufB, BLK, &n, portMAX_DELAY);
  appendSamples();

  spectra();
  noiseFloor();

  DirPacket pkt = {{'D', 'I', 'R', 'V'}};
  harmonics(findPitch(), pkt);

  // 5. Output
  Serial.write((const uint8_t *)"SYNC", 4);
  Serial.write(bufA, BLK);
  Serial.write(bufB, BLK);
  Serial.write((const uint8_t *)&pkt, sizeof(pkt));
}
