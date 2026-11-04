// Single-tone tracker. Reads both I2S buses at 32 kHz, then every packet (256 new samples per mic, 125 per second):
//   1. streams raw audio packed to 16-bit: "SYNC" + bus A block + bus B block (for host/plot, scope, peak.py)
//   2. spectra: 2048-point FFT per mic (64 ms window, 15.6 Hz per bin). Mics are FFT'd in pairs (one as the
//      real part, one as the imaginary part) and separated afterwards, so 2 FFTs per packet instead of 4.
//      Only the bins within +/- FLOOR_HZ of the tone are kept.
//   3. tone: strongest bin of the 4 summed spectra within TONE_HZ +/- TONE_TOL_HZ, accepted if it is
//      DETECT_DB above the local noise floor (median of the summed spectrum within +/- FLOOR_HZ)
//      for MIN_RUN packets in a row at a steady frequency
//   4. direction: each mic's tone amplitude (power in the peak bin +/- 1), amplitude-weighted sum of the
//      directions the mics face
//   5. streams "DIRV" + 4 amplitudes + x + y + tone frequency + SNR dB (little-endian float32), for vector.py
// No tone detected -> everything after the tag is 0.

#include <Arduino.h>
#include <driver/i2s.h>
#include <esp_dsp.h>
#include <math.h>
#include <algorithm>

static const int RATE = 32000;
static const int HOP = 256;               // new samples per mic per packet (8 ms)
static const int BLK = HOP * 2 * 4;       // bytes per bus per I2S read: 256 stereo frames of 32-bit
static const int BLK16 = HOP * 2 * 2;     // bytes per bus per packet once packed to 16-bit
static const int NFFT = 2048;             // FFT length: 64 ms of audio
static const float BIN_HZ = (float)RATE / NFFT;  // 15.625 Hz per bin; 7 kHz = bin 448

// ---- Tuning ----
static const float TONE_HZ = 7000;      // beacon frequency
static const float TONE_TOL_HZ = 150;   // search window around TONE_HZ (source frequency error)
static const float FLOOR_HZ = 1000;     // noise floor taken from bins within this of the tone
static const float DETECT_DB = 12;      // tone must be this far above the noise floor in a packet...
static const int MIN_RUN = 6;           // ...for this many packets in a row (48 ms) at the same frequency (+/- 2 bins).
                                        // Room sound gave isolated hits (runs of 3 at most); a beacon is continuous
static const float GAIN[4] = {1.312f, 0.927f, 0.727f, 1.131f};  // per-mic gain correction (from 2-6 kHz; recalibrate at TONE_HZ with host/calibrate.py)
static const float AZ_DEG[4] = {90, 180, 0, 270};  // direction each mic faces around the vertical (mic 1..4)

static const int K_TOL = std::max(2, (int)(TONE_TOL_HZ / BIN_HZ + 0.5f));  // tone search half-width (bins)
static const int K_FLOOR = (int)(FLOOR_HZ / BIN_HZ + 0.5f);               // analysis half-width (bins)
static const int NR = 2 * K_FLOOR + 1;                                    // bins kept around the tone

static uint8_t bufA[BLK];
static uint8_t bufB[BLK];
static int16_t outA[2 * HOP];    // bufA packed to 16-bit (top 16 bits of each 24-bit sample)
static int16_t outB[2 * HOP];
static float ring[4][NFFT];      // last NFFT samples per mic, circular, full-scale units (1.0 = full scale)
static int ringPos = 0;          // index of the oldest sample in ring
// ESP-DSP's optimised FFT needs 16-byte aligned buffers; unaligned gave garbage (~+24 dBFS on every mic)
static float win[NFFT] __attribute__((aligned(16)));          // Hann window
static float fftbuf[2 * NFFT] __attribute__((aligned(16)));   // interleaved re, im
static float P[4][NR];           // power per mic, |X|^2, for bins k0 .. k0 + NR - 1
static float S[NR];              // sum of the 4 power spectra
static float tmp[NR];            // scratch for the median
static float winPow;             // sum of win^2, for Parseval scaling
static float dirX[4], dirY[4];   // unit vectors each mic faces
static int k0;                   // first bin kept (tone bin - K_FLOOR)

struct DirPacket {
  char tag[4];
  float amp[4];  // tone RMS per mic, full-scale units (gain corrected)
  float x, y;    // direction vector, length 0..1
  float freq;    // Hz, measured tone frequency; 0 = no tone
  float snr;     // dB above the noise floor; 0 = no tone
};
static_assert(sizeof(DirPacket) == 36, "no padding: host/stream.py reads 4 + 8 floats");

static void startBus(i2s_port_t port, int sck, int ws, int sd) {
  i2s_config_t cfg = {};
  cfg.mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX);
  cfg.sample_rate = RATE;
  cfg.bits_per_sample = I2S_BITS_PER_SAMPLE_32BIT;
  cfg.channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT;  // stereo
  cfg.communication_format = I2S_COMM_FORMAT_STAND_I2S;
  cfg.dma_buf_count = 16;
  cfg.dma_buf_len = 256;  // frames per DMA buffer; 16 x 256 frames = 128 ms of slack at 32 kHz
  i2s_driver_install(port, &cfg, 0, nullptr);

  i2s_pin_config_t pins = {};
  pins.mck_io_num = I2S_PIN_NO_CHANGE;  // no MCLK (0 would put it on GPIO0)
  pins.bck_io_num = sck;
  pins.ws_io_num = ws;
  pins.data_out_num = I2S_PIN_NO_CHANGE;
  pins.data_in_num = sd;
  i2s_set_pin(port, &pins);
}

// 1. Write the new samples into each mic's ring, and pack the raw blocks to 16-bit for streaming
static void appendSamples() {
  const int32_t *a = (const int32_t *)bufA;  // frames: [left = mic 2, right = mic 1]
  const int32_t *b = (const int32_t *)bufB;  // frames: [left = mic 4, right = mic 3]
  const float s24 = 1.0f / 8388608.0f;       // 24-bit left-justified -> full-scale units
  float *r0 = ring[0] + ringPos, *r1 = ring[1] + ringPos, *r2 = ring[2] + ringPos, *r3 = ring[3] + ringPos;
  for (int f = 0; f < HOP; f++) {            // NFFT is a multiple of HOP, so a block never wraps
    r0[f] = (a[2 * f + 1] >> 8) * s24;
    r1[f] = (a[2 * f] >> 8) * s24;
    r2[f] = (b[2 * f + 1] >> 8) * s24;
    r3[f] = (b[2 * f] >> 8) * s24;
  }
  ringPos = (ringPos + HOP) % NFFT;
  for (int i = 0; i < 2 * HOP; i++) {
    outA[i] = (int16_t)(a[i] >> 16);
    outB[i] = (int16_t)(b[i] >> 16);
  }
}

// 2. One complex FFT of mic m1 (real part) + j * mic m2 (imaginary part); split into each mic's power
// near the tone using X1[k] = (Z[k] + conj Z[N-k]) / 2 and X2[k] = (Z[k] - conj Z[N-k]) / 2j
static void fftPair(int m1, int m2) {
  float mean1 = 0, mean2 = 0;
  for (int n = 0; n < NFFT; n++) {
    mean1 += ring[m1][n];
    mean2 += ring[m2][n];
  }
  mean1 /= NFFT;
  mean2 /= NFFT;
  for (int n = 0, i = ringPos; n < NFFT; n++, i = (i + 1 == NFFT) ? 0 : i + 1) {  // oldest -> newest
    fftbuf[2 * n] = (ring[m1][i] - mean1) * win[n];
    fftbuf[2 * n + 1] = (ring[m2][i] - mean2) * win[n];
  }
  dsps_fft2r_fc32(fftbuf, NFFT);
  dsps_bit_rev2r_fc32(fftbuf, NFFT);
  for (int r = 0; r < NR; r++) {
    int k = k0 + r;
    float zr = fftbuf[2 * k], zi = fftbuf[2 * k + 1];
    float wr = fftbuf[2 * (NFFT - k)], wi = fftbuf[2 * (NFFT - k) + 1];
    float x1r = 0.5f * (zr + wr), x1i = 0.5f * (zi - wi);
    float x2r = 0.5f * (zi + wi), x2i = -0.5f * (zr - wr);
    P[m1][r] = x1r * x1r + x1i * x1i;
    P[m2][r] = x2r * x2r + x2i * x2i;
  }
}

static void spectra() {
  fftPair(0, 1);
  fftPair(2, 3);
  for (int r = 0; r < NR; r++) S[r] = P[0][r] + P[1][r] + P[2][r] + P[3][r];
}

// 3 + 4. Find the tone, then each mic's amplitude at it and the direction vector
static void tone(DirPacket &pkt) {
  int c = K_FLOOR;  // tone bin within the kept range
  int k = c - K_TOL;
  for (int i = c - K_TOL + 1; i <= c + K_TOL; i++)
    if (S[i] > S[k]) k = i;

  int n = 0;  // noise floor: median of the summed spectrum around the tone, excluding the peak's main lobe
  for (int i = 0; i < NR; i++)
    if (abs(i - k) > 4) tmp[n++] = S[i];
  std::nth_element(tmp, tmp + n / 2, tmp + n);
  float floorP = tmp[n / 2];
  float snr = 10 * log10f(S[k] / (floorP + 1e-30f));
  if (snr < DETECT_DB) return;  // packet stays zeroed

  float a = logf(S[k - 1] + 1e-30f), b = logf(S[k] + 1e-30f), cc = logf(S[k + 1] + 1e-30f);
  float den = a - 2 * b + cc;
  pkt.freq = (k0 + k + (den != 0 ? 0.5f * (a - cc) / den : 0)) * BIN_HZ;  // parabolic interpolation
  pkt.snr = snr;

  float asum = 0, vx = 0, vy = 0;
  for (int m = 0; m < 4; m++) {
    float e = P[m][k - 1] + P[m][k] + P[m][k + 1];  // Hann main lobe: all of an on-bin tone's energy
    pkt.amp[m] = GAIN[m] * sqrtf(2 * e / (NFFT * winPow));  // Parseval, window-corrected RMS
    asum += pkt.amp[m];
    vx += pkt.amp[m] * dirX[m];
    vy += pkt.amp[m] * dirY[m];
  }
  if (asum > 0) {
    pkt.x = vx / asum;
    pkt.y = vy / asum;
  }
}

void setup() {
  Serial.setTxBufferSize(16384);
  Serial.begin();
  startBus(I2S_NUM_0, 5, 6, 3);  // bus A: mics 1, 2
  startBus(I2S_NUM_1, 8, 7, 4);  // bus B: mics 3, 4

  dsps_fft2r_init_fc32(nullptr, NFFT);
  dsps_wind_hann_f32(win, NFFT);
  for (int n = 0; n < NFFT; n++) winPow += win[n] * win[n];
  k0 = lroundf(TONE_HZ / BIN_HZ) - K_FLOOR;
  for (int m = 0; m < 4; m++) {
    dirX[m] = cosf(AZ_DEG[m] * (float)PI / 180);
    dirY[m] = sinf(AZ_DEG[m] * (float)PI / 180);
  }
}

void loop() {
  size_t n;
  i2s_read(I2S_NUM_0, bufA, BLK, &n, portMAX_DELAY);  // blocks until a block is ready
  i2s_read(I2S_NUM_1, bufB, BLK, &n, portMAX_DELAY);
#ifdef TEST_TONE
  // Bench test only (env mic_test): replace the audio with a synthetic TEST_TONE Hz sine, louder at mic 1
  static float ph = 0;
  const float dph = 2 * (float)PI * TEST_TONE / RATE, g[4] = {1.0f, 0.5f, 0.5f, 0.5f};
  int32_t *ta = (int32_t *)bufA, *tb = (int32_t *)bufB;
  for (int f = 0; f < HOP; f++) {
    float s = 0.01f * sinf(ph) * 2147483647.0f;
    ph += dph;
    if (ph > 2 * (float)PI) ph -= 2 * (float)PI;
    ta[2 * f + 1] = (int32_t)(g[0] * s); ta[2 * f] = (int32_t)(g[1] * s);
    tb[2 * f + 1] = (int32_t)(g[2] * s); tb[2 * f] = (int32_t)(g[3] * s);
  }
#endif
#ifdef TIMING
  uint32_t t0 = micros();
#endif
  appendSamples();
#ifdef TIMING
  uint32_t t1 = micros();
#endif
  spectra();
#ifdef TIMING
  uint32_t t2 = micros();
#endif

  DirPacket pkt = {{'D', 'I', 'R', 'V'}};
  tone(pkt);

  // Persistence: only report a tone held for MIN_RUN packets at a steady frequency
  static int run = 0;
  static float lastFreq = 0;
  bool hit = pkt.snr > 0;
  if (hit && run > 0 && fabsf(pkt.freq - lastFreq) > 2 * BIN_HZ) run = 0;  // frequency jumped: not the same tone
  run = hit ? run + 1 : 0;
  if (hit) lastFreq = pkt.freq;
  if (run < MIN_RUN) pkt = DirPacket{{'D', 'I', 'R', 'V'}};

  // 5. Output
  Serial.write((const uint8_t *)"SYNC", 4);
  Serial.write((const uint8_t *)outA, BLK16);
  Serial.write((const uint8_t *)outB, BLK16);
#ifdef TIMING
  pkt.amp[0] = t1 - t0;  // bench only: us for append / spectra / whole loop body
  pkt.amp[1] = t2 - t1;
  pkt.amp[2] = micros() - t0;
#endif
  Serial.write((const uint8_t *)&pkt, sizeof(pkt));
}
