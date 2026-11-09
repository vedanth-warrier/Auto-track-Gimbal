// TDOA tone tracker.
//
// Capture: I2S0 is the clock master on bus A (SCK 5, WS 6, SD 3). Its SCK/WS output signals are also routed,
// through the GPIO matrix, to bus B's clock pins (SCK 8, WS 7), and I2S1 runs as a slave on those pins
// (SD 4). All 4 mics are therefore clocked and sampled on the same edges. Both receivers are started
// slave-first so their DMA streams begin on the same frame; any DMA overflow triggers a re-sync.
//
// The slave's SCK input is inverted and its words shifted left by one: see startCapture.
//
// Every packet (256 new samples per mic, 8 ms, 125 per second):
//   1. raw audio packed to 16-bit is streamed: "SYNC" + bus A block + bus B block
//   2. 2048-point Hann FFT per mic (two mics per complex FFT), bins within +/- FLOOR_HZ of the tone kept
//   3. tone detection: strongest bin within TDOA_TONE_HZ +/- TONE_TOL_HZ, DETECT_DB above the local floor,
//      held for MIN_RUN packets at a steady frequency
//   4. TDOA: cross-spectrum of each of the 6 mic pairs at the tone (peak bin +/- 1), phase-normalised, corrected
//      for the calibrated per-mic delays (MIC_DELAY_US) and averaged over time (CS_TAU).
//      Phase difference = 2 pi f x delay.
//   5. direction: steered response over a grid of directions (upper hemisphere). The best match is taken only
//      within TDOA_CONE_DEG of a centre (array axis until locked, then the last estimate), because pairs
//      longer than half a wavelength also fit ghost directions. Least squares on the unwrapped pair phases
//      then refines it. Quality = how well all 6 pairs agree; ambiguity = best fit anywhere > AMBIG_SEP away
//      / best (ghosts give ~1, which is why the cone exists); edge = best match on the cone boundary.
//      For a planar array a single tone's ghosts are exact (identical pair phases), so a source that starts
//      outside the cone can lock onto a ghost inside it undetectably: acquisition needs the source in the cone.
//   6. tracker: ignores estimates below TDOA_TRK_Q_MIN, averages the rest (TDOA_TRK_TAU), caps the output's speed
//      (TDOA_TRK_MAX_RATE), holds when there's no estimate and drops the track after TDOA_TRK_LOST_S; a servo
//      command that only moves once the track is TDOA_DEADBAND_DEG away from it
//   7. servos (velocity mode): pointing error = tracked direction off the array axis, split into tilt (towards
//      TDOA_TILT_UP_AZ_DEG) and pan. Speed deadband with hysteresis (TDOA_START_DEG / TDOA_STOP_DEG), speed
//      proportional to the error (TDOA_GAIN) capped at TDOA_MAX_RATE, acceleration limit, stop when the track is
//      lost. Pulses for continuous-rotation servos. Commands from the host: E0/E1 off/auto, TP<r>/TT<r> test
//      speed (rad/s), NP<us>/NT<us> stop pulse, S stop.
//   8. "TDOA" packet: raw direction, quality, ambiguity, edge, tone, pair delays, re-sync count, sim truth,
//      track state, tracked direction, servo command, servo rates and pulses

#include <Arduino.h>
#include <driver/i2s.h>
#include <driver/gpio.h>
#include <esp_rom_gpio.h>
#include <soc/gpio_struct.h>
#include <soc/gpio_sig_map.h>
#include <esp_dsp.h>
#include <math.h>
#include <algorithm>
#include "config.h"

static const int RATE = TDOA_RATE;
static const int HOP = 256;               // new samples per mic per packet (8 ms)
static const int BLK = HOP * 2 * 4;       // bytes per bus per I2S read: 256 stereo frames of 32-bit
static const int BLK16 = HOP * 2 * 2;     // bytes per bus per packet once packed to 16-bit
static const int NFFT = 2048;             // FFT length: 64 ms of audio
static const float BIN_HZ = (float)RATE / NFFT;  // 15.625 Hz per bin
static const float OMEGA = 2 * (float)PI * TDOA_TONE_HZ;  // rad/s at the tone

// ---- Tuning ----
static const float TONE_TOL_HZ = 150;   // search window around the tone (source frequency error)
static const float FLOOR_HZ = 1000;     // noise floor and kept bins: within this of the tone
static const float DETECT_DB = 12;      // tone must be this far above the noise floor in a packet...
static const int MIN_RUN = TDOA_MIN_RUN;  // ...for this many packets in a row within +/- 2 bins (config.h)
static const float CS_TAU = TDOA_CS_TAU;  // s, time constant of the cross-spectrum average (config.h; 0 = off)
static const float AZ_STEP = 4;         // deg, direction grid spacing in azimuth
static const float EL_STEP = 4;         // deg, direction grid spacing in elevation (0 .. 90)
static const float AMBIG_SEP = 30;      // deg, how far away a competing direction must be to count
static const i2s_mclk_multiple_t SLAVE_MCLK_MULT = I2S_MCLK_MULTIPLE_384;  // bus B slave module clock = rate x this

static const int K_TOL = std::max(2, (int)(TONE_TOL_HZ / BIN_HZ + 0.5f));
static const int K_FLOOR = (int)(FLOOR_HZ / BIN_HZ + 0.5f);
static const int NR = 2 * K_FLOOR + 1;                // bins kept around the tone
static const int NPAIR = 6;
static const int PAIR[NPAIR][2] = {{0, 1}, {0, 2}, {0, 3}, {1, 2}, {1, 3}, {2, 3}};
static const int N_AZ = (int)(360 / AZ_STEP + 0.5f);
static const int N_EL = (int)(90 / EL_STEP + 0.5f);   // elevations 0 .. 90 - EL_STEP, plus the zenith
static const int NG = N_AZ * N_EL + 1;

// Capture
static uint8_t bufA[BLK];
static uint8_t bufB[BLK];
static int16_t outA[2 * HOP];
static int16_t outB[2 * HOP];
static QueueHandle_t qA, qB;      // I2S driver event queues (DMA overflow detection)
static float ring[4][NFFT];       // last NFFT samples per mic, circular, full-scale units
static int ringPos = 0;           // index of the oldest sample in ring
static int ringFill = 0;          // samples since the last (re)sync; analysis waits for a full window
static uint32_t resyncs = 0;

// Spectra. ESP-DSP's optimised FFT needs 16-byte aligned buffers
static float win[NFFT] __attribute__((aligned(16)));
static float fftbuf[2 * NFFT] __attribute__((aligned(16)));
static float Xr[4][NR], Xi[4][NR];  // complex spectrum per mic, bins k0 .. k0 + NR - 1
static float S[NR];                 // summed power
static float tmp[NR];
static int k0;

// Direction grid
static float gu[NG][3];               // unit vector of each grid direction
static int16_t steerC[NG][NPAIR];     // cos / sin of predicted pair phase, x 32767
static int16_t steerS[NG][NPAIR];
static float pd[NPAIR][3];            // p_i - p_j per pair
static float corrR[NPAIR], corrI[NPAIR];  // e^{+j w (d_i - d_j)}: removes the calibrated mic delays
static bool planar;                   // all mics at the same z (within 1 mm)
static float Gr[NPAIR], Gi[NPAIR];    // averaged phase-normalised cross-spectra
static bool gValid = false;
static float centre[3] = {0, 0, 1};   // search-cone centre: array axis, or the last estimate while tracking
static uint32_t lastFix = 0;          // packet count of the last valid estimate
static uint32_t pktCount = 0;

struct TdoaPacket {
  char tag[4];          // "TDOA"
  float ux, uy, uz;     // unit vector towards the source, array frame; all 0 = no tone
  float az, el;         // deg: azimuth anticlockwise from +x (mic 3), elevation above the array plane
  float quality;        // 0..1, how consistently all 6 pair phases fit the solution
  float ambiguity;      // 0..1, best fit anywhere > AMBIG_SEP away / best; near 1 = a ghost fits as well
  float edge;           // 1 = best match on the search-cone boundary (source may be leaving or outside the cone)
  float freq, snr;      // tone Hz and dB above the floor; 0 = no tone
  float tau[NPAIR];     // pair delays t_i - t_j (us) for (1,2) (1,3) (1,4) (2,3) (2,4) (3,4); + = j hears it first
  float resyncs;        // DMA-overflow re-syncs since boot
  float simAz, simEl;   // synthetic source direction (tdoa_sim builds), else 0
  float trackState;     // 0 lost, 1 tracking (estimate accepted), 2 coasting (estimate rejected or none)
  float tu[3];          // tracked direction (unit vector); 0 when lost
  float cmd[3];         // servo command direction (unit vector); 0 before the first track
  float panRate, tiltRate;  // rad/s commanded (after deadband, speed and acceleration limits)
  float panUs, tiltUs;      // pulse widths for those rates
  float moving;             // 1 = error outside the deadband, servos driving
  float servoMode;          // 0 off (no pulses sent), 1 automatic, 2 test
};
static_assert(sizeof(TdoaPacket) == 4 + 32 * 4, "no padding: host/tstream.py reads 4 + 32 floats");

// ---------------------------------------------------------------- capture

static void installBus(i2s_port_t port, bool master, int sck, int ws, int sd, QueueHandle_t *q) {
  i2s_config_t cfg = {};
  cfg.mode = (i2s_mode_t)((master ? I2S_MODE_MASTER : I2S_MODE_SLAVE) | I2S_MODE_RX);
  cfg.sample_rate = RATE;
  cfg.bits_per_sample = I2S_BITS_PER_SAMPLE_32BIT;
  cfg.channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT;
  cfg.communication_format = I2S_COMM_FORMAT_STAND_I2S;
  cfg.dma_buf_count = 16;
  cfg.dma_buf_len = 256;  // 16 x 256 frames = 128 ms of slack
  if (!master) cfg.mclk_multiple = SLAVE_MCLK_MULT;  // slave's module clock, which it samples SCK with
  i2s_driver_install(port, &cfg, 8, q);
  i2s_pin_config_t pins = {};
  pins.mck_io_num = I2S_PIN_NO_CHANGE;
  pins.bck_io_num = sck;
  pins.ws_io_num = ws;
  pins.data_out_num = I2S_PIN_NO_CHANGE;
  pins.data_in_num = sd;
  i2s_set_pin(port, &pins);
}

// Stop both, arm the slave, then start the master: both DMA streams begin on the master's first frame
static void resync() {
  i2s_stop(I2S_NUM_0);
  i2s_stop(I2S_NUM_1);
  xQueueReset(qA);
  xQueueReset(qB);
  i2s_start(I2S_NUM_1);
  i2s_start(I2S_NUM_0);
  ringFill = 0;
  gValid = false;
  resyncs++;
}

static void startCapture() {
  installBus(I2S_NUM_1, false, 8, 7, 4, &qB);  // slave on bus B pins; waits for clocks
#ifdef SYNC_TEST
  esp_rom_gpio_connect_in_signal(3, I2S1I_SD_IN_IDX, false);  // bus B receiver reads bus A's data line
#endif
  installBus(I2S_NUM_0, true, 5, 6, 3, &qA);   // master on bus A pins
  // Drive bus B's clock pins with the master's own SCK/WS (whatever signals the driver put on GPIO5/6)
  uint32_t bckSig = GPIO.func_out_sel_cfg[5].func_sel;
  uint32_t wsSig = GPIO.func_out_sel_cfg[6].func_sel;
  gpio_set_direction(GPIO_NUM_8, GPIO_MODE_INPUT_OUTPUT);  // keeps the slave's input routing on the pad
  gpio_set_direction(GPIO_NUM_7, GPIO_MODE_INPUT_OUTPUT);
  esp_rom_gpio_connect_out_signal(8, bckSig, false, false);
  esp_rom_gpio_connect_out_signal(7, wsSig, false, false);
  // The slave detects SCK edges a few module-clock cycles late, which put its sampling point right on the
  // mics' data transitions (falling SCK edge): bits slipped mid-sample. Inverting its SCK input moves the
  // sampling point half a bit away, into the stable part of each bit. It then reads every bit one position
  // late, which appendSamples undoes with << 1 (sync test: 100% of data bits match bus A after the shift).
  esp_rom_gpio_connect_in_signal(8, I2S1I_BCK_IN_IDX, true);
  resync();
  resyncs = 0;
}

// True if either receiver dropped data since the last call
static bool overflowed() {
  bool o = false;
  i2s_event_t ev;
  while (xQueueReceive(qA, &ev, 0) == pdTRUE) o |= ev.type == I2S_EVENT_RX_Q_OVF;
  while (xQueueReceive(qB, &ev, 0) == pdTRUE) o |= ev.type == I2S_EVENT_RX_Q_OVF;
  return o;
}

static void appendSamples() {
  const int32_t *a = (const int32_t *)bufA;  // frames: [left, right]
  int32_t *b = (int32_t *)bufB;              // frames: [left, right]
  for (int i = 0; i < 2 * HOP; i++) b[i] = (int32_t)((uint32_t)b[i] << 1);  // undo the slave's 1-bit lag
  const float s24 = 1.0f / 8388608.0f;
  for (int m = 0; m < 4; m++) {  // mic m's samples come from slot MIC_SLOT[m]: bus (A/B) and column (left/right)
    const int32_t *src = (MIC_SLOT[m] < 2 ? a : b) + (MIC_SLOT[m] & 1);
    float *dst = ring[m] + ringPos;
    for (int f = 0; f < HOP; f++) dst[f] = (src[2 * f] >> 8) * s24;
  }
  ringPos = (ringPos + HOP) % NFFT;
  ringFill = std::min(ringFill + HOP, NFFT);
  for (int i = 0; i < 2 * HOP; i++) {
    outA[i] = (int16_t)(a[i] >> 16);
    outB[i] = (int16_t)(b[i] >> 16);
  }
}

#ifdef TDOA_SIM
// Plane-wave tone from a slowly rotating direction: mic i hears sin(w t + w p_i.u / c), plus faint noise
static float simAz = 0, simEl = 90;
static void simulate() {
  static float ph = 0;
  static uint32_t lcg = 12345;
  static float simT = 0;
  simT += (float)HOP / RATE;
  simAz += 20.0f * HOP / RATE;  // 20 deg/s
  if (simAz >= 360) simAz -= 360;
  simEl = 67.5f + 22.5f * cosf(2 * (float)PI * simT / 20);  // 90 -> 45 -> 90 deg over 20 s: leaves the axis cone
  float az = simAz * (float)PI / 180, el = simEl * (float)PI / 180;
  float u[3] = {cosf(el) * cosf(az), cosf(el) * sinf(az), sinf(el)};
  float off[4];
  for (int m = 0; m < 4; m++)  // geometric phase, minus each mic's configured extra delay (as real mics would have)
    off[m] = OMEGA * (MIC_POS[m][0] * u[0] + MIC_POS[m][1] * u[1] + MIC_POS[m][2] * u[2]) / TDOA_SPEED - OMEGA * 1e-6f * MIC_DELAY_US[m];
  float gain2 = 1;
#ifdef SIM_CORRUPT
  if (fmodf(simT, 5.0f) >= 2.5f && fmodf(simT, 5.0f) < 2.8f) {  // fake reflection null on mic 2: weak, wrong phase
    gain2 = 0.1f;
    off[1] += 2.0f;
  }
#endif
  const float dph = OMEGA / RATE, amp = 0.01f * 2147483647.0f, nz = 1e-4f * 2147483647.0f / 2147483648.0f;
  int32_t *a = (int32_t *)bufA, *b = (int32_t *)bufB;
  for (int f = 0; f < HOP; f++) {
    float v[4];
    for (int m = 0; m < 4; m++) {
      lcg = lcg * 1664525u + 1013904223u;
      v[m] = amp * (m == 1 ? gain2 : 1) * sinf(ph + off[m]) + nz * (int32_t)lcg;
    }
    for (int m = 0; m < 4; m++) {  // each mic into its slot; bus B pre-shifted to undo appendSamples' << 1
      int s = MIC_SLOT[m];
      (s < 2 ? a : b)[2 * f + (s & 1)] = s < 2 ? (int32_t)v[m] : (int32_t)v[m] >> 1;
    }
    ph += dph;
    if (ph > 2 * (float)PI) ph -= 2 * (float)PI;
  }
}
#endif

// ---------------------------------------------------------------- spectra

// One complex FFT of mic m1 (real) + j mic m2 (imaginary), split with X1 = (Z[k] + conj Z[N-k]) / 2 and
// X2 = (Z[k] - conj Z[N-k]) / 2j, kept for bins k0 .. k0 + NR - 1
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
    Xr[m1][r] = 0.5f * (zr + wr);
    Xi[m1][r] = 0.5f * (zi - wi);
    Xr[m2][r] = 0.5f * (zi + wi);
    Xi[m2][r] = -0.5f * (zr - wr);
  }
}

static void spectra() {
  fftPair(0, 1);
  fftPair(2, 3);
  for (int r = 0; r < NR; r++) {
    S[r] = 0;
    for (int m = 0; m < 4; m++) S[r] += Xr[m][r] * Xr[m][r] + Xi[m][r] * Xi[m][r];
  }
}

// Tone bin (index into the kept range) or -1; fills freq and snr
static int detectTone(float &freq, float &snr) {
  int c = K_FLOOR, k = c - K_TOL;
  for (int i = c - K_TOL + 1; i <= c + K_TOL; i++)
    if (S[i] > S[k]) k = i;
  int n = 0;
  for (int i = 0; i < NR; i++)
    if (abs(i - k) > 4) tmp[n++] = S[i];
  std::nth_element(tmp, tmp + n / 2, tmp + n);
  snr = 10 * log10f(S[k] / (tmp[n / 2] + 1e-30f));
  if (snr < DETECT_DB) return -1;
  float a = logf(S[k - 1] + 1e-30f), b = logf(S[k] + 1e-30f), cc = logf(S[k + 1] + 1e-30f);
  float den = a - 2 * b + cc;
  freq = (k0 + k + (den != 0 ? 0.5f * (a - cc) / den : 0)) * BIN_HZ;
  return k;
}

// ---------------------------------------------------------------- direction

static void buildGrid() {
  for (int p = 0; p < NPAIR; p++) {
    for (int d = 0; d < 3; d++) pd[p][d] = MIC_POS[PAIR[p][0]][d] - MIC_POS[PAIR[p][1]][d];
    float ph = OMEGA * 1e-6f * (MIC_DELAY_US[PAIR[p][0]] - MIC_DELAY_US[PAIR[p][1]]);
    corrR[p] = cosf(ph);
    corrI[p] = sinf(ph);
  }
  float zmin = MIC_POS[0][2], zmax = MIC_POS[0][2];
  for (int m = 1; m < 4; m++) {
    zmin = std::min(zmin, MIC_POS[m][2]);
    zmax = std::max(zmax, MIC_POS[m][2]);
  }
  planar = (zmax - zmin) < 0.001f;
  int g = 0;
  for (int e = 0; e <= N_EL; e++) {
    for (int a = 0; a < (e == N_EL ? 1 : N_AZ); a++, g++) {
      float el = (e == N_EL ? 90.0f : e * EL_STEP) * (float)PI / 180, az = a * AZ_STEP * (float)PI / 180;
      gu[g][0] = cosf(el) * cosf(az);
      gu[g][1] = cosf(el) * sinf(az);
      gu[g][2] = sinf(el);
      for (int p = 0; p < NPAIR; p++) {
        // pair cross-phase angle(X_i conj X_j) = w (p_i - p_j).u / c
        float th = OMEGA * (pd[p][0] * gu[g][0] + pd[p][1] * gu[g][1] + pd[p][2] * gu[g][2]) / TDOA_SPEED;
        steerC[g][p] = (int16_t)lroundf(32767 * cosf(th));
        steerS[g][p] = (int16_t)lroundf(32767 * sinf(th));
      }
    }
  }
}

static float wrapPi(float x) {
  x = fmodf(x + (float)PI, 2 * (float)PI);
  if (x < 0) x += 2 * (float)PI;
  return x - (float)PI;
}

// Cross-spectra at the tone, steered search, least-squares refinement
static void direction(int k, TdoaPacket &pkt) {
  float gr[NPAIR], gi[NPAIR];
  for (int p = 0; p < NPAIR; p++) {
    int i = PAIR[p][0], j = PAIR[p][1];
    float cr = 0, ci = 0;
    for (int r = k - 1; r <= k + 1; r++) {  // X_i conj X_j over the Hann main lobe
      cr += Xr[i][r] * Xr[j][r] + Xi[i][r] * Xi[j][r];
      ci += Xi[i][r] * Xr[j][r] - Xr[i][r] * Xi[j][r];
    }
    float mag = sqrtf(cr * cr + ci * ci) + 1e-30f;  // phase transform: keep the phase only
    float nr = cr / mag, ni = ci / mag;
    gr[p] = nr * corrR[p] - ni * corrI[p];  // remove the calibrated mic delays
    gi[p] = nr * corrI[p] + ni * corrR[p];
  }
  float a = CS_TAU > 0 ? 1 - expf(-(float)HOP / RATE / CS_TAU) : 1;
  for (int p = 0; p < NPAIR; p++) {
    Gr[p] = gValid ? Gr[p] + a * (gr[p] - Gr[p]) : gr[p];
    Gi[p] = gValid ? Gi[p] + a * (gi[p] - Gi[p]) : gi[p];
  }
  gValid = true;

  // Steered response P(g) = sum over pairs Re(G conj steer); best match taken inside the search cone only
  int best = -1;
  float pBest = -1e30f, bestDot = 0;
  static float P[NG];
  const bool fullSky = TDOA_CONE_DEG >= 90;  // no ghosts: search the whole upper hemisphere
  const float cosCone = cosf(TDOA_CONE_DEG * (float)PI / 180);
  for (int g = 0; g < NG; g++) {
    float s = 0;
    for (int p = 0; p < NPAIR; p++) s += Gr[p] * steerC[g][p] + Gi[p] * steerS[g][p];
    P[g] = s;
    float dot = gu[g][0] * centre[0] + gu[g][1] * centre[1] + gu[g][2] * centre[2];
    if ((fullSky || dot >= cosCone) && s > pBest) {
      pBest = s;
      best = g;
      bestDot = dot;
    }
  }
  if (best < 0) return;  // cone contains no grid point (cone smaller than the grid step)
  float coneEdge = cosf((TDOA_CONE_DEG - std::max(AZ_STEP, EL_STEP)) * (float)PI / 180);
  pkt.edge = !fullSky && bestDot < coneEdge ? 1 : 0;
  float cosSep = cosf(AMBIG_SEP * (float)PI / 180), p2 = -1e30f;
  for (int g = 0; g < NG; g++) {
    float dot = gu[g][0] * gu[best][0] + gu[g][1] * gu[best][1] + gu[g][2] * gu[best][2];
    if (dot < cosSep && P[g] > p2) p2 = P[g];
  }
  float gsum = 0;
  for (int p = 0; p < NPAIR; p++) gsum += sqrtf(Gr[p] * Gr[p] + Gi[p] * Gi[p]);
  pBest /= 32767;
  p2 /= 32767;

  // Refine: unwrap each pair's phase to the grid solution, then least squares on path differences
  float L[NPAIR], w[NPAIR];
  for (int p = 0; p < NPAIR; p++) {
    float th = OMEGA * (pd[p][0] * gu[best][0] + pd[p][1] * gu[best][1] + pd[p][2] * gu[best][2]) / TDOA_SPEED;
    float ph = atan2f(Gi[p], Gr[p]);
    L[p] = (th + wrapPi(ph - th)) * TDOA_SPEED / OMEGA;  // (p_i - p_j).u in metres
    w[p] = sqrtf(Gr[p] * Gr[p] + Gi[p] * Gi[p]);
  }
  float u[3];
  if (planar) {  // in-plane components only; z from the unit length (source above the plane)
    float a11 = 0, a12 = 0, a22 = 0, b1 = 0, b2 = 0;
    for (int p = 0; p < NPAIR; p++) {
      a11 += w[p] * pd[p][0] * pd[p][0]; a12 += w[p] * pd[p][0] * pd[p][1]; a22 += w[p] * pd[p][1] * pd[p][1];
      b1 += w[p] * pd[p][0] * L[p]; b2 += w[p] * pd[p][1] * L[p];
    }
    float det = a11 * a22 - a12 * a12;
    if (fabsf(det) > 1e-12f) {
      u[0] = (a22 * b1 - a12 * b2) / det;
      u[1] = (a11 * b2 - a12 * b1) / det;
    } else {
      u[0] = gu[best][0];
      u[1] = gu[best][1];
    }
    float r2 = u[0] * u[0] + u[1] * u[1];
    if (r2 > 1) {
      float r = sqrtf(r2);
      u[0] /= r; u[1] /= r; r2 = 1;
    }
    u[2] = sqrtf(1 - r2);
  } else {  // full 3D least squares, then normalise
    float A[3][3] = {}, B[3] = {};
    for (int p = 0; p < NPAIR; p++)
      for (int r = 0; r < 3; r++) {
        B[r] += w[p] * pd[p][r] * L[p];
        for (int c = 0; c < 3; c++) A[r][c] += w[p] * pd[p][r] * pd[p][c];
      }
    float det = A[0][0] * (A[1][1] * A[2][2] - A[1][2] * A[2][1]) - A[0][1] * (A[1][0] * A[2][2] - A[1][2] * A[2][0]) +
                A[0][2] * (A[1][0] * A[2][1] - A[1][1] * A[2][0]);
    if (fabsf(det) > 1e-15f) {
      for (int c = 0; c < 3; c++) {  // Cramer's rule
        float M[3][3];
        for (int r = 0; r < 3; r++)
          for (int cc = 0; cc < 3; cc++) M[r][cc] = (cc == c) ? B[r] : A[r][cc];
        u[c] = (M[0][0] * (M[1][1] * M[2][2] - M[1][2] * M[2][1]) - M[0][1] * (M[1][0] * M[2][2] - M[1][2] * M[2][0]) +
                M[0][2] * (M[1][0] * M[2][1] - M[1][1] * M[2][0])) / det;
      }
      float n = sqrtf(u[0] * u[0] + u[1] * u[1] + u[2] * u[2]) + 1e-30f;
      for (int c = 0; c < 3; c++) u[c] /= n;
    } else {
      for (int c = 0; c < 3; c++) u[c] = gu[best][c];
    }
  }
  pkt.ux = u[0];
  pkt.uy = u[1];
  pkt.uz = u[2];
  pkt.az = fmodf(atan2f(u[1], u[0]) * 180 / (float)PI + 360, 360);
  pkt.el = asinf(std::max(-1.0f, std::min(1.0f, u[2]))) * 180 / (float)PI;
  pkt.quality = pBest / (gsum + 1e-30f);
  pkt.ambiguity = p2 > 0 ? p2 / pBest : 0;
  for (int c = 0; c < 3; c++) centre[c] = u[c];  // follow the source
  lastFix = pktCount;
  for (int p = 0; p < NPAIR; p++) pkt.tau[p] = -1e6f * L[p] / TDOA_SPEED;  // t_i - t_j = -(p_i - p_j).u / c
}

// ---------------------------------------------------------------- tracker (mirrors host/tdoa_ref.py DirTracker)
// Real-room errors are slow, smooth excursions (reflections), not isolated outliers, so the tracker doesn't try to
// reject estimates: it averages them (TDOA_TRK_TAU), weighting each by its fit quality (TDOA_TRK_Q_POW), caps how
// fast the output may move (TDOA_TRK_MAX_RATE), and ignores only clearly broken ones (quality < TDOA_TRK_Q_MIN). Measured on a real 60 s recording: worst-5% wobble
// 15 deg vs 19 raw, largest single step 4.8 deg vs 178 raw (the gated tracker it replaced: 41 deg, coasting 77%).

static const float TRK_DT = (float)HOP / RATE;
static struct {
  int state = 0;                        // 0 lost, 1 tracking (estimate used this packet), 2 holding
  float e[3] = {0, 0, 1};               // average of the estimates
  float o[3] = {0, 0, 1};               // output: follows e, at most TDOA_TRK_MAX_RATE
  float since = 0;                      // s since the last usable estimate
  float cmd[3] = {0, 0, 0};             // command after the deadband
  float cmdS[3] = {0, 0, 0};            // ...then smoothed (TDOA_CMD_TAU): what's reported and used
  bool hasCmd = false;
} trk;

static float angDeg(const float a[3], const float b[3]) {
  float d = a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
  return acosf(std::max(-1.0f, std::min(1.0f, d))) * 180 / (float)PI;
}
static void normalise(float v[3]) {
  float n = sqrtf(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]) + 1e-30f;
  for (int c = 0; c < 3; c++) v[c] /= n;
}

static void trackStep(bool have, const float u[3], float quality, TdoaPacket &pkt) {
  const float dt = TRK_DT, alpha = 1 - expf(-dt / TDOA_TRK_TAU);
  const float maxStep = TDOA_TRK_MAX_RATE * dt * 180 / (float)PI;  // deg per packet
  bool use = have && quality >= TDOA_TRK_Q_MIN;
  if (use) {
    if (trk.state == 0) {  // (re)acquired: start where the estimate is
      for (int c = 0; c < 3; c++) trk.e[c] = trk.o[c] = u[c];
    } else {  // weaker fits count less
      float w = powf((quality - TDOA_TRK_Q_MIN) / (1 - TDOA_TRK_Q_MIN), TDOA_TRK_Q_POW);
      for (int c = 0; c < 3; c++) trk.e[c] += alpha * w * (u[c] - trk.e[c]);
      normalise(trk.e);
    }
    trk.state = 1;
    trk.since = 0;
  } else if (trk.state) {
    trk.since += dt;
    trk.state = trk.since >= TDOA_TRK_LOST_S ? 0 : 2;
  }
  if (trk.state) {
    float a = angDeg(trk.o, trk.e);
    float f = a > maxStep ? maxStep / a : 1;  // speed limit on the output
    for (int c = 0; c < 3; c++) trk.o[c] += f * (trk.e[c] - trk.o[c]);
    normalise(trk.o);
    if (!trk.hasCmd) {
      for (int c = 0; c < 3; c++) trk.cmd[c] = trk.cmdS[c] = trk.o[c];
      trk.hasCmd = true;
    }
  }
  pkt.trackState = trk.state;
  if (trk.state) {
    for (int c = 0; c < 3; c++) pkt.tu[c] = trk.o[c];
    float a = angDeg(trk.cmd, trk.o);
    if (a > TDOA_DEADBAND_DEG) {  // follow with slack: stay DEADBAND behind
      float f = 1 - TDOA_DEADBAND_DEG / a;
      for (int c = 0; c < 3; c++) trk.cmd[c] += f * (trk.o[c] - trk.cmd[c]);
      normalise(trk.cmd);
    }
  }
  if (trk.hasCmd) {
    float b = TDOA_CMD_TAU > 0 ? 1 - expf(-dt / TDOA_CMD_TAU) : 1;
    for (int c = 0; c < 3; c++) trk.cmdS[c] += b * (trk.cmd[c] - trk.cmdS[c]);
    normalise(trk.cmdS);
    for (int c = 0; c < 3; c++) pkt.cmd[c] = trk.cmdS[c];
  }
}

// ---------------------------------------------------------------- servos (mirrors host/tdoa_ref.py ServoCtl)

static const int SERVO_BITS = 14;
static const float SERVO_PERIOD_US = 20000;  // 50 Hz
static struct {
  int mode = TDOA_SERVO_ENABLE ? 1 : 0;      // 0 off, 1 automatic, 2 test
  bool moving = false;
  float pan = 0, tilt = 0;                   // commanded rates, rad/s
  float testPan = 0, testTilt = 0;
  float neutralPan = TDOA_PAN_NEUTRAL_US, neutralTilt = TDOA_TILT_NEUTRAL_US;
  bool attached = false;
} sv;

static void servoAttach(bool on) {  // detached = no pulses, so a continuous servo can't creep
  if (on == sv.attached) return;
  if (on) {
    ledcAttachPin(TDOA_PAN_PIN, 0);
    ledcAttachPin(TDOA_TILT_PIN, 1);
  } else {
    ledcDetachPin(TDOA_PAN_PIN);
    ledcDetachPin(TDOA_TILT_PIN);
  }
  sv.attached = on;
}

static float rateToUs(float rate, float neutral, float sign) {
  float off = sign * rate * TDOA_US_PER_RADS;
  if (fabsf(rate) > 1e-3f) off += (off > 0 ? 1 : -1) * TDOA_MIN_US;
  return neutral + off;
}

static void servoStep(TdoaPacket &pkt) {
  const float dt = TRK_DT;
  float tp = 0, tt = 0;  // target rates
  bool have = trk.state == 1 || (trk.state == 2 && trk.since < TDOA_COAST_STOP_S);
  if (sv.mode == 2) {
    tp = sv.testPan;
    tt = sv.testTilt;
  } else if (have) {
    float ex = pkt.tu[0], ey = pkt.tu[1];  // in-plane part of the tracked direction = pointing error
    float r = sqrtf(ex * ex + ey * ey);
    float err = asinf(std::min(1.0f, r));
    const float start = TDOA_START_DEG * (float)PI / 180, stop = TDOA_STOP_DEG * (float)PI / 180;
    if (!sv.moving && err > start) sv.moving = true;
    if (sv.moving && err < stop) sv.moving = false;
    if (sv.moving && r > 0) {
      float s = std::min(TDOA_MAX_RATE, TDOA_GAIN * (err - stop));
      float a = TDOA_TILT_UP_AZ_DEG * (float)PI / 180;
      float upx = cosf(a), upy = sinf(a);  // tilt-up direction in the array plane
      float rx = upy, ry = -upx;           // 90 deg clockwise from up; TDOA_PAN_SIGN sets which way + turns
      tt = s * (ex * upx + ey * upy) / r;
      tp = s * (ex * rx + ey * ry) / r;
    }
  } else {
    sv.moving = false;
  }
  const float dmax = TDOA_ACCEL * dt;  // acceleration limit
  sv.pan += std::max(-dmax, std::min(dmax, tp - sv.pan));
  sv.tilt += std::max(-dmax, std::min(dmax, tt - sv.tilt));
  float pu = rateToUs(sv.pan, sv.neutralPan, TDOA_PAN_SIGN), tu = rateToUs(sv.tilt, sv.neutralTilt, TDOA_TILT_SIGN);
  if (sv.mode == 0) {
    servoAttach(false);
  } else {
    servoAttach(true);
    ledcWrite(0, (uint32_t)(pu / SERVO_PERIOD_US * ((1 << SERVO_BITS) - 1)));
    ledcWrite(1, (uint32_t)(tu / SERVO_PERIOD_US * ((1 << SERVO_BITS) - 1)));
  }
  pkt.panRate = sv.pan;
  pkt.tiltRate = sv.tilt;
  pkt.panUs = pu;
  pkt.tiltUs = tu;
  pkt.moving = sv.moving;
  pkt.servoMode = sv.mode;
}

// Host commands, one per line: E0/E1 off/automatic, TP<r>/TT<r> test speed rad/s, NP<us>/NT<us> stop pulse, S stop
static void handleCommand(const char *c) {
  switch (c[0]) {
    case 'E': sv.mode = c[1] == '1' ? 1 : 0; sv.testPan = sv.testTilt = 0; break;
    case 'T':
      sv.mode = 2;
      if (c[1] == 'P') sv.testPan = atof(c + 2);
      if (c[1] == 'T') sv.testTilt = atof(c + 2);
      break;
    case 'N':
      if (c[1] == 'P') sv.neutralPan = atof(c + 2);
      if (c[1] == 'T') sv.neutralTilt = atof(c + 2);
      break;
    case 'S': sv.mode = 0; sv.testPan = sv.testTilt = 0; break;
  }
}

static void readCommands() {
  static char line[32];
  static int n = 0;
  while (Serial.available()) {
    char ch = Serial.read();
    if (ch == '\n' || ch == '\r') {
      line[n] = 0;
      if (n) handleCommand(line);
      n = 0;
    } else if (n < 31) {
      line[n++] = ch;
    }
  }
}

// ---------------------------------------------------------------- main

void setup() {
  Serial.setTxBufferSize(16384);
  Serial.begin();
  dsps_fft2r_init_fc32(nullptr, NFFT);
  dsps_wind_hann_f32(win, NFFT);
  k0 = lroundf(TDOA_TONE_HZ / BIN_HZ) - K_FLOOR;
  buildGrid();
  startCapture();
  ledcSetup(0, 50, SERVO_BITS);  // servo PWM channels (pins attached only when driving)
  ledcSetup(1, 50, SERVO_BITS);
}

void loop() {
  size_t n;
  i2s_read(I2S_NUM_0, bufA, BLK, &n, portMAX_DELAY);  // blocks until a block is ready
  i2s_read(I2S_NUM_1, bufB, BLK, &n, portMAX_DELAY);
  if (overflowed()) resync();  // a receiver lost data: the two streams may no longer line up
#ifdef SYNC_TEST
  static uint32_t pk = 0;
  if (++pk % 250 == 0) resync();  // every 2 s, to prove re-syncing keeps alignment
#endif
#ifdef TDOA_SIM
  simulate();
#endif
#ifdef TIMING
  uint32_t t0 = micros();
#endif
  appendSamples();

  TdoaPacket pkt = {{'T', 'D', 'O', 'A'}};
  pktCount++;
  if (pktCount - lastFix > (uint32_t)(TDOA_TRACK_HOLD_S * RATE / HOP)) {  // lost it: search around the axis again
    centre[0] = 0; centre[1] = 0; centre[2] = 1;
  }
  static int run = 0;
  static float lastFreq = 0;
  if (ringFill == NFFT) {
    spectra();
#ifdef TIMING
    uint32_t t1 = micros();
#endif
    float freq = 0, snr = 0;
    int k = detectTone(freq, snr);
    if (k >= 0 && run > 0 && fabsf(freq - lastFreq) > 2 * BIN_HZ) run = 0;
    run = k >= 0 ? run + 1 : 0;
    if (k >= 0) lastFreq = freq;
    if (run >= MIN_RUN) {
      pkt.freq = freq;
      pkt.snr = snr;
      direction(k, pkt);
    } else if (run == 0) {
      gValid = false;  // tone lost: restart the cross-spectrum average when it returns
    }
#ifdef TIMING
    pkt.tau[0] = t1 - t0;
    pkt.tau[1] = micros() - t1;
    pkt.tau[2] = micros() - t0;
#endif
  }
  {
    float u[3] = {pkt.ux, pkt.uy, pkt.uz};
    trackStep(pkt.freq > 0, u, pkt.quality, pkt);
  }
  readCommands();
  servoStep(pkt);
  pkt.resyncs = resyncs;
#ifdef TDOA_SIM
  pkt.simAz = simAz;
  pkt.simEl = simEl;
#endif

  Serial.write((const uint8_t *)"SYNC", 4);
  Serial.write((const uint8_t *)outA, BLK16);
  Serial.write((const uint8_t *)outB, BLK16);
  Serial.write((const uint8_t *)&pkt, sizeof(pkt));
}
