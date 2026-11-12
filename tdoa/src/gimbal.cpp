// Gimbal: the TDOA tone tracker (copied from main.cpp, unchanged up to and including the tracker) driving two FS90MR
// continuous-rotation servos: pan (GPIO10, about the vertical) and elevation (GPIO11). Settings: gimbal_config.h.
//
// Per 8 ms packet: capture, spectra, tone detection, TDOA direction and tracker as in main.cpp, then:
//   - motion compensation: each direction estimate is rotated by the gimbal's own motion over the estimation lag
//     (GIMBAL_EST_LAG_PKTS), and the tracker's state turns with the plate every packet, so the gimbal turning
//     doesn't look like the source moving
//   - tracking: the source's angle off the plate axis, split into elevation (towards mic 2) and pan (towards mic 4)
//     parts, gives the pose error; speed = GIMBAL_GAIN x pose error, capped, acceleration-limited, each axis with
//     start/stop hysteresis, clipped to the travel limits minus GIMBAL_LIMIT_MARGIN_DEG
//   - tone lost: hold position (GIMBAL_SEARCH_ENABLE 0), or hold, then search: listen at 3 pan stops in turn
//     (servos stopped), then park at the centre
//   - pose = dead reckoning (speed x time, speeds from the measured tables): no position feedback, so it drifts.
//     Elevation includes the worm gear's slack (GIMBAL_TILT_BACKLASH_DEG) on direction changes
//   - speed -> pulse through the speed tables. Below the slowest calibrated speed an axis runs at that speed or
//     gets no pulses at all (a stopped servo can't creep)
// Packet: as main.cpp ("SYNC" + audio + "TDOA" + 32 floats) + "GMBL" + 24 floats (GimbalPacket). The TDOA host
// scripts still read it (they skip the extra block); host/gstream.py reads both.
// When no program is reading the USB port, packets are dropped instead of blocking, so it runs without a host.
// gimbal_sim: closed-loop simulation (see simulate()); never drives the servo pins.

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
#include "gimbal_config.h"

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

struct GimbalPacket {
  char tag[4];            // "GMBL"
  float mode;             // 0 off (no pulses), 1 automatic, 2 jog, 3 raw pulses (calibration)
  float state;            // 0 idle, 1 tracking, 2 holding (tone lost, waiting), 3 searching, 4 parked
  float pan, tilt;        // estimated pose, deg (dead reckoning from home). + pan = FORWARD (towards mic 4)
  float panRate, tiltRate;  // deg/s the pulses give (speed tables), + = FORWARD
  float panUs, tiltUs;    // pulses sent, us (0 = none)
  float tgtPan, tgtTilt;  // pose being driven to, deg (inside the limits)
  float errPan, errTilt;  // pointing error per axis, deg (pan part scaled by cos elevation)
  float srcAz, srcEl;     // source direction in the world (deg), from the track and the estimated pose
  float limits;           // target clipped: 1 pan min, 2 pan max, 4 tilt min, 8 tilt max
  float calibrated;       // GIMBAL_CALIBRATED: automatic mode is refused until the speed tables are measured
  float searchPass;       // sweeps done in the current search
  float txDrops;          // packets dropped because no program was reading the USB port
  float simT, simPan, simTilt, simSrcAz, simSrcEl, simTone;  // gimbal_sim truth (true pose, world source), else 0
  float tiltSlack;        // elevation slack still to cross before the plate moves DOWN: 0 = engaged downwards,
                          // GIMBAL_TILT_BACKLASH_DEG = engaged upwards (as at home)
};
static_assert(sizeof(GimbalPacket) == 4 + 25 * 4, "no padding: host/gstream.py reads 4 + 25 floats");

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
  installBus(I2S_NUM_1, false, 8, 12, 4, &qB);  // slave on bus B pins (WS on GPIO12 since 2026-11-12); waits for clocks
#ifdef SYNC_TEST
  esp_rom_gpio_connect_in_signal(3, I2S1I_SD_IN_IDX, false);  // bus B receiver reads bus A's data line
#endif
  installBus(I2S_NUM_0, true, 5, 6, 3, &qA);   // master on bus A pins
  // Drive bus B's clock pins with the master's own SCK/WS (whatever signals the driver put on GPIO5/6)
  uint32_t bckSig = GPIO.func_out_sel_cfg[5].func_sel;
  uint32_t wsSig = GPIO.func_out_sel_cfg[6].func_sel;
  gpio_set_direction(GPIO_NUM_8, GPIO_MODE_INPUT_OUTPUT);  // keeps the slave's input routing on the pad
  gpio_set_direction(GPIO_NUM_12, GPIO_MODE_INPUT_OUTPUT);
  esp_rom_gpio_connect_out_signal(8, bckSig, false, false);
  esp_rom_gpio_connect_out_signal(12, wsSig, false, false);
  // The slave detects SCK edges a few module-clock cycles late, which put its sampling point right on the
  // mics' data transitions (falling SCK edge): bits slipped mid-sample. Inverting its SCK input moves the
  // sampling point half a bit away, into the stable part of each bit. It then reads every bit one position
  // late, which appendSamples undoes with << 1 (sync test: 100% of data bits match bus A after the shift).
  esp_rom_gpio_connect_in_signal(8, I2S1I_BCK_IN_IDX, true);
#ifdef SD_PULLUP
  gpio_pulldown_dis(GPIO_NUM_4);  // wiring check: an undriven bus B data line now reads all 1s (-1), not 0
  gpio_pullup_en(GPIO_NUM_4);
#endif
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

// ---------------------------------------------------------------- gimbal geometry
// Array-frame vectors are handled as components along U (MIC_POS azimuth GIMBAL_UP_AZ_DEG, mic 2: the way elevation
// FORWARD swings the axis), L (GIMBAL_PANFWD_AZ_DEG, mic 4: the way pan FORWARD swings it) and the axis A (+z). Only
// dot products with these are used, so it doesn't matter whether MIC_POS's frame is mirrored.
// World frame: x = the axis at home (pan 0, plate facing the horizon), z = up, y = where pan FORWARD swings the axis
// from home. At pan p, elevation e the plate's directions in world coordinates are
//   A = (cos e cos p, cos e sin p, sin e),  U = (-sin e cos p, -sin e sin p, cos e),  L = (-sin p, cos p, 0)
// (elevation turns the plate about L, pan about the vertical).

static const float D2R = (float)PI / 180, R2D = 180 / (float)PI;
static const float P_LO = GIMBAL_PAN_MIN_DEG + GIMBAL_LIMIT_MARGIN_DEG, P_HI = GIMBAL_PAN_MAX_DEG - GIMBAL_LIMIT_MARGIN_DEG;
static const float T_LO = GIMBAL_TILT_MIN_DEG + GIMBAL_LIMIT_MARGIN_DEG, T_HI = GIMBAL_TILT_MAX_DEG - GIMBAL_LIMIT_MARGIN_DEG;
static float axU[3], axL[3];  // U and L in the array frame (in-plane; L made exactly perpendicular to U)

static float clampf(float x, float lo, float hi) { return x < lo ? lo : x > hi ? hi : x; }

static void initAxes() {
  float a = GIMBAL_UP_AZ_DEG * D2R, b = GIMBAL_PANFWD_AZ_DEG * D2R;
  axU[0] = cosf(a); axU[1] = sinf(a); axU[2] = 0;
  float l0 = cosf(b), l1 = sinf(b), d = l0 * axU[0] + l1 * axU[1];
  l0 -= d * axU[0];
  l1 -= d * axU[1];
  float n = sqrtf(l0 * l0 + l1 * l1);
  axL[0] = l0 / n; axL[1] = l1 / n; axL[2] = 0;
}

static void plateAxes(float pan, float el, float A[3], float Uw[3], float Lw[3]) {
  float p = pan * D2R, e = el * D2R, cp = cosf(p), sp = sinf(p), ce = cosf(e), se = sinf(e);
  A[0] = ce * cp; A[1] = ce * sp; A[2] = se;
  Uw[0] = -se * cp; Uw[1] = -se * sp; Uw[2] = ce;
  Lw[0] = -sp; Lw[1] = cp; Lw[2] = 0;
}

static void toWorld(const float v[3], float pan, float el, float w[3]) {  // array frame -> world, at pose (pan, el)
  float A[3], Uw[3], Lw[3];
  plateAxes(pan, el, A, Uw, Lw);
  float cu = v[0] * axU[0] + v[1] * axU[1], cl = v[0] * axL[0] + v[1] * axL[1], ca = v[2];
  for (int c = 0; c < 3; c++) w[c] = cu * Uw[c] + cl * Lw[c] + ca * A[c];
}

static void toArray(const float w[3], float pan, float el, float v[3]) {  // world -> array frame, at pose (pan, el)
  float A[3], Uw[3], Lw[3];
  plateAxes(pan, el, A, Uw, Lw);
  float cu = 0, cl = 0, ca = 0;
  for (int c = 0; c < 3; c++) {
    cu += w[c] * Uw[c];
    cl += w[c] * Lw[c];
    ca += w[c] * A[c];
  }
  for (int c = 0; c < 2; c++) v[c] = cu * axU[c] + cl * axL[c];
  v[2] = ca;
}

static void reframe(float v[3], float pan0, float el0, float pan1, float el1) {  // a fixed world direction, as
  float w[3];                                                                   // seen after the plate moved
  toWorld(v, pan0, el0, w);
  toArray(w, pan1, el1, v);
}

// ---------------------------------------------------------------- gimbal control

#ifdef GIMBAL_SIM
static const bool IS_SIM = true;
#else
static const bool IS_SIM = false;
#endif
static const bool CAN_AUTO = GIMBAL_CALIBRATED || IS_SIM;  // automatic mode needs measured speeds
static const int SERVO_BITS = 14;
static const float SERVO_PERIOD_US = 20000;  // 50 Hz
static const uint32_t JOG_TIMEOUT_MS = 300, RAW_MAX_MS = 10000;
static const int HIST = 64;                  // pose history in packets, for GIMBAL_EST_LAG_PKTS
static_assert(GIMBAL_EST_LAG_PKTS < HIST, "pose history too short for GIMBAL_EST_LAG_PKTS");
static float hist[HIST][2];

static struct {
  int mode = GIMBAL_ENABLE && CAN_AUTO ? 1 : 0;  // 0 off (no pulses), 1 automatic, 2 jog, 3 raw pulses
  int state = 0;                  // 0 idle, 1 tracking, 2 holding, 3 searching, 4 parked
  float pan = 0, el = 0;          // estimated pose, deg (dead reckoning from home)
  float motT = 0;                 // elevation motor side, plate degrees: el - BACKLASH <= motT <= el
                                  // (motT = el: engaged upwards, as at home)
  float rp = 0, rt = 0;           // requested speeds after the acceleration limit, deg/s (+ = FORWARD)
  float ep = 0, et = 0;           // speeds the pulses give (speed tables), deg/s: integrated into the pose
  float usP = 0, usT = 0;         // pulses sent, us (0 = none)
  bool moving = false;            // start/stop hysteresis on the total pointing error
  bool movP = false, movT = false;  // which axes are driving
  float lostFor = 0;              // s since the track was lost
  float jogP = 0, jogT = 0;       // jog speeds, deg/s
  uint32_t jogMs = JOG_TIMEOUT_MS;  // jog stops this long after the last jog command
  float rawP = 0, rawT = 0;       // raw pulse offsets from neutral, us
  uint32_t cmdAt = 0;             // millis() of the last jog / raw command
  uint32_t rawMs = RAW_MAX_MS;    // raw pulses stop this long after the last raw command
  float tgtP = 0, tgtT = 0, errP = 0, errT = 0;
  float srcAz = 0, srcEl = 0;     // last source direction in the world, deg
  int limits = 0;
  int wp = 0, wpDir = 1, passes = 0;  // search: current stop, direction, sweeps done
  float dwell = 0;
  uint32_t txDrops = 0;
} gb;

// Pulse offset (us) for a wanted speed (deg/s, >= 0) from a speed table; got = the speed it really gives.
// Points that don't move, or aren't faster than the previous one, are skipped. Slower than the slowest usable
// point: run at that point (the hysteresis decides when to stop). Faster than the fastest: capped.
static float offsetFor(float want, const float *dps, float &got) {
  float pu = 0, pv = 0;
  bool have = false;
  for (int i = 0; i < GIMBAL_CAL_N; i++) {
    float u = GIMBAL_CAL_US[i], v = dps[i];
    if (v <= (have ? pv : 0)) continue;
    if (want <= v) {
      if (!have) {
        got = v;
        return u;
      }
      got = want;
      return pu + (want - pv) / (v - pv) * (u - pu);
    }
    pu = u;
    pv = v;
    have = true;
  }
  got = pv;
  return have ? pu : 0;
}

// Wanted speed (deg/s, + = FORWARD) -> pulse (us, 0 = none); eff = the speed that pulse gives
static float pulseFor(float rate, float neutral, float sign, const float *fwd, const float *rev, float &eff) {
  eff = 0;
  if (rate == 0) return 0;
  float got, off = offsetFor(fabsf(rate), rate > 0 ? fwd : rev, got);
  if (off <= 0) return 0;
  eff = rate > 0 ? got : -got;
  return neutral + (rate > 0 ? sign : -sign) * off;
}

static void writePulse(int ch, float us) {
#ifndef GIMBAL_SIM  // the simulation never drives the real servos
  ledcWrite(ch, us > 0 ? (uint32_t)(us / SERVO_PERIOD_US * ((1 << SERVO_BITS) - 1)) : 0);
#endif
}

// Tracking, from the source's angle relative to the plate (tracked direction u, array frame): its angle along U
// is the elevation error; its angle along L, divided by cos(elevation), the pan error (pan moves the axis cos(el)
// deg per deg). The estimated pose is only used for that scaling and the limits, so its drift doesn't bend the aim.
// Start when the total pointing error exceeds GIMBAL_START_DEG, stop once it's under GIMBAL_STOP_DEG.
static void aim(const float u[3], float &wantP, float &wantT) {
  float aT = atan2f(u[0] * axU[0] + u[1] * axU[1], u[2]) * R2D;
  float aL = atan2f(u[0] * axL[0] + u[1] * axL[1], u[2]) * R2D;
  float ce = std::max(cosf(gb.el * D2R), 0.3f);  // caps the pan gain near the zenith (x 3.3)
  float tp = clampf(gb.pan + aL / ce, P_LO, P_HI), tt = clampf(gb.el + aT, T_LO, T_HI);
  gb.limits = (gb.pan + aL / ce < P_LO ? 1 : 0) | (gb.pan + aL / ce > P_HI ? 2 : 0) | (gb.el + aT < T_LO ? 4 : 0) |
              (gb.el + aT > T_HI ? 8 : 0);
  float dp = tp - gb.pan, dt = tt - gb.el, ep = dp * ce;
  gb.tgtP = tp; gb.tgtT = tt; gb.errP = ep; gb.errT = dt;
  float err = sqrtf(ep * ep + dt * dt);  // total pointing error (both axes)
  if (!gb.moving && err > GIMBAL_START_DEG) gb.moving = true;
  if (gb.moving && err < GIMBAL_STOP_DEG) gb.moving = false;
  gb.movP = gb.moving && fabsf(ep) > 1;  // an axis with almost no error stays put (its slowest speed would overshoot)
  gb.movT = gb.moving && fabsf(dt) > 1;
  wantP = gb.movP ? clampf(GIMBAL_GAIN * dp, -GIMBAL_MAX_DPS, GIMBAL_MAX_DPS) : 0;
  wantT = gb.movT ? clampf(GIMBAL_GAIN * dt, -GIMBAL_MAX_DPS, GIMBAL_MAX_DPS) : 0;
}

// Search moves: to a pose within 1 deg, at up to GIMBAL_SEARCH_DPS. True once there and stopped.
static bool goTo(float tp, float tt, float &wantP, float &wantT) {
  float dp = tp - gb.pan, dt = tt - gb.el;
  gb.tgtP = tp; gb.tgtT = tt; gb.errP = dp; gb.errT = dt; gb.limits = 0;
  wantP = fabsf(dp) > 1 ? clampf(GIMBAL_GAIN * dp, -GIMBAL_SEARCH_DPS, GIMBAL_SEARCH_DPS) : 0;
  wantT = fabsf(dt) > 1 ? clampf(GIMBAL_GAIN * dt, -GIMBAL_SEARCH_DPS, GIMBAL_SEARCH_DPS) : 0;
  return wantP == 0 && wantT == 0 && gb.rp == 0 && gb.rt == 0;
}

// Listening stops at pan -GIMBAL_SEARCH_PAN_DEG, 0, +GIMBAL_SEARCH_PAN_DEG: the array hears its whole front
// hemisphere, so these cover the travel without sweeping it end to end (every degree travelled adds drift)
static int nStops() { return 3; }
static float stopPan(int i) { return clampf((i - 1) * GIMBAL_SEARCH_PAN_DEG, P_LO, P_HI); }

static void startSearch() {
  int n = nStops(), best = 0;
  float from = clampf(gb.srcAz, P_LO, P_HI);  // start at the stop nearest where the source was last seen...
  for (int i = 1; i < n; i++)
    if (fabsf(stopPan(i) - from) < fabsf(stopPan(best) - from)) best = i;
  gb.wp = best;
  gb.wpDir = best < (n - 1) / 2.0f ? 1 : -1;  // ...then head for the far end
  gb.passes = 0;
  gb.dwell = 0;
  gb.state = 3;
}

static void searchStep(float dt, float &wantP, float &wantT) {
  float el = clampf(GIMBAL_SEARCH_EL_DEG, T_LO, T_HI);
  if (gb.state == 4) {  // parked at the centre, listening
    goTo(clampf(0, P_LO, P_HI), el, wantP, wantT);
    return;
  }
  if (!goTo(stopPan(gb.wp), el, wantP, wantT)) return;
  if ((gb.dwell += dt) < GIMBAL_SEARCH_DWELL_S) return;  // listen with the servos stopped
  gb.dwell = 0;
  if (gb.wp + gb.wpDir < 0 || gb.wp + gb.wpDir >= nStops()) {  // reached an end stop: one pass done
    gb.wpDir = -gb.wpDir;
    if (++gb.passes >= GIMBAL_SEARCH_PASSES) {
      gb.state = 4;
      return;
    }
  }
  gb.wp += gb.wpDir;
}

static void autoStep(float dt, float &wantP, float &wantT) {
  bool have = trk.state == 1 || (trk.state == 2 && trk.since < GIMBAL_COAST_STOP_S);
  if (have) {
    float s[3];  // for the display and where to start a search
    toWorld(trk.o, gb.pan, gb.el, s);
    gb.srcAz = atan2f(s[1], s[0]) * R2D;
    gb.srcEl = asinf(clampf(s[2], -1, 1)) * R2D;
    gb.state = 1;
    gb.lostFor = 0;
    aim(trk.o, wantP, wantT);
    return;
  }
  gb.moving = gb.movP = gb.movT = false;
  if (trk.state == 0) gb.lostFor += dt;
  if (gb.state == 3 || gb.state == 4) {
    searchStep(dt, wantP, wantT);
  } else if (GIMBAL_SEARCH_ENABLE && trk.state == 0 && gb.lostFor >= GIMBAL_SEARCH_DELAY_S) {
    startSearch();
    searchStep(dt, wantP, wantT);
  } else {
    gb.state = 2;  // holding: stopped, waiting for the tone
  }
}

static void gimbalStep(TdoaPacket &pkt, GimbalPacket &g) {
  const float dt = TRK_DT;
  uint32_t now = millis();
  if (gb.mode == 2 && now - gb.cmdAt > gb.jogMs) gb.mode = 0;
  if (gb.mode == 3 && now - gb.cmdAt > gb.rawMs) gb.mode = 0;
  float wantP = 0, wantT = 0;
  if (gb.mode == 1) {
    autoStep(dt, wantP, wantT);
  } else {
    gb.state = 0;
    gb.moving = gb.movP = gb.movT = false;
    gb.lostFor = 0;
    gb.tgtP = gb.pan; gb.tgtT = gb.el; gb.errP = gb.errT = 0; gb.limits = 0;
    if (gb.mode == 2) {  // jog: no limits (manual, and the estimate may be wrong, e.g. while calibrating)
      wantP = gb.jogP;
      wantT = gb.jogT;
    }
  }
  if (gb.mode == 1) {
    const float dmax = GIMBAL_ACCEL_DPS2 * dt;
    gb.rp += clampf(wantP - gb.rp, -dmax, dmax);
    gb.rt += clampf(wantT - gb.rt, -dmax, dmax);
  } else if (gb.mode == 2) {  // jog: straight to the asked speed, as a manual control should
    gb.rp = wantP;
    gb.rt = wantT;
  } else {
    gb.rp = gb.rt = 0;
  }
  if (gb.mode == 1) {  // automatic: never further past a limit
    if ((gb.rp > 0 && gb.pan >= P_HI) || (gb.rp < 0 && gb.pan <= P_LO)) gb.rp = 0;
    if ((gb.rt > 0 && gb.el >= T_HI) || (gb.rt < 0 && gb.el <= T_LO)) gb.rt = 0;
  }
  float effP = 0, effT = 0;
  if (gb.mode == 3) {  // calibration: raw pulse offsets, not counted in the pose
    gb.usP = gb.rawP != 0 ? GIMBAL_PAN_NEUTRAL_US + GIMBAL_PAN_FWD_SIGN * gb.rawP : 0;
    gb.usT = gb.rawT != 0 ? GIMBAL_TILT_NEUTRAL_US + GIMBAL_TILT_FWD_SIGN * gb.rawT : 0;
  } else {
    gb.usP = pulseFor(gb.rp, GIMBAL_PAN_NEUTRAL_US, GIMBAL_PAN_FWD_SIGN, GIMBAL_PAN_FWD_DPS, GIMBAL_PAN_REV_DPS, effP);
    gb.usT = pulseFor(gb.rt, GIMBAL_TILT_NEUTRAL_US, GIMBAL_TILT_FWD_SIGN, GIMBAL_TILT_FWD_DPS, GIMBAL_TILT_REV_DPS, effT);
  }
  writePulse(0, gb.usP);
  writePulse(1, gb.usT);

  // Dead reckoning. The tracker's directions (array frame) turn with the plate.
  float p0 = gb.pan, e0 = gb.el;
  gb.pan += effP * dt;
  gb.motT += effT * dt;  // elevation: the plate only moves once the motor has crossed the worm gear's slack
  if (gb.motT > gb.el) gb.el = gb.motT;
  else if (gb.motT < gb.el - GIMBAL_TILT_BACKLASH_DEG) gb.el = gb.motT + GIMBAL_TILT_BACKLASH_DEG;
  if (gb.pan != p0 || gb.el != e0) {
    reframe(trk.e, p0, e0, gb.pan, gb.el);
    reframe(trk.o, p0, e0, gb.pan, gb.el);
    reframe(trk.cmd, p0, e0, gb.pan, gb.el);
    reframe(trk.cmdS, p0, e0, gb.pan, gb.el);
    reframe(centre, p0, e0, gb.pan, gb.el);
  }
  gb.ep = effP;
  gb.et = effT;

  g.mode = gb.mode; g.state = gb.state; g.pan = gb.pan; g.tilt = gb.el;
  g.panRate = effP; g.tiltRate = effT; g.panUs = gb.usP; g.tiltUs = gb.usT;
  g.tgtPan = gb.tgtP; g.tgtTilt = gb.tgtT; g.errPan = gb.errP; g.errTilt = gb.errT;
  g.srcAz = gb.srcAz; g.srcEl = gb.srcEl; g.limits = gb.limits; g.calibrated = GIMBAL_CALIBRATED;
  g.searchPass = gb.passes; g.txDrops = gb.txDrops; g.tiltSlack = GIMBAL_TILT_BACKLASH_DEG - (gb.el - gb.motT);
  pkt.panRate = effP * D2R; pkt.tiltRate = effT * D2R; pkt.panUs = gb.usP; pkt.tiltUs = gb.usT;
  pkt.moving = gb.movP || gb.movT; pkt.servoMode = gb.mode;
}

// LED: off = off, green = tracking (red = tracking but the source is outside the travel), yellow = holding,
// blue = searching, purple = parked, white = jog / calibration
static void showState() {
  static int last = -1;
  int code = gb.mode == 0 ? 0 : gb.mode >= 2 ? 5 : gb.state == 1 ? (gb.limits ? 6 : 1) : gb.state;
  if (code == last) return;
  last = code;
  static const uint8_t RGB[7][3] = {{0, 0, 0}, {0, 40, 0}, {30, 30, 0}, {0, 0, 40}, {20, 0, 20}, {20, 20, 20}, {40, 0, 0}};
  neopixelWrite(21, RGB[code][1], RGB[code][0], RGB[code][2]);  // this board's LED has red and green swapped
}

// ---------------------------------------------------------------- closed-loop simulation (gimbal_sim)
#ifdef GIMBAL_SIM
// A scripted source in the world (scenario) is heard by a simulated array sitting on the gimbal at its TRUE pose.
// The true pose integrates the speeds the board commands with per-direction speed errors (SIM_ERR_*: an assumed
// residual error of the calibrated tables, not a measured one), so the dead-reckoned pose drifts from it. The plate blocks sound from behind.
// The elevation has SIM_TILT_BACKLASH of worm-gear slack. Servo dynamics (spin-up, coasting) are not modelled. "R" restarts the scenario with both poses at home.
static const float SIM_ERR_PF = 0.05f, SIM_ERR_PR = -0.03f, SIM_ERR_TF = -0.05f, SIM_ERR_TR = 0.05f;
static const float SIM_TILT_BACKLASH = 6.0f;  // true worm-gear slack in the sim (deg), assumed
static const float SCEN_S = 70;
static float simT = 0, simPan = 0, simTilt = 0, simMotT = 0, simSrcAz = 0, simSrcEl = 0, simAz = 0, simEl = 0;
static bool simTone = false;

static void scenario(float t, float &az, float &el, bool &on) {
  on = true;
  if (t < 10) { az = 40; el = 25; }                                      // still: acquire and settle
  else if (t < 20) { az = 40 - 9 * (t - 10); el = 25 + 2 * (t - 10); }   // moving 9 deg/s in az, 2 in el
  else if (t < 26) { az = -50; el = 45; on = false; }                    // tone off: hold, then search
  else if (t < 36) { az = -70; el = 15; }                                // back on elsewhere
  else if (t < 46) { az = 150; el = 30; }                                // behind: outside the pan range
  else if (t < 56) { az = -20; el = 82; }                                // near the zenith
  else if (t < 66) { az = 10; el = 2; }                                  // near the horizon (below the margin)
  else { az = 10; el = 2; on = false; }
}

static void simulate() {
  static float ph = 0;
  static uint32_t lcg = 12345;
  const float dt = (float)HOP / RATE;
  simT += dt;
  if (simT >= SCEN_S) simT -= SCEN_S;
  scenario(simT, simSrcAz, simSrcEl, simTone);
  simPan += gb.ep * (1 + (gb.ep > 0 ? SIM_ERR_PF : SIM_ERR_PR)) * dt;
  simMotT += gb.et * (1 + (gb.et > 0 ? SIM_ERR_TF : SIM_ERR_TR)) * dt;
  if (simMotT > simTilt) simTilt = simMotT;
  else if (simMotT < simTilt - SIM_TILT_BACKLASH) simTilt = simMotT + SIM_TILT_BACKLASH;
  float a = simSrcAz * D2R, e = simSrcEl * D2R;
  float s[3] = {cosf(e) * cosf(a), cosf(e) * sinf(a), sinf(e)}, u[3];
  toArray(s, simPan, simTilt, u);
  simAz = fmodf(atan2f(u[1], u[0]) * R2D + 360, 360);
  simEl = asinf(clampf(u[2], -1, 1)) * R2D;
  float gain = simTone && u[2] > -0.1f ? 1 : 0;
  float off[4];
  for (int m = 0; m < 4; m++)  // geometric phase, minus each mic's configured extra delay (as real mics would have)
    off[m] = OMEGA * (MIC_POS[m][0] * u[0] + MIC_POS[m][1] * u[1] + MIC_POS[m][2] * u[2]) / TDOA_SPEED - OMEGA * 1e-6f * MIC_DELAY_US[m];
  const float dph = OMEGA / RATE, amp = 0.01f * 2147483647.0f * gain, nz = 1e-4f * 2147483647.0f / 2147483648.0f;
  int32_t *ba = (int32_t *)bufA, *bb = (int32_t *)bufB;
  for (int f = 0; f < HOP; f++) {
    float v[4];
    for (int m = 0; m < 4; m++) {
      lcg = lcg * 1664525u + 1013904223u;
      v[m] = amp * sinf(ph + off[m]) + nz * (int32_t)lcg;
    }
    for (int m = 0; m < 4; m++) {  // each mic into its slot; bus B pre-shifted to undo appendSamples' << 1
      int sl = MIC_SLOT[m];
      (sl < 2 ? ba : bb)[2 * f + (sl & 1)] = sl < 2 ? (int32_t)v[m] : (int32_t)v[m] >> 1;
    }
    ph += dph;
    if (ph > 2 * (float)PI) ph -= 2 * (float)PI;
  }
}
#endif

// ---------------------------------------------------------------- host commands
// One per line:
//   E1 automatic (refused until GIMBAL_CALIBRATED), E0 or S off (no pulses)
//   H  pose = home (put the gimbal there by hand first: pan centred, plate facing the horizon)
//   JP<deg/s> / JT<deg/s>  jog one axis (+ = FORWARD), counted in the pose, no limits; stops 0.3 s after the
//                          last one, so repeat to keep moving
//   NP<deg/s> <ms> / NT..  as J, for exactly that long (board-timed step; calibration fine adjustment)
//   CP<us> / CT<us>  raw pulse offset from neutral (+ = FORWARD, 0 = stop) for calibration: not counted in the
//                    pose, no limits, 10 s at most
//   MP<us> / MT<us>  as C, but stops 0.3 s after the last one (repeat while held; currently unused)
//   R  (gimbal_sim only) restart the scenario, both poses at home
static void setHome() {  // home is zeroed with the elevation slack taken up for moving up
  gb.pan = gb.el = gb.motT = 0;
  for (int i = 0; i < HIST; i++) hist[i][0] = hist[i][1] = 0;
}

static void handleCommand(const char *c) {
  switch (c[0]) {
    case 'E':
      if (c[1] != '1') gb.mode = 0;
      else if (CAN_AUTO) {
        gb.mode = 1;
        gb.state = 2;
        gb.lostFor = 0;
      }
      break;
    case 'S': gb.mode = 0; break;
    case 'H': setHome(); break;
    case 'J':
    case 'N': {
      if (c[1] != 'P' && c[1] != 'T') break;
      char *end;
      float v = strtof(c + 2, &end);
      gb.jogP = c[1] == 'P' ? v : 0;
      gb.jogT = c[1] == 'T' ? v : 0;
      gb.jogMs = c[0] == 'N' ? (uint32_t)clampf(strtof(end, nullptr), 1, 5000) : JOG_TIMEOUT_MS;
      gb.mode = 2;
      gb.cmdAt = millis();
      break;
    }
    case 'C':
    case 'M':
      if (c[1] != 'P' && c[1] != 'T') break;
      gb.rawP = c[1] == 'P' ? clampf(atof(c + 2), -800, 800) : 0;
      gb.rawT = c[1] == 'T' ? clampf(atof(c + 2), -800, 800) : 0;
      gb.mode = gb.rawP != 0 || gb.rawT != 0 ? 3 : 0;
      gb.rawMs = c[0] == 'M' ? JOG_TIMEOUT_MS : RAW_MAX_MS;
      gb.cmdAt = millis();
      break;
#ifdef GIMBAL_SIM
    case 'R':
      simT = 0;
      simPan = simTilt = simMotT = 0;
      setHome();
      break;
#endif
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
  initAxes();
  startCapture();
  ledcSetup(0, 50, SERVO_BITS);
  ledcSetup(1, 50, SERVO_BITS);
#ifndef GIMBAL_SIM
  ledcAttachPin(GIMBAL_PAN_PIN, 0);
  ledcAttachPin(GIMBAL_TILT_PIN, 1);
#endif
  writePulse(0, 0);  // no pulses until told
  writePulse(1, 0);
  neopixelWrite(21, 0, 0, 0);
}

void loop() {
  size_t n;
  i2s_read(I2S_NUM_0, bufA, BLK, &n, portMAX_DELAY);  // blocks until a block is ready
  i2s_read(I2S_NUM_1, bufB, BLK, &n, portMAX_DELAY);
  if (overflowed()) resync();  // a receiver lost data: the two streams may no longer line up
#ifdef GIMBAL_SIM
  simulate();
#endif
  appendSamples();

  TdoaPacket pkt = {{'T', 'D', 'O', 'A'}};
  GimbalPacket g = {{'G', 'M', 'B', 'L'}};
  pktCount++;
  hist[pktCount % HIST][0] = gb.pan;  // pose while this packet's audio arrived
  hist[pktCount % HIST][1] = gb.el;
  if (pktCount - lastFix > (uint32_t)(TDOA_TRACK_HOLD_S * RATE / HOP)) {  // lost it: search around the axis again
    centre[0] = 0; centre[1] = 0; centre[2] = 1;
  }
  static int run = 0;
  static float lastFreq = 0;
  if (ringFill == NFFT) {
    spectra();
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
  }
  {
    float u[3] = {pkt.ux, pkt.uy, pkt.uz};
    bool have = pkt.freq > 0;
    if (have && GIMBAL_EST_LAG_PKTS > 0 && pktCount > GIMBAL_EST_LAG_PKTS) {  // the estimate describes the pose
      const float *h = hist[(pktCount - GIMBAL_EST_LAG_PKTS) % HIST];         // GIMBAL_EST_LAG_PKTS ago
      reframe(u, h[0], h[1], gb.pan, gb.el);
    }
    trackStep(have, u, pkt.quality, pkt);
  }
  readCommands();
  gimbalStep(pkt, g);
  showState();
  pkt.resyncs = resyncs;
#ifdef GIMBAL_SIM
  pkt.simAz = simAz;
  pkt.simEl = simEl;
  g.simT = simT; g.simPan = simPan; g.simTilt = simTilt;
  g.simSrcAz = simSrcAz; g.simSrcEl = simSrcEl; g.simTone = simTone;
#endif

  // Never block: with nobody reading the port, a full buffer drops the packet instead
  const int total = 4 + 2 * BLK16 + sizeof(pkt) + sizeof(g);
  if (!Serial || Serial.availableForWrite() >= total) {
    Serial.write((const uint8_t *)"SYNC", 4);
    Serial.write((const uint8_t *)outA, BLK16);
    Serial.write((const uint8_t *)outB, BLK16);
    Serial.write((const uint8_t *)&pkt, sizeof(pkt));
    Serial.write((const uint8_t *)&g, sizeof(g));
  } else {
    gb.txDrops++;
  }
}
