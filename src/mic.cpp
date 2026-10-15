// Reads both I2S buses, then every packet (256 new samples per mic, 62.5 per second):
//   1. streams raw audio: "SYNC" + bus A block + bus B block (unchanged, for plot/scope/peak.py)
//   2. per-mic amplitude: RMS in a frequency band, from a 1024-point FFT
//   3. direction vector: amplitude-weighted sum of the directions the mics face
//   4. streams "DIRV" + 4 amplitudes + x + y (little-endian float32), for vector.py

#include <Arduino.h>
#include <driver/i2s.h>
#include <esp_dsp.h>
#include <math.h>

static const int RATE = 16000;
static const int BLK = 2048;     // bytes per bus per packet (256 stereo frames)
static const int HOP = BLK / 8;  // new samples per mic per packet (256)
static const int NFFT = 1024;    // FFT length: 64 ms of audio, 15.6 Hz per bin

// ---- Tuning: set these from the directivity sweep and calibration ----
static const float BAND_LO = 2000;  // Hz, band the amplitude is measured in
static const float BAND_HI = 6000;  // Hz
static const float GAIN[4] = {1, 1, 1, 1};         // per-mic gain correction
static const float AZ_DEG[4] = {0, 0, 180, 180};  // direction each mic faces around the vertical (mic 1..4)

static uint8_t bufA[BLK];
static uint8_t bufB[BLK];
static float hist[4][NFFT];      // last NFFT samples per mic, oldest first, full-scale units (1.0 = full scale)
// ESP-DSP's optimised FFT needs 16-byte aligned buffers; unaligned gave garbage (~+24 dBFS on every mic)
static float win[NFFT] __attribute__((aligned(16)));          // Hann window
static float fftbuf[2 * NFFT] __attribute__((aligned(16)));   // interleaved re, im
static float winPow;             // sum of win^2, for Parseval scaling
static float dirX[4], dirY[4];   // unit vectors each mic faces
static int kLo, kHi;             // FFT bins of the band

struct DirPacket {
  char tag[4];
  float amp[4];  // band RMS per mic, full-scale units
  float x, y;    // direction vector, length 0..1
};
static_assert(sizeof(DirPacket) == 28, "no padding: vector.py reads 4 + 6 floats");

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

// 2. RMS of one mic's signal within BAND_LO..BAND_HI
static float bandRms(int m) {
  float mean = 0;
  for (int n = 0; n < NFFT; n++) mean += hist[m][n];
  mean /= NFFT;
  for (int n = 0; n < NFFT; n++) {
    fftbuf[2 * n] = (hist[m][n] - mean) * win[n];
    fftbuf[2 * n + 1] = 0;
  }
  dsps_fft2r_fc32(fftbuf, NFFT);
  dsps_bit_rev_fc32(fftbuf, NFFT);
  float p = 0;
  for (int k = kLo; k <= kHi; k++) p += fftbuf[2 * k] * fftbuf[2 * k] + fftbuf[2 * k + 1] * fftbuf[2 * k + 1];
  return sqrtf(2 * p / (NFFT * winPow));  // Parseval, corrected for the window
}

// 3. Direction vector: each mic pulls towards the way it faces, in proportion to its amplitude
static void direction(const float amp[4], float &x, float &y) {
  float sum = 0;
  x = y = 0;
  for (int m = 0; m < 4; m++) {
    x += amp[m] * dirX[m];
    y += amp[m] * dirY[m];
    sum += amp[m];
  }
  if (sum > 0) {
    x /= sum;
    y /= sum;
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
  kLo = (int)ceilf(BAND_LO * NFFT / RATE);
  kHi = (int)floorf(BAND_HI * NFFT / RATE);
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

  DirPacket pkt = {{'D', 'I', 'R', 'V'}};
  for (int m = 0; m < 4; m++) pkt.amp[m] = GAIN[m] * bandRms(m);
  direction(pkt.amp, pkt.x, pkt.y);

  // 4. Output
  Serial.write((const uint8_t *)"SYNC", 4);
  Serial.write(bufA, BLK);
  Serial.write(bufB, BLK);
  Serial.write((const uint8_t *)&pkt, sizeof(pkt));
}
