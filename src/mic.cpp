// Reads both I2S buses and streams "SYNC" + bus A block + bus B block over USB.
// Same packet format as the old MicroPython mic.py, so host/plot.py is unchanged.

#include <Arduino.h>
#include <driver/i2s.h>

static const int RATE = 16000;
static const int BLK = 2048;  // bytes per bus per packet (256 stereo frames)

static uint8_t bufA[BLK];
static uint8_t bufB[BLK];

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

void setup() {
  Serial.setTxBufferSize(16384);
  Serial.begin();
  startBus(I2S_NUM_0, 5, 6, 3);  // bus A: mics 1, 2
  startBus(I2S_NUM_1, 8, 7, 4);  // bus B: mics 3, 4
}

void loop() {
  size_t n;
  i2s_read(I2S_NUM_0, bufA, BLK, &n, portMAX_DELAY);  // blocks until a block is ready
  i2s_read(I2S_NUM_1, bufB, BLK, &n, portMAX_DELAY);
  Serial.write((const uint8_t *)"SYNC", 4);
  Serial.write(bufA, BLK);
  Serial.write(bufB, BLK);
}
