// Servo test on GPIO12: 0 deg, then 90 deg, then release.

#include <Arduino.h>

static const int PIN = 12;
static const int CH = 0;       // LEDC (PWM) channel
static const int BITS = 14;    // duty resolution
static const int PERIOD_US = 20000;  // 50 Hz

static void angle(float deg, float min_us = 600, float max_us = 2400) {
  deg = constrain(deg, 0, 180);
  float us = min_us + (max_us - min_us) * deg / 180;
  ledcWrite(CH, (uint32_t)(us / PERIOD_US * ((1 << BITS) - 1)));
}

void setup() {
  ledcSetup(CH, 50, BITS);
  ledcAttachPin(PIN, CH);
  angle(0);
  delay(1000);
  angle(90);
  delay(1000);
  ledcDetachPin(PIN);  // stop pulses; servo goes limp
}

void loop() {}
