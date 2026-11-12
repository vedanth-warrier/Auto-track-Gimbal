// Servo oscillation test, FS90MR (continuous rotation). GPIO10 (GPIO12 is the mics' bus B WS). Runs from every boot:
// left for A_MS, stop, right for A_MS, stop, repeat. Raise A_MS (and reflash) until each swing is 90 deg.
// No position feedback: the angle of each swing = speed x A_MS. Start with the servo at the end of its travel
// that the first (left) swing moves away from; flip LEFT_SIGN if the first swing goes into the end stop.
// LED: red = turning left, blue = turning right, off = stopped.
// No serial output on purpose: with the Mac plugged in but no program reading, prints block and the servo
// would keep running at the last pulse.
#include <Arduino.h>

static const int PINS[] = {10};       // GPIO12 is now the mics' bus B WS line: never drive it
static const int CH = 0;              // LEDC (PWM) channel
static const int BITS = 14;           // duty resolution
static const int PERIOD_US = 20000;   // 50 Hz
static const int A_MS = 1000;          // drive time per swing ("a"). The servo sees whole 20 ms pulses: step in 20s
static const int SPEED_US = 800;      // pulse offset from neutral = speed (about 500 = full speed)
static const int NEUTRAL_US = 1500;   // FS90MR stop pulse (nominal). If the swings drift one way, adjust this
static const int LEFT_SIGN = -1;      // -1: left = pulse below neutral. Flip to +1 if left/right are swapped
static const int PAUSE_MS = 500;      // stopped between swings

static void pulse(int us) {           // 0 = no pulses: the servo stops dead
  ledcWrite(CH, us ? (uint32_t)((float)us / PERIOD_US * ((1 << BITS) - 1)) : 0);
}

static void led(uint8_t r, uint8_t g, uint8_t b) {
  neopixelWrite(21, g, r, b);         // this board's LED has red and green swapped
}

static void swing(int sign, uint8_t r, uint8_t b) {
  led(r, 0, b);
  pulse(NEUTRAL_US + sign * SPEED_US);
  delay(A_MS);
  pulse(0);
  led(0, 0, 0);
  delay(PAUSE_MS);
}

void setup() {
  ledcSetup(CH, 50, BITS);
  for (int p : PINS) ledcAttachPin(p, CH);
  pulse(0);
  delay(1000);
}

void loop() {
  swing(LEFT_SIGN, 40, 0);    // left, LED red
  swing(-LEFT_SIGN, 0, 40);   // right, LED blue
}
