// Servos controlled from the Mac (host/servo_gui.py). Two FS90MR (continuous rotation): servo 1 on GPIO10,
// servo 2 on GPIO11.
// Serial commands, one per line: F<n> <us> forward, R<n> <us> reverse (n = servo 1 or 2, us = pulse offset from
// neutral, 0-800), S stops both. One servo runs at a time: a command for one stops the other.
// Each command is answered with "OK <n> <pulse us>" (S answers "OK 0 0"; pulse 0 = stopped, no pulses).
// Watchdog: both servos stop if no command arrives for WATCHDOG_MS, so nothing can run away if the program
// closes or crashes. The GUI repeats the command every 0.1 s while a button is held.
// LED: green = forward, blue = reverse, off = stopped.
#include <Arduino.h>

static const int PINS[] = {10, 11};   // servo 1, servo 2. LEDC channel = index (0 and 1 share a timer; both 50 Hz)
static const int N = sizeof(PINS) / sizeof(PINS[0]);
static const int BITS = 14;           // duty resolution
static const int PERIOD_US = 20000;   // 50 Hz
static const int NEUTRAL_US = 1500;   // FS90MR stop pulse (nominal)
static const int MAX_US = 800;        // largest offset accepted (FS90MR range 700-2300 us, datasheet, from memory)
static const int WATCHDOG_MS = 300;

static int pulseUs[N] = {0};          // pulse being sent per servo in us, 0 = none
static unsigned long lastCmd = 0;

static void led(uint8_t r, uint8_t g, uint8_t b) {
  neopixelWrite(21, g, r, b);         // this board's LED has red and green swapped
}

static void setPulse(int i, int us) {
  if (us == pulseUs[i]) return;
  pulseUs[i] = us;
  ledcWrite(i, us ? (uint32_t)((float)us / PERIOD_US * ((1 << BITS) - 1)) : 0);
}

static bool running() {
  for (int i = 0; i < N; i++) if (pulseUs[i]) return true;
  return false;
}

static void showLed() {
  int us = 0;
  for (int i = 0; i < N; i++) if (pulseUs[i]) us = pulseUs[i];
  if (us > NEUTRAL_US) led(0, 40, 0);
  else if (us) led(0, 0, 40);
  else led(0, 0, 0);
}

static void stopAll() {
  for (int i = 0; i < N; i++) setPulse(i, 0);
  showLed();
}

static void handle(const char *s) {
  if (s[0] == 'S') {
    stopAll();
    lastCmd = millis();
    Serial.printf("OK 0 0\n");
    return;
  }
  if (s[0] != 'F' && s[0] != 'R') return;
  int n, off;
  if (sscanf(s + 1, "%d %d", &n, &off) != 2 || n < 1 || n > N) return;
  off = constrain(off, 0, MAX_US);
  int us = off ? NEUTRAL_US + (s[0] == 'F' ? off : -off) : 0;
  for (int i = 0; i < N; i++) setPulse(i, i == n - 1 ? us : 0);   // one servo at a time
  showLed();
  lastCmd = millis();
  Serial.printf("OK %d %d\n", n, us);
}

void setup() {
  Serial.begin();
  for (int i = 0; i < N; i++) {
    ledcSetup(i, 50, BITS);
    ledcAttachPin(PINS[i], i);
    ledcWrite(i, 0);                  // no pulses until told
  }
  led(0, 0, 0);
}

void loop() {
  static char line[16];
  static int n = 0;
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      line[n] = 0;
      if (n) handle(line);
      n = 0;
    } else if (n < (int)sizeof(line) - 1) {
      line[n++] = c;
    }
  }
  if (running() && millis() - lastCmd > WATCHDOG_MS) stopAll();
}
