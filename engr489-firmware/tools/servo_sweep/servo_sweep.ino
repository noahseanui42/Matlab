// Bench tool for calibration Step 1: drive one servo to a raw pulse width.
// Standalone -- flash this instead of delta_servo, then reflash delta_servo
// afterwards. Serial Monitor at 115200, line ending "Newline".
//
// Commands:
//   <pin> <us>   e.g. "10 1360"  attach pin (9/10/11) and hold that pulse
//   <us>         e.g. "1520"     new pulse on the last pin used
//   n / p        next / previous step in the sweep 1200..1840..1200
//   off          detach every servo (arms go limp)
#include <Servo.h>

const int PINS[3] = {9, 10, 11};
const int US_MIN = 830, US_MAX = 2170;  // same safe range as config.h
const int SWEEP[] = {1200, 1360, 1520, 1680, 1840, 1680, 1520, 1360, 1200};
const int SWEEP_LEN = sizeof(SWEEP) / sizeof(SWEEP[0]);

Servo servos[3];
int curIdx = -1;  // index into PINS, -1 = none selected yet
int step = -1;

int pinToIdx(int pin) {
  for (int i = 0; i < 3; i++) if (PINS[i] == pin) return i;
  return -1;
}

void drive(int us) {
  if (curIdx < 0) { Serial.println("pick a pin first, e.g. \"9 1520\""); return; }
  us = constrain(us, US_MIN, US_MAX);
  if (!servos[curIdx].attached()) servos[curIdx].attach(PINS[curIdx]);
  servos[curIdx].writeMicroseconds(us);  // after attach(): R4 ignores it before
  Serial.print("D"); Serial.print(PINS[curIdx]);
  Serial.print(" -> "); Serial.print(us); Serial.println(" us");
}

void setup() {
  Serial.begin(115200);
  Serial.println("servo_sweep ready: \"<pin> <us>\", \"<us>\", n, p, off");
}

void loop() {
  if (!Serial.available()) return;
  String line = Serial.readStringUntil('\n');
  line.trim();
  if (line.length() == 0) return;

  if (line == "off") {
    for (int i = 0; i < 3; i++) servos[i].detach();
    Serial.println("all detached");
  } else if (line == "n" || line == "p") {
    step = (line == "n") ? step + 1 : step - 1;
    step = constrain(step, 0, SWEEP_LEN - 1);
    Serial.print("step "); Serial.print(step + 1); Serial.print("/"); Serial.print(SWEEP_LEN);
    Serial.print(step < SWEEP_LEN / 2 ? " (up)  " : " (down) ");
    drive(SWEEP[step]);
  } else {
    int sp = line.indexOf(' ');
    if (sp > 0) {
      int idx = pinToIdx(line.substring(0, sp).toInt());
      if (idx < 0) { Serial.println("pin must be 9, 10 or 11"); return; }
      curIdx = idx;
      step = -1;
      drive(line.substring(sp + 1).toInt());
    } else {
      drive(line.toInt());
    }
  }
}
