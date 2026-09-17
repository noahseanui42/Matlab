// servo_calibration_sweep.ino
// Bidirectional pulse-width sweep for the calibration rig (protractor +
// pointer + fixed camera). One servo at a time, direct PWM on D9/D10/D11
// (Arduino UNO R4 Minima, no PCA9685) — matches the wiring already used
// on the robot.
//
// At each commanded pulse width the servo holds, and this sketch prompts
// for the angle read off the camera photo. Each measurement is echoed as
// a plain "pulse_width_us,angle_deg" line with no prefix; every other
// line (prompts/status) starts with "# ". Capture the Serial Monitor
// output and keep only the un-prefixed lines to build
// cal_servo<N>_fwd.csv / cal_servo<N>_bwd.csv for calibrate_servos_lut.m.
//
// Serial commands (9600 baud, newline-terminated):
//   S1 / S2 / S3   select servo (pins D9/D10/D11), detaches the others
//   F              forward sweep: PW_MIN -> PW_MAX, NUM_POINTS steps
//   B              backward sweep: PW_MAX -> PW_MIN, NUM_POINTS steps
//   H              move selected servo to its center pulse width
//   P<us>          hold-out check point, e.g. P1650: move to that pulse width,
//                  prompt N_REPEATS times, log inside "# BEGIN HOLDOUT" /
//                  "# END HOLDOUT" markers. Use pulse widths NOT on the sweep
//                  grid; parse_sweep_log.m routes these to cal_servo<N>_holdout.csv
//                  for compare_calibrations.m (HANDOFF step 12).
//   Q              detach the selected servo (park it, no more holding torque)
//   ?              print this help
//
// During a sweep, at each point you'll be prompted N_REPEATS times for
// the measured angle — type the number read off the current photo and
// press enter each time.

#include <Servo.h>

const uint8_t SERVO_PINS[3] = {9, 10, 11};
const int PW_MIN = 830;     // us, matches init_scan.m pw_min
const int PW_MAX = 2170;    // us, matches init_scan.m pw_max
const uint8_t NUM_POINTS = 11;   // evenly spaced points per sweep direction
const uint8_t N_REPEATS  = 3;    // repeat measurements per point
const unsigned long SETTLE_MS = 800; // wait for the servo to stop moving before prompting

Servo servo;
int8_t activeServo = -1; // 0,1,2 -> SERVO_PINS index; -1 = none attached

void printHelp() {
  Serial.println(F("# S1/S2/S3 select servo, F forward sweep, B backward sweep, P<us> hold-out point, H home, Q detach, ? help"));
}

void selectServo(uint8_t idx) {
  if (servo.attached()) servo.detach();
  activeServo = idx;
  servo.attach(SERVO_PINS[idx]);
  Serial.print(F("# Selected servo "));
  Serial.print(idx + 1);
  Serial.print(F(" on pin "));
  Serial.println(SERVO_PINS[idx]);
}

// Blocks until a full line is available on Serial, returns it as a float.
float readMeasuredAngle() {
  while (!Serial.available()) { /* wait for the operator */ }
  String line = Serial.readStringUntil('\n');
  return line.toFloat();
}

void runPoint(int pw) {
  servo.writeMicroseconds(pw);
  delay(SETTLE_MS);
  for (uint8_t r = 1; r <= N_REPEATS; r++) {
    Serial.print(F("# PW="));
    Serial.print(pw);
    Serial.print(F(" rep "));
    Serial.print(r);
    Serial.print(F("/"));
    Serial.print(N_REPEATS);
    Serial.println(F(" -> enter measured angle (deg):"));

    float ang = readMeasuredAngle();
    Serial.print(pw);
    Serial.print(',');
    Serial.println(ang, 3);
  }
}

void runSweep(bool forward) {
  if (activeServo < 0) {
    Serial.println(F("# No servo selected - send S1/S2/S3 first"));
    return;
  }
  Serial.print(F("# BEGIN "));
  Serial.print(forward ? F("FORWARD") : F("BACKWARD"));
  Serial.print(F(" SWEEP servo="));
  Serial.println(activeServo + 1);

  for (uint8_t i = 0; i < NUM_POINTS; i++) {
    uint8_t step = forward ? i : (NUM_POINTS - 1 - i);
    int pw = PW_MIN + (long)(PW_MAX - PW_MIN) * step / (NUM_POINTS - 1);
    runPoint(pw);
  }

  Serial.println(F("# END SWEEP"));
}

void setup() {
  Serial.begin(9600);
  while (!Serial) { /* wait for USB serial on R4 */ }
  Serial.println(F("# Servo calibration sweep ready"));
  printHelp();
}

void loop() {
  if (!Serial.available()) return;
  String cmd = Serial.readStringUntil('\n');
  cmd.trim();
  cmd.toUpperCase();

  if (cmd == "S1") selectServo(0);
  else if (cmd == "S2") selectServo(1);
  else if (cmd == "S3") selectServo(2);
  else if (cmd == "F") runSweep(true);
  else if (cmd == "B") runSweep(false);
  else if (cmd == "H") {
    if (activeServo < 0) {
      Serial.println(F("# No servo selected - send S1/S2/S3 first"));
    } else {
      servo.writeMicroseconds((PW_MIN + PW_MAX) / 2);
      Serial.println(F("# Homed to center pulse width"));
    }
  }
  else if (cmd.startsWith("P") && cmd.length() > 1) {
    int pw = cmd.substring(1).toInt();
    if (activeServo < 0) {
      Serial.println(F("# No servo selected - send S1/S2/S3 first"));
    } else if (pw < PW_MIN || pw > PW_MAX) {
      Serial.print(F("# Pulse width out of range ("));
      Serial.print(PW_MIN); Serial.print('-'); Serial.print(PW_MAX);
      Serial.println(F(" us)"));
    } else {
      Serial.print(F("# BEGIN HOLDOUT servo="));
      Serial.println(activeServo + 1);
      runPoint(pw);
      Serial.println(F("# END HOLDOUT"));
    }
  }
  else if (cmd == "Q") {
    if (servo.attached()) servo.detach();
    activeServo = -1;
    Serial.println(F("# Detached"));
  }
  else if (cmd == "?") printHelp();
  else if (cmd.length() > 0) {
    Serial.print(F("# Unknown command: "));
    Serial.println(cmd);
    printHelp();
  }
}
