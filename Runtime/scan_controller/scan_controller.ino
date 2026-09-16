// scan_controller.ino — drives the delta robot's 3 bicep servos on command.
//
// Serial protocol (115200 baud, newline-terminated):
//   Host -> "H"                              home: park all 3 servos at PW_HOME
//   Host -> "M,<idx>,<pw1>,<pw2>,<pw3>"       move to this row's pulse widths
//   Arduino -> "OK,HOME" or "OK,<idx>"        once the move has settled
//   Arduino -> "ERR,<reason>"                 bad command, nothing moved
//
// Direct-pin PWM out D9/D10/D11 on an Arduino UNO R4 Minima — no PCA9685
// shield, matching the actual build (the shield only appears in the older
// system block diagram).

#include <Servo.h>

Servo servo1, servo2, servo3;

const int PIN1 = 9, PIN2 = 10, PIN3 = 11;

// Must match pw_home in init_scan.m
const int PW_HOME[3] = {1500, 1500, 1500};

// How long to wait after commanding a move before treating it as settled.
// This is a first guess — tune it against how fast these servos actually
// move once the arm is assembled. Err generous rather than reading the
// magnetometer mid-swing.
const unsigned long SETTLE_MS = 400;

void setup() {
  Serial.begin(115200);
  servo1.attach(PIN1);
  servo2.attach(PIN2);
  servo3.attach(PIN3);
  goHome();
}

void loop() {
  if (Serial.available()) {
    String line = Serial.readStringUntil('\n');
    line.trim();
    if (line.length() > 0) {
      handleCommand(line);
    }
  }
}

void goHome() {
  servo1.writeMicroseconds(PW_HOME[0]);
  servo2.writeMicroseconds(PW_HOME[1]);
  servo3.writeMicroseconds(PW_HOME[2]);
  delay(SETTLE_MS);
  Serial.println("OK,HOME");
}

void handleCommand(const String &line) {
  if (line == "H") {
    goHome();
    return;
  }

  if (line.charAt(0) != 'M') {
    Serial.println("ERR,unknown command");
    return;
  }

  int idx, pw1, pw2, pw3;
  int parsed = sscanf(line.c_str(), "M,%d,%d,%d,%d", &idx, &pw1, &pw2, &pw3);
  if (parsed != 4) {
    Serial.println("ERR,parse failure");
    return;
  }

  servo1.writeMicroseconds(pw1);
  servo2.writeMicroseconds(pw2);
  servo3.writeMicroseconds(pw3);
  delay(SETTLE_MS);

  Serial.print("OK,");
  Serial.println(idx);
}
