// Derived from grzesiek2201/Delta-Robot (GPL-3.0) @ 48a8038
// https://github.com/grzesiek2201/Delta-Robot
//
// Arduino glue only (HANDOFF §6.5): setup/loop, Servo, Serial, millis.
// Everything else (kinematics, framing, planning) lives in the
// Arduino-header-free, host-testable kinematics/protocol/motion modules.
#include <Servo.h>
#include "config.h"
#include "kinematics.h"
#include "protocol.h"
#include "motion.h"

namespace {

Motion motion;
Servo servos[3];

class GlueCallbacks : public ProtocolCallbacks {
 public:
  void onMove(const MovePoint &pt) override { motion.acceptManualMove(pt, millis()); }

  void onProgramPoint(const MovePoint &pt) override { motion.acceptProgramPoint(pt); }

  void onFunc(FuncType type, int ptNo, float value, int pin) override {
    (void)pin;
    if (type == FuncType::WAIT_TIME) {
      motion.setDwell(ptNo, value);
    }
    // WAIT_INPUT / SET_OUTPUT: v1 parses and stores nothing further; no pin
    // is ever written for these (HANDOFF §5).
  }

  void onStart(bool start) override { motion.setStart(start, millis()); }

  void onEnable(bool enabled) override {
    if (enabled) {
      motion.enable(millis());
      int us[3];
      motion.computeServoUs(us);
      for (int i = 0; i < 3; i++) {
        servos[i].attach(CAL[i].pin);
        // The R4 Servo library (ArduinoCore-renesas) ignores
        // writeMicroseconds() called before attach() -- servoIndex is still
        // unassigned, so the call is a silent no-op -- and attach() itself
        // always starts the pulse at DEFAULT_PULSE_WIDTH (1500us) before
        // returning. So the write has to come after attach(), to overwrite
        // that default as quickly as possible. 1500us is inside the
        // 830-2170us safe range, so the brief default pulse isn't a hazard,
        // just a documented quirk (see README).
        servos[i].writeMicroseconds(us[i]);
      }
    } else {
      for (int i = 0; i < 3; i++) {
        servos[i].detach();
      }
      motion.disable();
    }
  }

  void onJsonError() override { motion.setError(4); }

  void sendOK() override {
    Serial.print("OK");
    Serial.flush();
  }
};

GlueCallbacks callbacks;
Protocol proto(callbacks);

uint32_t tTick = 0;
uint32_t tTx = 0;

void writeServos() {
  int us[3];
  motion.computeServoUs(us);
  if (!motion.isEnabled()) {
    return;  // detached: nothing to write
  }
  for (int i = 0; i < 3; i++) {
    servos[i].writeMicroseconds(us[i]);
  }
}

void printStatusLine() {
  char buf[80];
  Protocol::buildStatus(buf, sizeof(buf), motion.theta(), motion.isMoving() ? 1 : 0,
                        motion.isRunning() ? 1 : 0, motion.isEnabled() ? 1 : 0, motion.error());
  Serial.println(buf);
}

}  // namespace

void setup() {
  Serial.begin(115200);
  // No while(!Serial): the GUI opens the port with dtr=0 and never waits for
  // a host-side terminal (HANDOFF §5 rule 4).
  // Servos are left unattached here: Motion boots DISABLED (HANDOFF §6.4),
  // and streaming starts immediately below regardless of enable state
  // (HANDOFF §5 rule 1) so the GUI always has a deg line to read.
  tTick = millis();
  tTx = millis();
}

void loop() {
  uint32_t now = millis();

  while (Serial.available()) {
    proto.feed((uint8_t)Serial.read(), now);
    now = millis();
  }

  now = millis();
  if (now - tTick >= TICK_MS) {
    tTick += TICK_MS;
    motion.tick(now);
    writeServos();
  }

  now = millis();
  if (!proto.rxBusy(now) && now - tTx >= STREAM_MS) {
    tTx = now;
    printStatusLine();
  }
}
