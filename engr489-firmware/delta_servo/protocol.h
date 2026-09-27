// Frame parser + dispatcher for the GUI's serial protocol (HANDOFF §5, §6.3).
// No Arduino headers: must compile with plain g++ so it is host-testable
// (tests/test_protocol.cpp). The glue layer (delta_servo.ino) feeds bytes in
// from Serial and implements ProtocolCallbacks to reach motion.{h,cpp}.
#pragma once

#include <stdint.h>
#include <stddef.h>

struct MovePoint {
  int n = 0;
  int i = 0;      // 0=Joint, 1=Linear, 2=Circular (treated as Joint)
  int v = 0;      // 1-10, percent/10; 0 means "not yet set" -> V_DEFAULT
  int a = 0;      // 1-10
  float c[3] = {0, 0, 0};  // effector centre xyz, mm, base frame
};

enum class FuncType { WAIT_TIME, WAIT_INPUT, SET_OUTPUT };

// Callbacks the parser dispatches into once a frame's JSON payload has been
// read. Implemented by the glue layer (delta_servo.ino) in production and by
// a recording fake in tests/test_protocol.cpp.
class ProtocolCallbacks {
 public:
  virtual ~ProtocolCallbacks() = default;
  virtual void onMove(const MovePoint &pt) = 0;           // mode '2'
  virtual void onProgramPoint(const MovePoint &pt) = 0;   // mode '1'
  virtual void onFunc(FuncType type, int ptNo, float value, int pin) = 0;  // modes '3','4','5'
  virtual void onStart(bool start) = 0;                   // mode '0'
  virtual void onEnable(bool enabled) = 0;                // mode '8' (inverted: enable:0 -> enabled=true)
  virtual void onJsonError() = 0;                         // malformed payload JSON
  virtual void sendOK() = 0;                              // glue prints "OK", no newline, then flushes
};

class Protocol {
 public:
  static constexpr size_t kBufCap = 128;

  explicit Protocol(ProtocolCallbacks &cb) : cb_(cb) {}

  // Feed one byte received from Serial. nowMs is the caller's millis().
  void feed(uint8_t b, uint32_t nowMs);

  // True while a message is mid-transit: streaming should pause. Also clears
  // rxBusy internally once RX_IDLE_TIMEOUT_MS has passed with no bytes fed.
  bool rxBusy(uint32_t nowMs);

  // Builds "{"deg":[..],"mv":..,"run":..,"en":..,"e":..}\n" into buf (must be
  // able to hold at least 64 bytes). deg values get 2 decimal places.
  static void buildStatus(char *buf, size_t bufLen, const float thetaDeg[3], int mv, int run, int en, int e);

 private:
  ProtocolCallbacks &cb_;
  char buf_[kBufCap + 1] = {0};
  size_t len_ = 0;
  bool inFrame_ = false;
  bool overflow_ = false;
  char mode_ = 0;  // 0 = none pending
  bool rxBusy_ = false;
  uint32_t lastByteMs_ = 0;
  bool haveLastByte_ = false;

  void handleToken();
  void dispatchPayload();
};
