// Derived from grzesiek2201/Delta-Robot (GPL-3.0) @ 48a8038
// https://github.com/grzesiek2201/Delta-Robot
//
// Framing (<mode>, <{json}>, <#>) is ported from delta_main_code.ino's
// recvWithStartEndMarkers()/readMode()/deserializeSerial(); the dispatch
// table and reply rules are HANDOFF §5's protocol contract, which the
// upstream .ino only partially matches (it drove steppers, not this
// firmware's motion state machine).
#include "protocol.h"
#include "config.h"
#include <ArduinoJson.h>
#include <stdio.h>

namespace {

MovePoint parseMovePoint(JsonDocument &doc) {
  MovePoint pt;
  pt.n = doc["n"] | 0;
  pt.i = doc["i"] | 0;
  pt.v = doc["v"] | 0;
  pt.a = doc["a"] | 0;
  JsonArrayConst c = doc["c"];
  pt.c[0] = c[0] | 0.0f;
  pt.c[1] = c[1] | 0.0f;
  pt.c[2] = c[2] | 0.0f;
  return pt;
}

}  // namespace

void Protocol::feed(uint8_t b, uint32_t nowMs) {
  lastByteMs_ = nowMs;
  haveLastByte_ = true;

  const char c = static_cast<char>(b);

  if (!inFrame_) {
    if (c == '<') {
      inFrame_ = true;
      len_ = 0;
      overflow_ = false;
    }
    return;
  }

  if (c == '>') {
    inFrame_ = false;
    if (!overflow_) {
      buf_[len_] = '\0';
      handleToken();
    }
    len_ = 0;
    overflow_ = false;
    return;
  }

  if (len_ >= kBufCap) {
    overflow_ = true;  // frame too long (HANDOFF §5 rule 5): discard on '>'
    return;
  }
  buf_[len_++] = c;
}

bool Protocol::rxBusy(uint32_t nowMs) {
  if (rxBusy_ && haveLastByte_ && (nowMs - lastByteMs_) >= RX_IDLE_TIMEOUT_MS) {
    rxBusy_ = false;
  }
  return rxBusy_;
}

void Protocol::handleToken() {
  if (len_ == 1 && buf_[0] >= '0' && buf_[0] <= '9') {
    mode_ = buf_[0];
    rxBusy_ = true;
    return;
  }

  if (len_ == 1 && buf_[0] == '#') {
    rxBusy_ = false;
    return;
  }

  if (len_ > 0 && buf_[0] == '{') {
    if (mode_ == 0) {
      return;  // payload with no mode pending: ignore (HANDOFF §5 rule 5)
    }
    dispatchPayload();
    mode_ = 0;
    return;
  }

  // anything else: ignore
}

void Protocol::dispatchPayload() {
  JsonDocument doc;
  DeserializationError err = deserializeJson(doc, buf_, len_);
  if (err) {
    cb_.onJsonError();
    return;
  }

  switch (mode_) {
    case '0': {
      bool start = doc["start"] | false;
      cb_.onStart(start);
      break;
    }
    case '1': {
      MovePoint pt = parseMovePoint(doc);
      cb_.onProgramPoint(pt);
      cb_.sendOK();
      break;
    }
    case '2': {
      MovePoint pt = parseMovePoint(doc);
      cb_.onMove(pt);
      break;
    }
    // pt_no/value/pin_no may arrive as JSON numbers or numeric strings (Tk
    // treeview values, HANDOFF §5 field notes) -- .as<T>() coerces either,
    // where the `|` default-value operator would silently accept only the
    // number form and fall back to 0 for a string.
    case '3': {
      int ptNo = doc["pt_no"].as<int>();
      float value = doc["value"].as<float>();
      cb_.onFunc(FuncType::WAIT_TIME, ptNo, value, 0);
      cb_.sendOK();
      break;
    }
    case '4': {
      int ptNo = doc["pt_no"].as<int>();
      float value = doc["value"].as<float>();
      int pin = doc["pin_no"].as<int>();
      cb_.onFunc(FuncType::WAIT_INPUT, ptNo, value, pin);
      cb_.sendOK();
      break;
    }
    case '5': {
      int ptNo = doc["pt_no"].as<int>();
      float value = doc["value"].as<float>();
      int pin = doc["pin_no"].as<int>();
      cb_.onFunc(FuncType::SET_OUTPUT, ptNo, value, pin);
      cb_.sendOK();
      break;
    }
    case '8': {
      int enableField = doc["enable"] | 1;
      cb_.onEnable(enableField == 0);  // inverted: enable:0 means ENABLE
      break;
    }
    default:
      // '6' (calibrate/home), '7' (gripper), '9' (SD interpolation): ignored
      // in v1 (HANDOFF §5).
      break;
  }
}

void Protocol::buildStatus(char *buf, size_t bufLen, const float thetaDeg[3], int mv, int run, int en, int e) {
  snprintf(buf, bufLen, "{\"deg\":[%.2f,%.2f,%.2f],\"mv\":%d,\"run\":%d,\"en\":%d,\"e\":%d}",
           (double)thetaDeg[0], (double)thetaDeg[1], (double)thetaDeg[2], mv, run, en, e);
}
