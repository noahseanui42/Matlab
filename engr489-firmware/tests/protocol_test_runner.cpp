// Host test runner for protocol.{h,cpp} (HANDOFF §9.3). Built and run by
// test_protocol.py. Exits non-zero (with a message on stderr) on the first
// failed check; prints "ALL PASS" and exits 0 otherwise.
#include "../delta_servo/protocol.h"
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <vector>

namespace {

int g_failures = 0;

#define CHECK(cond)                                                           \
  do {                                                                        \
    if (!(cond)) {                                                            \
      fprintf(stderr, "FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond);         \
      g_failures++;                                                           \
    }                                                                         \
  } while (0)

struct Call {
  std::string kind;
  MovePoint pt;
  FuncType funcType = FuncType::WAIT_TIME;
  int ptNo = 0;
  float value = 0;
  int pin = 0;
  bool boolArg = false;
};

class RecordingCallbacks : public ProtocolCallbacks {
 public:
  std::vector<Call> calls;
  int okCount = 0;

  void onMove(const MovePoint &pt) override {
    Call c;
    c.kind = "move";
    c.pt = pt;
    calls.push_back(c);
  }
  void onProgramPoint(const MovePoint &pt) override {
    Call c;
    c.kind = "programPoint";
    c.pt = pt;
    calls.push_back(c);
  }
  void onFunc(FuncType type, int ptNo, float value, int pin) override {
    Call c;
    c.kind = "func";
    c.funcType = type;
    c.ptNo = ptNo;
    c.value = value;
    c.pin = pin;
    calls.push_back(c);
  }
  void onStart(bool start) override {
    Call c;
    c.kind = "start";
    c.boolArg = start;
    calls.push_back(c);
  }
  void onEnable(bool enabled) override {
    Call c;
    c.kind = "enable";
    c.boolArg = enabled;
    calls.push_back(c);
  }
  void onJsonError() override {
    Call c;
    c.kind = "jsonError";
    calls.push_back(c);
  }
  void sendOK() override { okCount++; }
};

// Feeds a whole string byte-by-byte at a single point in time.
void feedString(Protocol &p, const std::string &s, uint32_t nowMs) {
  for (char ch : s) {
    p.feed(static_cast<uint8_t>(ch), nowMs);
  }
}

void test_move_dispatch() {
  RecordingCallbacks cb;
  Protocol p(cb);
  feedString(p, "<2><{\"n\":0,\"i\":0,\"v\":1,\"a\":1,\"c\":[0.0,0.0,-600.0]}>", 0);
  CHECK(cb.calls.size() == 1);
  CHECK(cb.calls[0].kind == "move");
  CHECK(cb.calls[0].pt.n == 0);
  CHECK(cb.calls[0].pt.v == 1);
  CHECK(cb.calls[0].pt.c[2] == -600.0f);
  CHECK(cb.okCount == 0);  // mode '2' replies "none"
}

void test_program_upload_then_wait_ok_sequence_and_rxbusy() {
  RecordingCallbacks cb;
  Protocol p(cb);
  uint32_t t = 0;

  // Three program points, no <#> between them.
  feedString(p, "<1><{\"n\":0,\"i\":1,\"v\":5,\"a\":5,\"c\":[10.0,0.0,-600.0]}>", t++);
  CHECK(p.rxBusy(t) == true);
  feedString(p, "<1><{\"n\":1,\"i\":1,\"v\":5,\"a\":5,\"c\":[20.0,0.0,-600.0]}>", t++);
  CHECK(p.rxBusy(t) == true);
  feedString(p, "<1><{\"n\":2,\"i\":1,\"v\":5,\"a\":5,\"c\":[30.0,0.0,-600.0]}>", t++);
  CHECK(p.rxBusy(t) == true);

  // Wait function with the GUI's literal spacing: `{"pt_no": 2, "value": 1000, "pin_no": 0}`.
  feedString(p, "<3><{\"pt_no\": 2, \"value\": 1000, \"pin_no\": 0}>", t++);
  CHECK(p.rxBusy(t) == true);  // still busy: no <#> yet

  CHECK(cb.okCount == 4);
  CHECK(cb.calls.size() == 4);
  CHECK(cb.calls[0].kind == "programPoint" && cb.calls[0].pt.n == 0);
  CHECK(cb.calls[1].kind == "programPoint" && cb.calls[1].pt.n == 1);
  CHECK(cb.calls[2].kind == "programPoint" && cb.calls[2].pt.n == 2);
  CHECK(cb.calls[3].kind == "func" && cb.calls[3].ptNo == 2);

  feedString(p, "<#>", t++);
  CHECK(p.rxBusy(t) == false);
}

void test_start_with_space() {
  RecordingCallbacks cb;
  Protocol p(cb);
  feedString(p, "<0><{\"start\": true}>", 0);
  feedString(p, "<#>", 1);
  CHECK(cb.calls.size() == 1);
  CHECK(cb.calls[0].kind == "start");
  CHECK(cb.calls[0].boolArg == true);
}

void test_enable_zero_means_enabled() {
  RecordingCallbacks cb;
  Protocol p(cb);
  feedString(p, "<8><{\"enable\": 0}>", 0);
  feedString(p, "<#>", 1);
  CHECK(cb.calls.size() == 1);
  CHECK(cb.calls[0].kind == "enable");
  CHECK(cb.calls[0].boolArg == true);  // enable:0 -> enabled
}

void test_sd_interpolate_no_dispatch() {
  RecordingCallbacks cb;
  Protocol p(cb);
  feedString(p, "<9><#>", 0);
  CHECK(cb.calls.empty());
  CHECK(cb.okCount == 0);
}

void test_numeric_string_value() {
  RecordingCallbacks cb;
  Protocol p(cb);
  feedString(p, "<3><{\"pt_no\": \"2\", \"value\": \"1000\", \"pin_no\": \"0\"}>", 0);
  CHECK(cb.calls.size() == 1);
  CHECK(cb.calls[0].kind == "func");
  CHECK(cb.calls[0].ptNo == 2);
  CHECK(cb.calls[0].value == 1000.0f);
  CHECK(cb.okCount == 1);
}

void test_oversized_frame_discarded_no_crash() {
  RecordingCallbacks cb;
  Protocol p(cb);
  std::string junk = "<2><" + std::string(200, 'x') + ">";
  feedString(p, junk, 0);
  CHECK(cb.calls.empty());
  CHECK(cb.okCount == 0);

  // Protocol must still work normally afterwards.
  feedString(p, "<0><{\"start\": true}>", 1);
  CHECK(cb.calls.size() == 1);
  CHECK(cb.calls[0].kind == "start");
}

void test_rxbusy_timeout() {
  RecordingCallbacks cb;
  Protocol p(cb);
  feedString(p, "<2>", 0);  // sets rxBusy, no payload yet
  CHECK(p.rxBusy(0) == true);
  CHECK(p.rxBusy(999) == true);
  CHECK(p.rxBusy(1000) == false);  // 1000ms with no bytes received
}

void test_json_error_sets_e4_via_callback() {
  RecordingCallbacks cb;
  Protocol p(cb);
  feedString(p, "<2><{not valid json>", 0);
  CHECK(cb.calls.size() == 1);
  CHECK(cb.calls[0].kind == "jsonError");
}

void test_payload_with_no_mode_ignored() {
  RecordingCallbacks cb;
  Protocol p(cb);
  feedString(p, "<{\"start\": true}>", 0);
  CHECK(cb.calls.empty());
  CHECK(cb.okCount == 0);
}

}  // namespace

int main() {
  test_move_dispatch();
  test_program_upload_then_wait_ok_sequence_and_rxbusy();
  test_start_with_space();
  test_enable_zero_means_enabled();
  test_sd_interpolate_no_dispatch();
  test_numeric_string_value();
  test_oversized_frame_discarded_no_crash();
  test_rxbusy_timeout();
  test_json_error_sets_e4_via_callback();
  test_payload_with_no_mode_ignored();

  if (g_failures == 0) {
    printf("ALL PASS\n");
    return 0;
  }
  fprintf(stderr, "%d failure(s)\n", g_failures);
  return 1;
}
