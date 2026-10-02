// Host test runner for motion.{h,cpp} (HANDOFF §9.4). Built and run by
// test_motion.py. Uses a fake clock: every call takes an explicit nowMs, so
// no real time ever passes.
#include "../delta_servo/motion.h"
#include "../delta_servo/kinematics.h"
#include "../delta_servo/config.h"
#include <cmath>
#include <cstdio>

namespace {

int g_failures = 0;

#define CHECK(cond)                                                    \
  do {                                                                 \
    if (!(cond)) {                                                     \
      fprintf(stderr, "FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond);  \
      g_failures++;                                                    \
    }                                                                  \
  } while (0)

#define CHECK_NEAR(a, b, tol)                                                          \
  do {                                                                                 \
    double _a = (a), _b = (b), _tol = (tol);                                           \
    if (std::fabs(_a - _b) > _tol) {                                                   \
      fprintf(stderr, "FAIL %s:%d: %s ~= %s (%.6f vs %.6f)\n", __FILE__, __LINE__,      \
              #a, #b, _a, _b);                                                         \
      g_failures++;                                                                    \
    }                                                                                  \
  } while (0)

MovePoint makeMove(int n, int i, int v, int a, float x, float y, float z) {
  MovePoint pt;
  pt.n = n;
  pt.i = i;
  pt.v = v;
  pt.a = a;
  pt.c[0] = x;
  pt.c[1] = y;
  pt.c[2] = z;
  return pt;
}

uint32_t expectedDurationMs(const float xyz0[3], const float xyz1[3], const float theta0[3],
                            const float theta1[3], int v) {
  float dx = xyz1[0] - xyz0[0], dy = xyz1[1] - xyz0[1], dz = xyz1[2] - xyz0[2];
  float dist = sqrtf(dx * dx + dy * dy + dz * dz);
  float maxDelta = 0;
  for (int i = 0; i < 3; i++) {
    float d = fabsf(theta1[i] - theta0[i]);
    if (d > maxDelta) maxDelta = d;
  }
  int vEff = (v > 0) ? v : V_DEFAULT;
  float speed = SPEED_PER_V * (float)vEff;
  float t1 = dist / speed * 1000.0f;
  float t2 = maxDelta / MAX_JOINT_DPS * 1000.0f;
  float dur = t1;
  if (t2 > dur) dur = t2;
  if (dur < (float)T_MIN_MS) dur = (float)T_MIN_MS;
  return (uint32_t)(dur + 0.5f);
}

void test_smoothstep_and_joint_mode_sync() {
  Motion m;
  m.enable(0);
  m.setApproachEnabled(false);  // base planner; approach tested separately
  float theta0[3] = {m.theta()[0], m.theta()[1], m.theta()[2]};
  float xyz0[3] = {m.xyz()[0], m.xyz()[1], m.xyz()[2]};

  float target[3] = {0, 0, -650};
  float theta1[3];
  CHECK(ik(target, theta1));

  m.acceptManualMove(makeMove(0, 0, 5, 5, target[0], target[1], target[2]), 1000);
  CHECK(m.state() == MotionState::MOVING);

  uint32_t dur = expectedDurationMs(xyz0, target, theta0, theta1, 5);

  m.tick(1000);  // u=0
  CHECK_NEAR(m.theta()[0], theta0[0], 1e-4);
  CHECK_NEAR(m.theta()[1], theta0[1], 1e-4);
  CHECK_NEAR(m.theta()[2], theta0[2], 1e-4);

  m.tick(1000 + dur / 2);  // u=0.5 -> smoothstep(0.5) == 0.5 exactly
  for (int i = 0; i < 3; i++) {
    float expected = theta0[i] + 0.5f * (theta1[i] - theta0[i]);
    CHECK_NEAR(m.theta()[i], expected, 0.05);
  }
  CHECK(m.state() == MotionState::MOVING);

  m.tick(1000 + dur);  // u=1 -> all three arms land exactly on target, same tick
  CHECK(m.state() == MotionState::IDLE);
  for (int i = 0; i < 3; i++) {
    CHECK_NEAR(m.theta()[i], theta1[i], 1e-4);
  }
  CHECK_NEAR(m.xyz()[0], target[0], 1e-4);
  CHECK_NEAR(m.xyz()[1], target[1], 1e-4);
  CHECK_NEAR(m.xyz()[2], target[2], 1e-4);
}

void test_t_formula_v_default() {
  Motion m;
  m.enable(0);
  m.setApproachEnabled(false);  // base planner; approach tested separately
  float xyz0[3] = {m.xyz()[0], m.xyz()[1], m.xyz()[2]};
  float theta0[3] = {m.theta()[0], m.theta()[1], m.theta()[2]};
  float target[3] = {0, 0, -650};
  float theta1[3];
  CHECK(ik(target, theta1));

  uint32_t durWithVDefault = expectedDurationMs(xyz0, target, theta0, theta1, V_DEFAULT);

  m.acceptManualMove(makeMove(0, 0, /*v=*/0, 5, target[0], target[1], target[2]), 0);
  // Not finished just before the V_DEFAULT-implied duration...
  m.tick(durWithVDefault - 1);
  CHECK(m.state() == MotionState::MOVING);
  // ...but finished at it, proving v=0 was treated as V_DEFAULT.
  m.tick(durWithVDefault);
  CHECK(m.state() == MotionState::IDLE);
}

void test_t_min_floor() {
  Motion m;
  m.enable(0);
  // Same point commanded again: zero distance, zero joint delta -> duration
  // must still floor at T_MIN_MS, not collapse to 0.
  float target[3] = {m.xyz()[0], m.xyz()[1], m.xyz()[2]};
  m.acceptManualMove(makeMove(0, 0, 5, 5, target[0], target[1], target[2]), 0);
  CHECK(m.state() == MotionState::MOVING);
  m.tick(T_MIN_MS - 1);
  CHECK(m.state() == MotionState::MOVING);
  m.tick(T_MIN_MS);
  CHECK(m.state() == MotionState::IDLE);
}

void test_linear_mode_intermediate_on_segment() {
  Motion m;
  m.enable(0);
  m.setApproachEnabled(false);  // base planner; approach tested separately
  float xyz0[3] = {m.xyz()[0], m.xyz()[1], m.xyz()[2]};
  float target[3] = {50, -30, -600};

  m.acceptManualMove(makeMove(0, /*i=*/1, 3, 3, target[0], target[1], target[2]), 0);
  CHECK(m.state() == MotionState::MOVING);

  // Sample partway through; xyz() must lie on the xyz0->target segment.
  m.tick(50);
  float u_guess = -1;
  for (int axis = 0; axis < 3; axis++) {
    float span = target[axis] - xyz0[axis];
    if (fabsf(span) > 1e-3) {
      u_guess = (m.xyz()[axis] - xyz0[axis]) / span;
      break;
    }
  }
  CHECK(u_guess > -0.5f);  // found a usable axis
  for (int axis = 0; axis < 3; axis++) {
    float expected = xyz0[axis] + u_guess * (target[axis] - xyz0[axis]);
    CHECK_NEAR(m.xyz()[axis], expected, 0.1);
  }
}

void test_linear_mode_unreachable_sets_e2_and_holds() {
  Motion m;
  m.enable(0);
  m.setApproachEnabled(false);  // base planner; approach tested separately
  // Re-derived for the real calibrated CAL limits (-20/70 deg, confirmed
  // with the full 3-arm assembly). These points are lateral extremes just
  // to exercise the IK-failure code path; they don't need to be a
  // physically sensible probe position for this unit test.
  float a[3] = {141, -218, -560};
  float b[3] = {-120, -190, -560};
  float thetaA[3];
  CHECK(ik(a, thetaA));
  float thetaB[3];
  CHECK(ik(b, thetaB));
  // Confirm the segment's midpoint really is unreachable (fixture sanity).
  float mid[3] = {(a[0] + b[0]) / 2, (a[1] + b[1]) / 2, (a[2] + b[2]) / 2};
  float thetaMid[3];
  CHECK(!ik(mid, thetaMid));

  m.acceptManualMove(makeMove(0, 0, 5, 5, a[0], a[1], a[2]), 0);
  uint32_t tFinishA = 0;
  for (uint32_t t = 0; t <= 20000; t += (uint32_t)TICK_MS) {
    m.tick(t);
    if (m.state() == MotionState::IDLE) {
      tFinishA = t;
      break;
    }
  }
  CHECK(m.state() == MotionState::IDLE);  // (joint-mode) move to A completed
  CHECK_NEAR(m.xyz()[0], a[0], 0.01);

  float xyzAtA[3] = {m.xyz()[0], m.xyz()[1], m.xyz()[2]};
  float thetaAtA[3] = {m.theta()[0], m.theta()[1], m.theta()[2]};

  m.acceptManualMove(makeMove(0, /*i=*/1, 5, 5, b[0], b[1], b[2]), tFinishA);
  CHECK(m.state() == MotionState::MOVING);

  // Tick through the whole nominal duration; the IK failure at the midpoint
  // must end the move early, holding the last good theta/xyz, with e=2.
  for (uint32_t t = tFinishA; t <= tFinishA + 20000; t += (uint32_t)TICK_MS) {
    m.tick(t);
    if (m.state() == MotionState::IDLE) break;
  }
  CHECK(m.state() == MotionState::IDLE);
  CHECK(m.error() == 2);
  // Held short of B: never actually reached the requested target.
  CHECK(fabsf(m.xyz()[0] - b[0]) > 1.0f || fabsf(m.xyz()[1] - b[1]) > 1.0f);
  (void)xyzAtA;
  (void)thetaAtA;
}

void test_pending_slot_newest_wins() {
  Motion m;
  m.enable(0);
  m.setApproachEnabled(false);  // base planner; approach tested separately
  float t1[3] = {0, 0, -650};
  float t2[3] = {30, 0, -650};
  float t3[3] = {-30, 0, -650};

  m.acceptManualMove(makeMove(0, 0, 2, 2, t1[0], t1[1], t1[2]), 0);
  CHECK(m.state() == MotionState::MOVING);

  m.acceptManualMove(makeMove(0, 0, 2, 2, t2[0], t2[1], t2[2]), 10);  // queued
  m.acceptManualMove(makeMove(0, 0, 2, 2, t3[0], t3[1], t3[2]), 20);  // newest wins, replaces t2

  for (uint32_t t = 0; t < 20000; t += (uint32_t)TICK_MS) {
    m.tick(t);
    if (m.state() == MotionState::IDLE) break;
  }
  // First move (t1) must have completed before the pending one started.
  // The pending move that actually ran must be t3, not t2.
  float theta3[3];
  CHECK(ik(t3, theta3));
  CHECK_NEAR(m.xyz()[0], t3[0], 0.01);
  CHECK_NEAR(m.theta()[0], theta3[0], 0.01);
}

void test_program_runs_once_with_dwell() {
  Motion m;
  m.enable(0);
  m.setApproachEnabled(false);  // base planner; approach tested separately
  float p0[3] = {0, 0, -650};
  float p1[3] = {30, 0, -650};

  m.acceptProgramPoint(makeMove(0, 0, 5, 5, p0[0], p0[1], p0[2]));
  m.acceptProgramPoint(makeMove(1, 0, 5, 5, p1[0], p1[1], p1[2]));
  m.setDwell(0, 500);  // 500ms dwell after point 0

  m.setStart(true, 0);
  CHECK(m.isRunning());
  CHECK(m.state() == MotionState::MOVING);

  uint32_t t = 0;
  bool sawDwell = false;
  for (; t < 30000; t += (uint32_t)TICK_MS) {
    m.tick(t);
    if (m.state() == MotionState::DWELL) sawDwell = true;
    if (!m.isRunning() && m.state() == MotionState::IDLE && t > 100) break;
  }
  CHECK(sawDwell);
  CHECK(!m.isRunning());  // LOOP_PROGRAM=false: stops after the last point
  CHECK_NEAR(m.xyz()[0], p1[0], 0.01);

  // Ticking further must not restart the program.
  uint32_t finishedAt = t;
  m.tick(finishedAt + 1000);
  CHECK(!m.isRunning());
  CHECK(m.state() == MotionState::IDLE);
}

void test_stop_finishes_current_segment() {
  Motion m;
  m.enable(0);
  m.setApproachEnabled(false);  // base planner; approach tested separately
  float p0[3] = {0, 0, -650};
  float p1[3] = {40, 0, -650};

  m.acceptProgramPoint(makeMove(0, 0, 2, 2, p0[0], p0[1], p0[2]));
  m.acceptProgramPoint(makeMove(1, 0, 2, 2, p1[0], p1[1], p1[2]));
  m.setStart(true, 0);
  CHECK(m.state() == MotionState::MOVING);

  m.tick(5);  // partway into the first segment, not finished
  CHECK(m.state() == MotionState::MOVING);
  float thetaMidFlight[3] = {m.theta()[0], m.theta()[1], m.theta()[2]};

  m.setStart(false, 5);  // stop requested mid-segment

  // The in-flight move must still complete smoothly to point 0, not jump.
  bool everWentBackwardsOrJumped = false;
  float prevTheta0 = thetaMidFlight[0];
  uint32_t t = 5;
  for (; t < 20000; t += (uint32_t)TICK_MS) {
    m.tick(t);
    float d = fabsf(m.theta()[0] - prevTheta0);
    if (d > 5.0f) everWentBackwardsOrJumped = true;  // no single-tick teleport
    prevTheta0 = m.theta()[0];
    if (m.state() == MotionState::IDLE) break;
  }
  CHECK(!everWentBackwardsOrJumped);
  CHECK(m.state() == MotionState::IDLE);
  CHECK(!m.isRunning());
  // Stopped after finishing point 0, never advanced to point 1.
  CHECK_NEAR(m.xyz()[0], p0[0], 0.01);
}

void test_move_while_disabled_sets_e3() {
  Motion m;  // never enabled: starts DISABLED
  m.acceptManualMove(makeMove(0, 0, 5, 5, 0, 0, -650), 0);
  CHECK(m.error() == 3);
  CHECK(m.state() == MotionState::DISABLED);
}

void test_manual_move_while_program_running_sets_e3() {
  Motion m;
  m.enable(0);
  m.acceptProgramPoint(makeMove(0, 0, 5, 5, 0, 0, -650));
  m.acceptProgramPoint(makeMove(1, 0, 5, 5, 30, 0, -650));
  m.setStart(true, 0);
  CHECK(m.isRunning());
  m.acceptManualMove(makeMove(0, 0, 5, 5, -30, 0, -650), 1);
  CHECK(m.error() == 3);
}

void test_us_clamp_sets_e5() {
  Motion m;
  bool clamped = false;
  ServoCal wideOpenCal = CAL[0];
  wideOpenCal.centre_us = 1520;
  wideOpenCal.us_per_deg = 100;  // deliberately mismatched vs config.h's real value
  wideOpenCal.dir = 1;
  int us = Motion::angleToUs(80.0f, wideOpenCal, clamped);
  CHECK(clamped);
  CHECK(us == US_MAX);

  clamped = false;
  us = Motion::angleToUs(-30.0f, wideOpenCal, clamped);
  CHECK(clamped);
  CHECK(us == US_MIN);

  clamped = false;
  us = Motion::angleToUs(0.0f, CAL[0], clamped);
  CHECK(!clamped);  // config.h's actual placeholder calibration stays in range at 0 deg
}


// Ticks until IDLE (or timeout), tracking the lowest z and whether the move
// ever went IDLE in between (it mustn't: dip + rise are one move).
struct RunResult {
  float minZ = 1e9f;
  uint32_t tEnd = 0;
  bool finished = false;
  bool thetaRoseAfterMin = false;  // any bicep moved DOWN after the lowest point
};

RunResult runToIdle(Motion &m, uint32_t t0) {
  RunResult r;
  bool pastMin = false;
  float prev[3] = {m.theta()[0], m.theta()[1], m.theta()[2]};
  for (uint32_t t = t0; t <= t0 + 60000; t += (uint32_t)TICK_MS) {
    m.tick(t);
    if (m.xyz()[2] < r.minZ - 1e-4f) {
      r.minZ = m.xyz()[2];
    } else if (m.xyz()[2] > r.minZ + 0.5f) {
      pastMin = true;
    }
    for (int i = 0; i < 3; i++) {
      if (pastMin && m.theta()[i] > prev[i] + 1e-3f) r.thetaRoseAfterMin = true;
      prev[i] = m.theta()[i];
    }
    if (m.state() == MotionState::IDLE) {
      r.finished = true;
      r.tEnd = t;
      break;
    }
  }
  return r;
}

void test_downward_move_dips_then_rises() {
  Motion m;
  m.enable(0);
  float target[3] = {0, 0, -650};  // below home (zHome(0) ~ -597): every bicep moves down
  float theta1[3];
  CHECK(ik(target, theta1));

  m.acceptManualMove(makeMove(0, 0, 5, 5, target[0], target[1], target[2]), 0);
  CHECK(m.state() == MotionState::MOVING);
  CHECK(m.isApproaching());

  RunResult r = runToIdle(m, 0);
  CHECK(r.finished);
  CHECK(!m.isApproaching());
  CHECK_NEAR(r.minZ, target[2] - APPROACH_DZ_MM, 0.01);
  CHECK(!r.thetaRoseAfterMin);  // the final leg only ever lifts the biceps
  CHECK(m.error() == 0);
  for (int i = 0; i < 3; i++) {
    CHECK_NEAR(m.xyz()[i], target[i], 1e-3);
    CHECK_NEAR(m.theta()[i], theta1[i], 1e-3);
  }
}

void test_sideways_move_with_a_bicep_going_down_dips() {
  Motion m;
  m.enable(0);
  m.setApproachEnabled(false);
  float start[3] = {0, 0, -650};
  m.acceptManualMove(makeMove(0, 0, 5, 5, start[0], start[1], start[2]), 0);
  RunResult r0 = runToIdle(m, 0);
  CHECK(r0.finished);

  // Same z, moved sideways: some biceps go down, some up.
  m.setApproachEnabled(true);
  float target[3] = {40, 0, -650};
  m.acceptManualMove(makeMove(0, 0, 5, 5, target[0], target[1], target[2]), r0.tEnd);
  CHECK(m.isApproaching());
  RunResult r = runToIdle(m, r0.tEnd);
  CHECK(r.finished);
  CHECK_NEAR(r.minZ, target[2] - APPROACH_DZ_MM, 0.01);
  CHECK_NEAR(m.xyz()[0], target[0], 1e-3);
  CHECK_NEAR(m.xyz()[2], target[2], 1e-3);
}

void test_upward_move_goes_direct() {
  Motion m;
  m.enable(0);
  m.setApproachEnabled(false);
  float low[3] = {0, 0, -700};
  m.acceptManualMove(makeMove(0, 0, 5, 5, low[0], low[1], low[2]), 0);
  RunResult r0 = runToIdle(m, 0);
  CHECK(r0.finished);

  m.setApproachEnabled(true);
  float xyz0[3] = {m.xyz()[0], m.xyz()[1], m.xyz()[2]};
  float theta0[3] = {m.theta()[0], m.theta()[1], m.theta()[2]};
  float target[3] = {0, 0, -650};
  float theta1[3];
  CHECK(ik(target, theta1));
  m.acceptManualMove(makeMove(0, 0, 5, 5, target[0], target[1], target[2]), r0.tEnd);
  CHECK(!m.isApproaching());
  RunResult r = runToIdle(m, r0.tEnd);
  CHECK(r.finished);
  CHECK(r.minZ >= low[2] - 1e-3f);  // never went below where it started
  CHECK(r.tEnd - r0.tEnd <= expectedDurationMs(xyz0, target, theta0, theta1, 5) + TICK_MS);
  CHECK_NEAR(m.xyz()[2], target[2], 1e-3);
}

void test_dip_is_clamped_to_floor() {
  Motion m;
  m.enable(0);
  float target[3] = {0, 0, APPROACH_Z_FLOOR_MM + 5.0f};
  float theta1[3];
  CHECK(ik(target, theta1));
  m.acceptManualMove(makeMove(0, 0, 5, 5, target[0], target[1], target[2]), 0);
  CHECK(m.isApproaching());
  RunResult r = runToIdle(m, 0);
  CHECK(r.finished);
  CHECK_NEAR(r.minZ, APPROACH_Z_FLOOR_MM, 0.01);  // shortened dip, never below the floor
  CHECK_NEAR(m.xyz()[2], target[2], 1e-3);
}

void test_program_dwell_starts_after_the_rise() {
  Motion m;
  m.enable(0);
  float p0[3] = {0, 0, -650};
  m.acceptProgramPoint(makeMove(0, 0, 5, 5, p0[0], p0[1], p0[2]));
  m.setDwell(0, 500);
  m.setStart(true, 0);
  CHECK(m.isApproaching());

  bool sawDwell = false;
  for (uint32_t t = 0; t < 60000; t += (uint32_t)TICK_MS) {
    m.tick(t);
    if (m.state() == MotionState::DWELL && !sawDwell) {
      sawDwell = true;
      CHECK_NEAR(m.xyz()[2], p0[2], 1e-3);  // dwelling at the point, not at the dip
    }
    if (!m.isRunning() && m.state() == MotionState::IDLE && t > 100) break;
  }
  CHECK(sawDwell);
  CHECK_NEAR(m.xyz()[2], p0[2], 1e-3);
}

void test_disable_mid_dip_cancels_rise() {
  Motion m;
  m.enable(0);
  m.acceptManualMove(makeMove(0, 0, 5, 5, 0, 0, -650), 0);
  CHECK(m.isApproaching());
  m.tick(100);
  m.disable();
  CHECK(!m.isApproaching());
  m.enable(200);
  m.tick(400);
  CHECK(m.state() == MotionState::IDLE);  // no leftover rise starts by itself
}
}  // namespace

int main() {
  test_smoothstep_and_joint_mode_sync();
  test_t_formula_v_default();
  test_t_min_floor();
  test_linear_mode_intermediate_on_segment();
  test_linear_mode_unreachable_sets_e2_and_holds();
  test_pending_slot_newest_wins();
  test_program_runs_once_with_dwell();
  test_stop_finishes_current_segment();
  test_move_while_disabled_sets_e3();
  test_manual_move_while_program_running_sets_e3();
  test_us_clamp_sets_e5();
  test_downward_move_dips_then_rises();
  test_sideways_move_with_a_bicep_going_down_dips();
  test_upward_move_goes_direct();
  test_dip_is_clamped_to_floor();
  test_program_dwell_starts_after_the_rise();
  test_disable_mid_dip_cancels_rise();

  if (g_failures == 0) {
    printf("ALL PASS\n");
    return 0;
  }
  fprintf(stderr, "%d failure(s)\n", g_failures);
  return 1;
}
