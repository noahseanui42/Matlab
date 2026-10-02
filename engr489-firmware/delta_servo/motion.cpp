// Planner + program runner (HANDOFF §6.4).
#include "motion.h"
#include "kinematics.h"
#include "config.h"
#include <math.h>

Motion::Motion() : state_(MotionState::DISABLED) {
  theta_[0] = theta_[1] = theta_[2] = 0.0f;
  xyz_[0] = 0.0f;
  xyz_[1] = 0.0f;
  xyz_[2] = zHome(0.0f);
}

void Motion::enable(uint32_t /*nowMs*/) {
  // theta_/xyz_ are deliberately left untouched: they already hold either
  // the boot default (0,0,zHome(0)) set above, or whatever was last
  // commanded before a previous disable(). HANDOFF §6.4 documents both: the
  // very first enable "snaps to flat" only because that's the boot default,
  // and "commanded theta/xyz are kept, so re-enabling snaps back to them"
  // on every later cycle. The glue layer attaches the Servo objects and
  // writes this theta out; that's the actual "snap".
  if (state_ == MotionState::DISABLED) {
    state_ = MotionState::IDLE;
  }
}

void Motion::disable() {
  state_ = MotionState::DISABLED;
  running_ = false;
  stopRequested_ = false;
  pendingValid_ = false;
  approachPending_ = false;
  active_.linear = false;
}

bool Motion::solveAndBuildTarget(const float xyz[3], int mode_i, int v, int programIndex, MoveTarget &out) {
  if (!ik(xyz, out.theta1)) {
    error_ = 1;
    return false;
  }
  out.linear = (mode_i == 1);
  out.xyz1[0] = xyz[0];
  out.xyz1[1] = xyz[1];
  out.xyz1[2] = xyz[2];
  out.v = v;
  out.programIndex = programIndex;
  return true;
}

// Decides, from the current commanded pose, whether t needs the upward final
// approach, and if so builds the dip leg. Falls back to a direct move (false)
// when no bicep would finish moving down, when the floor leaves no room to
// dip, when the dip point is unreachable, or when rising from it wouldn't
// lift every bicep (so the rise couldn't take up the backlash anyway).
bool Motion::planApproach(const MoveTarget &t, MoveTarget &dip) const {
  if (!approachEnabled_ || APPROACH_DZ_MM <= 0.0f) {
    return false;
  }
  bool anyDown = false;
  for (int i = 0; i < 3; i++) {
    if (t.theta1[i] - theta_[i] > APPROACH_TRIGGER_DEG) anyDown = true;
  }
  if (!anyDown) {
    return false;
  }
  float dipZ = t.xyz1[2] - APPROACH_DZ_MM;
  if (dipZ < APPROACH_Z_FLOOR_MM) dipZ = APPROACH_Z_FLOOR_MM;
  if (dipZ > t.xyz1[2] - 1.0f) {
    return false;  // target is at (or below) the floor: nowhere to dip
  }
  float p[3] = {t.xyz1[0], t.xyz1[1], dipZ};
  if (!ik(p, dip.theta1)) {
    return false;
  }
  for (int i = 0; i < 3; i++) {
    if (dip.theta1[i] <= t.theta1[i]) return false;
  }
  dip.linear = t.linear;
  dip.xyz1[0] = p[0];
  dip.xyz1[1] = p[1];
  dip.xyz1[2] = p[2];
  dip.v = t.v;
  dip.programIndex = t.programIndex;
  return true;
}

// Starts a newly accepted target: straight there, or via the dip leg with
// the vertical rise queued in approachFinal_ (run by finishMove()).
void Motion::startMove(const MoveTarget &t, uint32_t nowMs) {
  MoveTarget dip;
  if (planApproach(t, dip)) {
    approachFinal_ = t;
    approachFinal_.linear = true;  // rise straight up, not a joint-space arc
    approachPending_ = true;
    beginMove(dip, nowMs);
    return;
  }
  approachPending_ = false;
  beginMove(t, nowMs);
}

void Motion::beginMove(const MoveTarget &t, uint32_t nowMs) {
  active_.linear = t.linear;
  active_.programIndex = t.programIndex;
  for (int i = 0; i < 3; i++) {
    active_.theta0[i] = theta_[i];
    active_.theta1[i] = t.theta1[i];
    active_.xyz0[i] = xyz_[i];
    active_.xyz1[i] = t.xyz1[i];
  }
  active_.tStartMs = nowMs;

  float dx = active_.xyz1[0] - active_.xyz0[0];
  float dy = active_.xyz1[1] - active_.xyz0[1];
  float dz = active_.xyz1[2] - active_.xyz0[2];
  float dist = sqrtf(dx * dx + dy * dy + dz * dz);

  float maxDeltaTheta = 0.0f;
  for (int i = 0; i < 3; i++) {
    float d = fabsf(active_.theta1[i] - active_.theta0[i]);
    if (d > maxDeltaTheta) maxDeltaTheta = d;
  }

  int vEff = (t.v > 0) ? t.v : V_DEFAULT;
  float speed = SPEED_PER_V * (float)vEff;  // mm/s

  float tFromSpeedMs = (speed > 0.0f) ? (dist / speed * 1000.0f) : 0.0f;
  float tFromJointMs = (MAX_JOINT_DPS > 0.0f) ? (maxDeltaTheta / MAX_JOINT_DPS * 1000.0f) : 0.0f;

  float durationMs = tFromSpeedMs;
  if (tFromJointMs > durationMs) durationMs = tFromJointMs;
  if (durationMs < (float)T_MIN_MS) durationMs = (float)T_MIN_MS;

  active_.durationMs = (uint32_t)(durationMs + 0.5f);
  state_ = MotionState::MOVING;
}

void Motion::beginProgramPoint(int idx, uint32_t nowMs) {
  ProgramPoint &p = program_[idx];
  MoveTarget t;
  if (!solveAndBuildTarget(p.xyz, p.mode_i, p.v, idx, t)) {
    // A stored point turned out unreachable (e.g. bad geometry/limits) --
    // stop the program rather than run off into an undefined state.
    running_ = false;
    stopRequested_ = false;
    state_ = MotionState::IDLE;
    return;
  }
  error_ = 0;
  if (state_ == MotionState::MOVING) {
    pending_ = t;
    pendingValid_ = true;
    return;
  }
  startMove(t, nowMs);
}

void Motion::acceptManualMove(const MovePoint &pt, uint32_t nowMs) {
  if (state_ == MotionState::DISABLED) {
    error_ = 3;
    return;
  }
  if (running_) {
    error_ = 3;
    return;
  }
  MoveTarget t;
  if (!solveAndBuildTarget(pt.c, pt.i, pt.v, -1, t)) {
    return;  // error_ == 1
  }
  error_ = 0;
  if (state_ == MotionState::MOVING) {
    pending_ = t;
    pendingValid_ = true;
    return;
  }
  startMove(t, nowMs);
}

void Motion::acceptProgramPoint(const MovePoint &pt) {
  if (pt.n < 0 || pt.n >= MAX_POINTS) {
    return;
  }
  ProgramPoint &p = program_[pt.n];
  p.valid = true;
  p.mode_i = pt.i;
  p.v = pt.v;
  p.xyz[0] = pt.c[0];
  p.xyz[1] = pt.c[1];
  p.xyz[2] = pt.c[2];
  programLen_ = pt.n + 1;
}

void Motion::setDwell(int ptNo, float ms) {
  if (ptNo < 0 || ptNo >= MAX_POINTS) {
    return;
  }
  program_[ptNo].hasDwell = true;
  program_[ptNo].dwellMs = ms;
}

void Motion::setStart(bool start, uint32_t nowMs) {
  if (start) {
    if (running_ || state_ == MotionState::DISABLED || programLen_ <= 0) {
      return;
    }
    running_ = true;
    stopRequested_ = false;
    beginProgramPoint(0, nowMs);
  } else {
    if (!running_) {
      return;
    }
    stopRequested_ = true;
    pendingValid_ = false;
  }
}

void Motion::advanceProgram(int justFinishedIdx, uint32_t nowMs) {
  int next = justFinishedIdx + 1;
  if (next >= programLen_) {
    if (LOOP_PROGRAM) {
      next = 0;
    } else {
      running_ = false;
      state_ = MotionState::IDLE;
      return;
    }
  }
  beginProgramPoint(next, nowMs);
}

void Motion::finishMove(uint32_t nowMs) {
  int justFinishedIdx = active_.programIndex;

  // The dip leg of an upward approach just ended: the rise is part of the
  // same move, so it runs before any stop, dwell or pending move.
  if (approachPending_) {
    approachPending_ = false;
    beginMove(approachFinal_, nowMs);
    return;
  }

  if (stopRequested_) {
    running_ = false;
    stopRequested_ = false;
    pendingValid_ = false;
    state_ = MotionState::IDLE;
    return;
  }

  if (running_ && justFinishedIdx >= 0 && program_[justFinishedIdx].hasDwell) {
    state_ = MotionState::DWELL;
    dwellStartMs_ = nowMs;
    dwellDurationMs_ = (uint32_t)program_[justFinishedIdx].dwellMs;
    dwellProgramIndex_ = justFinishedIdx;
    return;
  }

  if (pendingValid_) {
    MoveTarget t = pending_;
    pendingValid_ = false;
    startMove(t, nowMs);
    return;
  }

  if (running_) {
    advanceProgram(justFinishedIdx, nowMs);
    return;
  }

  state_ = MotionState::IDLE;
}

void Motion::endDwell(uint32_t nowMs) {
  int idx = dwellProgramIndex_;

  if (stopRequested_) {
    running_ = false;
    stopRequested_ = false;
    pendingValid_ = false;
    state_ = MotionState::IDLE;
    return;
  }

  if (pendingValid_) {
    MoveTarget t = pending_;
    pendingValid_ = false;
    startMove(t, nowMs);
    return;
  }

  if (running_) {
    advanceProgram(idx, nowMs);
    return;
  }

  state_ = MotionState::IDLE;
}

void Motion::tick(uint32_t nowMs) {
  if (state_ == MotionState::DWELL) {
    if ((uint32_t)(nowMs - dwellStartMs_) >= dwellDurationMs_) {
      endDwell(nowMs);
    }
    return;
  }
  if (state_ != MotionState::MOVING) {
    return;
  }

  uint32_t elapsed = nowMs - active_.tStartMs;
  float u = (active_.durationMs == 0) ? 1.0f : (float)elapsed / (float)active_.durationMs;
  bool doneByTime = u >= 1.0f;
  if (u > 1.0f) u = 1.0f;
  float s = 3.0f * u * u - 2.0f * u * u * u;

  if (active_.linear) {
    float p[3] = {
      active_.xyz0[0] + s * (active_.xyz1[0] - active_.xyz0[0]),
      active_.xyz0[1] + s * (active_.xyz1[1] - active_.xyz0[1]),
      active_.xyz0[2] + s * (active_.xyz1[2] - active_.xyz0[2]),
    };
    float thetaCandidate[3];
    if (!ik(p, thetaCandidate)) {
      error_ = 2;
      approachPending_ = false;  // don't rise from wherever this stopped
      finishMove(nowMs);  // holds the last good theta_/xyz_ (untouched here)
      return;
    }
    theta_[0] = thetaCandidate[0];
    theta_[1] = thetaCandidate[1];
    theta_[2] = thetaCandidate[2];
    xyz_[0] = p[0];
    xyz_[1] = p[1];
    xyz_[2] = p[2];
  } else {
    theta_[0] = active_.theta0[0] + s * (active_.theta1[0] - active_.theta0[0]);
    theta_[1] = active_.theta0[1] + s * (active_.theta1[1] - active_.theta0[1]);
    theta_[2] = active_.theta0[2] + s * (active_.theta1[2] - active_.theta0[2]);
  }

  if (doneByTime) {
    xyz_[0] = active_.xyz1[0];
    xyz_[1] = active_.xyz1[1];
    xyz_[2] = active_.xyz1[2];
    theta_[0] = active_.theta1[0];
    theta_[1] = active_.theta1[1];
    theta_[2] = active_.theta1[2];
    finishMove(nowMs);
  }
}

int Motion::angleToUs(float thetaDeg, const ServoCal &cal, bool &clamped) {
  float usF = cal.centre_us + (float)cal.dir * cal.us_per_deg * thetaDeg;
  int us = (int)lroundf(usF);
  if (us < US_MIN) {
    us = US_MIN;
    clamped = true;
  }
  if (us > US_MAX) {
    us = US_MAX;
    clamped = true;
  }
  return us;
}

void Motion::computeServoUs(int usOut[3]) {
  bool clamped = false;
  for (int i = 0; i < 3; i++) {
    usOut[i] = angleToUs(theta_[i], CAL[i], clamped);
  }
  if (clamped) {
    error_ = 5;
  }
}
