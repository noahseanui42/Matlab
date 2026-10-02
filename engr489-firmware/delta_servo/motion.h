// Planner + program runner state machine (HANDOFF §6.4). No Arduino
// headers: must compile with plain g++ so it is host-testable
// (tests/motion_test_runner.cpp). Time is injected via nowMs everywhere so
// tests can use a fake clock; the glue layer (delta_servo.ino) passes
// millis().
#pragma once

#include <stdint.h>
#include "protocol.h"  // MovePoint, FuncType
#include "config.h"    // MAX_POINTS

enum class MotionState { DISABLED, IDLE, MOVING, DWELL };

class Motion {
 public:
  Motion();

  // --- Commands, wired up from ProtocolCallbacks by the glue layer ---

  // Attaching/writing the actual Servo objects is the glue layer's job (it's
  // the only place allowed to touch Arduino headers); this just updates the
  // planner's own state. theta_/xyz_ are NOT reset here -- see the note on
  // enable() in motion.cpp for why re-enabling snaps back to whatever was
  // last commanded, not to flat.
  void enable(uint32_t nowMs);
  void disable();

  // mode '2': manual/jog move. Rejects (e=3) if disabled or a program is
  // running; rejects (e=1) if xyz is unreachable; otherwise starts the move
  // now, or queues it in the single pending slot (newest wins) if a move is
  // already in flight.
  void acceptManualMove(const MovePoint &pt, uint32_t nowMs);

  // mode '1': store/overwrite program point pt.n; program_len becomes n+1.
  // Does not start anything.
  void acceptProgramPoint(const MovePoint &pt);

  // mode '3' (FuncType::WAIT_TIME only; v1 ignores WAIT_INPUT/SET_OUTPUT
  // per HANDOFF §5): store point ptNo's post-move dwell, in ms.
  void setDwell(int ptNo, float ms);

  // mode '0'.
  void setStart(bool start, uint32_t nowMs);

  // Advance the state machine by one tick (call every TICK_MS).
  void tick(uint32_t nowMs);

  // Servo pulse widths (us) for the current theta, clamped to
  // [US_MIN, US_MAX]; sets error()==5 if any pulse was clamped.
  void computeServoUs(int usOut[3]);

  // Pure conversion for one joint: angle (deg) -> clamped pulse width (us)
  // using that arm's calibration. Exposed as a public static, rather than
  // folded only into computeServoUs(), so the clamp math can be tested
  // directly -- with the placeholder calibration in config.h, normal motion
  // never drives an angle far enough to hit US_MIN/US_MAX (CAL[i].min_deg/
  // max_deg map well inside them), so the clamp is only exercisable this
  // way until real calibration data can disagree with the angle limits.
  static int angleToUs(float thetaDeg, const ServoCal &cal, bool &clamped);

  // Upward final approach (config.h APPROACH_*): a move that would finish
  // with any bicep moving down first goes APPROACH_DZ_MM below the target,
  // then rises straight up into it, so every servo's backlash is taken up
  // the same way. Both legs run as one move (state stays MOVING; a dwell,
  // a pending move or a requested stop all wait for the rise). On by
  // default when APPROACH_DZ_MM > 0; tests of the base planner turn it off.
  void setApproachEnabled(bool on) { approachEnabled_ = on; }
  bool isApproaching() const { return approachPending_; }

  // --- Status, for protocol::buildStatus() / the glue layer ---
  const float *theta() const { return theta_; }
  const float *xyz() const { return xyz_; }
  MotionState state() const { return state_; }
  bool isMoving() const { return state_ == MotionState::MOVING || state_ == MotionState::DWELL; }
  bool isRunning() const { return running_; }
  bool isEnabled() const { return state_ != MotionState::DISABLED; }
  int error() const { return error_; }

  // Lets the glue layer report a protocol-level error (currently just e=4,
  // a JSON parse failure) into the same latch Motion itself uses for
  // e=1/2/3/5. HANDOFF §5 describes "e" as one shared, latched-until-next-
  // accepted-move field, and only Motion knows when a move is accepted.
  void setError(int code) { error_ = code; }

 private:
  struct ProgramPoint {
    bool valid = false;
    int mode_i = 0;
    int v = 0;
    float xyz[3] = {0, 0, 0};
    bool hasDwell = false;
    float dwellMs = 0;
  };

  struct MoveTarget {
    bool linear = false;
    float xyz1[3] = {0, 0, 0};
    float theta1[3] = {0, 0, 0};
    int v = 0;
    int programIndex = -1;  // -1 for a manual (mode '2') move
  };

  struct ActiveMove {
    bool linear = false;
    int programIndex = -1;
    float theta0[3] = {0, 0, 0};
    float theta1[3] = {0, 0, 0};
    float xyz0[3] = {0, 0, 0};
    float xyz1[3] = {0, 0, 0};
    uint32_t tStartMs = 0;
    uint32_t durationMs = 0;
  };

  MotionState state_;
  bool running_ = false;
  bool stopRequested_ = false;
  int error_ = 0;

  float theta_[3];
  float xyz_[3];

  ActiveMove active_;
  bool pendingValid_ = false;
  MoveTarget pending_;

  ProgramPoint program_[MAX_POINTS];
  int programLen_ = 0;

  uint32_t dwellStartMs_ = 0;
  uint32_t dwellDurationMs_ = 0;
  int dwellProgramIndex_ = -1;

  bool approachEnabled_ = (APPROACH_DZ_MM > 0.0f);
  bool approachPending_ = false;  // in the dip leg; approachFinal_ runs next
  MoveTarget approachFinal_;

  bool solveAndBuildTarget(const float xyz[3], int mode_i, int v, int programIndex, MoveTarget &out);
  bool planApproach(const MoveTarget &t, MoveTarget &dip) const;
  void startMove(const MoveTarget &t, uint32_t nowMs);
  void beginMove(const MoveTarget &t, uint32_t nowMs);
  void beginProgramPoint(int idx, uint32_t nowMs);
  void finishMove(uint32_t nowMs);
  void endDwell(uint32_t nowMs);
  void advanceProgram(int justFinishedIdx, uint32_t nowMs);
};
