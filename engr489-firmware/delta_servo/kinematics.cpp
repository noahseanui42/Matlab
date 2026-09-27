// Derived from grzesiek2201/Delta-Robot (GPL-3.0) @ 48a8038
// https://github.com/grzesiek2201/Delta-Robot
//
// Port of inverse_kin.cpp, checked against DeltaApp/src/deltarobot.py
// (calculateConstants / calculateIPK), with the fixes noted in HANDOFF §6.2:
// explicit discriminant/singularity guards instead of relying on NaN
// comparisons, M_PI instead of 3.14, and no avr-libc square().
#include "kinematics.h"
#include "config.h"
#include <math.h>

namespace {

constexpr float SQRT_3 = 1.7320508075688772f;

// Derived geometry constants (HANDOFF §6.2), computed once at static init.
const float wb = SB / (2.0f * SQRT_3);
const float up = SP / SQRT_3;
const float wp = up / 2.0f;
const float a = wb - up;
const float b = SP / 2.0f - (SQRT_3 / 2.0f) * wb;
const float c = wp - wb / 2.0f;

constexpr float EPS_DENOM = 1e-6f;

inline float sq(float v) { return v * v; }

// Solves one arm's angle (degrees) from its E/F/G constants, trying the '+'
// root first and falling back to '-' if it's outside [minDeg, maxDeg].
// Returns false if neither root is real or reachable.
bool solveArm(float E, float F, float G, float minDeg, float maxDeg, float &thetaDegOut) {
  float disc = sq(E) + sq(F) - sq(G);
  if (disc < 0.0f) {
    return false;  // no real solution for this arm
  }
  float root = sqrtf(disc);
  float denom = G - E;
  if (fabsf(denom) < EPS_DENOM) {
    return false;  // avoid dividing by ~0
  }

  float thetaPlus = 2.0f * atanf((-F + root) / denom) * (180.0f / (float)M_PI);
  if (thetaPlus >= minDeg && thetaPlus <= maxDeg) {
    thetaDegOut = thetaPlus;
    return true;
  }

  float thetaMinus = 2.0f * atanf((-F - root) / denom) * (180.0f / (float)M_PI);
  if (thetaMinus >= minDeg && thetaMinus <= maxDeg) {
    thetaDegOut = thetaMinus;
    return true;
  }

  return false;
}

}  // namespace

bool ik(const float xyz[3], float thetaDeg[3]) {
  const float x = xyz[0];
  const float y = xyz[1];
  const float z = xyz[2];

  const float E[3] = {
    2.0f * L_UP * (y + a),
    -L_UP * (SQRT_3 * (x + b) + y + c),
    L_UP * (SQRT_3 * (x - b) - y - c),
  };
  const float F[3] = {2.0f * z * L_UP, 2.0f * z * L_UP, 2.0f * z * L_UP};
  const float l2 = L_LO * L_LO;
  const float L2 = L_UP * L_UP;
  const float G[3] = {
    sq(x) + sq(y) + sq(z) + sq(a) + L2 + 2.0f * y * a - l2,
    sq(x) + sq(y) + sq(z) + sq(b) + sq(c) + L2 + 2.0f * x * b + 2.0f * y * c - l2,
    sq(x) + sq(y) + sq(z) + sq(b) + sq(c) + L2 - 2.0f * x * b + 2.0f * y * c - l2,
  };

  float result[3];
  for (int i = 0; i < 3; i++) {
    if (!solveArm(E[i], F[i], G[i], CAL[i].min_deg, CAL[i].max_deg, result[i])) {
      return false;  // no partial writes to thetaDeg on failure
    }
  }

  thetaDeg[0] = result[0];
  thetaDeg[1] = result[1];
  thetaDeg[2] = result[2];
  return true;
}

float zHome(float thetaDeg) {
  float theta = thetaDeg * ((float)M_PI / 180.0f);
  float radial = wb + L_UP * cosf(theta) - up;
  return -L_UP * sinf(theta) - sqrtf(L_LO * L_LO - radial * radial);
}
