// Derived from grzesiek2201/Delta-Robot (GPL-3.0) @ 48a8038
// https://github.com/grzesiek2201/Delta-Robot
//
// Delta inverse kinematics. No Arduino headers: must compile with plain g++
// so it is host-testable (tests/test_kinematics.py, tests/ik_cli).
#pragma once

// Solves inverse kinematics for effector position xyz (mm, base frame) and
// writes joint angles (degrees) to thetaDeg on success. Returns false, and
// leaves thetaDeg untouched, if any arm can't reach xyz within its
// configured [min_deg, max_deg] limit (config.h CAL[i]).
bool ik(const float xyz[3], float thetaDeg[3]);

// Effector z (mm) with all three joints at thetaDeg (degrees), i.e. the
// z of (0, 0, z) that puts every bicep at that same angle. Used to seed the
// starting XYZ from theta=0 on enable; v1 has no general FK.
float zHome(float thetaDeg);
