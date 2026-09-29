// Derived from grzesiek2201/Delta-Robot (GPL-3.0) @ 48a8038
// https://github.com/grzesiek2201/Delta-Robot
//
// Single source of firmware config. No Arduino headers here: this file must
// compile with plain g++ so kinematics/protocol/motion stay host-testable.
// tests/test_config_sync.py regex-parses this file and asserts it matches
// delta_app/robot_config.py's SB/SP/L_UP/L_LO and CAL[i] min/max exactly.
#pragma once

#include <stdint.h>

// Geometry (mm). sb/sp = equilateral triangle SIDE lengths (see HANDOFF §4 Q1)
constexpr float SB = 175.0f;
constexpr float SP = 150.0f;
constexpr float L_UP = 180.0f;   // bicep
constexpr float L_LO = 625.0f;   // forearm (HANDOFF §4 Q2: confirmed 625 mm)

// Per-servo calibration. index = arm number - 1. arm 1 on -y axis; arms 1->2->3 CCW viewed from above.
// us = centre_us + dir * us_per_deg * theta_deg ; theta > 0 = bicep DOWN
// Pin mapping (HANDOFF §4 Q3, default confirmed): D9=arm1, D10=arm2, D11=arm3.
//
// centre_us/us_per_deg/dir: per-arm, measured with each arm driven
// individually (rest of the linkage resting flat).
//
// min_deg/max_deg: reconfirmed with the FULL 3-arm assembly (bring-up §10
// steps 4-5, all three enabled and jogged together) -- the single-arm
// numbers above under-measured the true range, because driving one arm
// while the other two are slack lets the effector plate sag/tilt out of
// level, creating an early false collision that doesn't happen when all
// three hold the plate level together. -20/70 deg is described as a safe
// margin, not the absolute mechanical stop -- there may be more room.
// centre_us = 1520 + trim_us from the calibration tool's output.
struct ServoCal { uint8_t pin; float centre_us; float us_per_deg; int8_t dir; float min_deg; float max_deg; };
constexpr ServoCal CAL[3] = {
  { 9, 1460.0f, 11.8231f, +1, -20.0f, 70.0f},  // arm1 (trim_us=-60)
  {10, 1385.0f, 10.0481f, +1, -20.0f, 70.0f},  // arm2 (trim_us=-135)
  {11, 1410.0f, 10.5544f, +1, -20.0f, 70.0f},  // arm3 (trim_us=-110)
};
constexpr int   US_MIN = 830, US_MAX = 2170;

// Which physical arm (CAL[]/thetaDeg[] index, i.e. Arduino pin above) sits
// at each geometric position the IK model in kinematics.cpp assumes: index 0
// is the model's "-y axis" arm, 1 is +120 deg CCW from it, 2 is +240 deg CCW.
// Confirmed during the pen-holder bench test (README's "Pen-holder end
// effector" section): physical Arm 3 (D11) is the one actually sitting at
// the model's assumed -y position, not physical Arm 1 (D9) -- the base
// plate's extra mounting-hole options meant the arms got bolted on walked
// 120 deg around from the assumed layout. Calibration (CAL[] above) is
// unaffected: it's measured per physical servo and stays indexed by pin
// regardless of which geometric role that servo plays.
// Keep in sync with delta_app/robot_config.py's GEOM_TO_PHYS
// (tests/test_config_sync.py enforces this).
constexpr int GEOM_TO_PHYS[3] = {2, 0, 1};

// Motion
constexpr uint32_t TICK_MS = 20;          // servo frame & planner tick
constexpr uint32_t STREAM_MS = 20;
constexpr uint32_t RX_IDLE_TIMEOUT_MS = 1000;
constexpr float SPEED_PER_V = 5.0f;       // mm/s per GUI v step -> 5..50 mm/s
constexpr int   V_DEFAULT = 2;
constexpr float MAX_JOINT_DPS = 90.0f;    // cap on average joint speed
constexpr uint32_t T_MIN_MS = 200;
constexpr int   MAX_POINTS = 50;          // matches GUI MAX_PROGRAM_LENGTH
constexpr bool  LOOP_PROGRAM = false;     // upstream loops forever; scans run once
