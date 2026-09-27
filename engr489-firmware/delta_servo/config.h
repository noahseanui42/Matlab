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
struct ServoCal { uint8_t pin; float centre_us; float us_per_deg; int8_t dir; float min_deg; float max_deg; };
constexpr ServoCal CAL[3] = {
  { 9, 1520.0f, 8.0f, +1, -30.0f, 80.0f},  // TODO(calibrate)
  {10, 1520.0f, 8.0f, +1, -30.0f, 80.0f},  // TODO(calibrate)
  {11, 1520.0f, 8.0f, +1, -30.0f, 80.0f},  // TODO(calibrate)
};
constexpr int   US_MIN = 830, US_MAX = 2170;

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
