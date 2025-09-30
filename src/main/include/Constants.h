// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once
#include <units/angle.h>
#include <units/length.h>
#include <units/angular_velocity.h>
#include <units/dimensionless.h>
#include "IO/CustomXbox.h"

enum Alliances {
  RED,
  BLUE
};

inline bool USE_COMP_VALUE;
inline Alliances currAlliance;

// Hardware IDs
#define PIGEON_ID 50

// Drivebase IDs
#define FR_DRIVE_MOTOR_ID 1
#define FR_STEER_MOTOR_ID 2

#define FL_DRIVE_MOTOR_ID 3
#define FL_STEER_MOTOR_ID 4

#define BR_DRIVE_MOTOR_ID 5
#define BR_STEER_MOTOR_ID 6

#define BL_DRIVE_MOTOR_ID 7
#define BL_STEER_MOTOR_ID 8

// Constants
static constexpr double RAD_TO_DEG = 180 / std::numbers::pi;
static constexpr double DEG_TO_RAD = std::numbers::pi / 180;

static constexpr int DRIVING_MOTOR_PINION_TEETH = 13;
static constexpr units::scalar_t DRIVE_GEAR_RATIO = (45.0 * 22.0) / (DRIVING_MOTOR_PINION_TEETH * 15.0);
static constexpr units::scalar_t STEER_GEAR_RATIO = 3.0 * 4.0 * (62.0 / 13.0);
static constexpr units::scalar_t WHEEL_DIAMETER = 3;

static constexpr units::inch_t X_OFFSET_FORWARD = 14.0_in;
static constexpr units::inch_t X_OFFSET_BACK = 14.0_in;
static constexpr units::inch_t Y_OFFSET = 14.0_in;

static constexpr units::turns_per_second_t MAX_SPEED_TPS = 50_tps;  // 110 tps

static constexpr double MAX_ROTATIONAL_SPEED = 2;
static constexpr double MAX_SPEED = 100; //198 inches per second

struct ModuleVelocity {
  units::turns_per_second_t magnitude;
  units::degree_t direction;
};

// Controls
#define D_FIELD_DRIVE CustomXbox::LS
#define D_ROBOT_DRIVE CustomXbox::RS