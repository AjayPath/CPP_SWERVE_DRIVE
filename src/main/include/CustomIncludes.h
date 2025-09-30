// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

// Unit Includes
#include <units/angle.h>
#include <units/angular_acceleration.h>
#include <units/angular_velocity.h>
#include <units/math.h>
#include <units/velocity.h>
#include <units/length.h>
#include <units/dimensionless.h>

// General Libs
#include <math.h>
#include <stdio.h>
#include <string.h>
#include <cmath>
#include <iostream>

// FRC/WPI Libs
#include <frc/DigitalInput.h>
#include <frc/DriverStation.h>
#include <frc/RobotController.h>
#include <frc/Counter.h>
#include <frc/estimator/KalmanFilter.h>
#include <frc/filter/SlewRateLimiter.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <networktables/NetworkTable.h>
#include <networktables/NetworkTableEntry.h>
#include <networktables/NetworkTableInstance.h>
#include <networktables/NetworkTableValue.h>
#include <wpi/SpanExtras.h>
#include <wpinet/PortForwarder.h>

// CTRE
#include <ctre/phoenix6/Pigeon2.hpp>
#include <rev/Sparkmax.h>
#include <rev/AbsoluteEncoder.h>

// Custom
#include "Calculations.h"
#include "Constants.h"
#include "Utils/CustomLoggable.h"
#include "Utils/CustomVector.h"
#include "Utils/CustomPose.h"