// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once
#include "Calculations.h"
#include <frc/Timer.h>
#include <units/time.h>
#include <wpimath/MathShared.h>

class SlewRateLimiter {
 public:
 
  explicit SlewRateLimiter(double _rateLimit, double);

  double Calculate(double _input);

  void Reset(double _input);

 private:
  double maxRateLimit = 0.0;
  double currRateLimit = 0.0;
  double prevRateLimit = 0.0;
  double jerkLimit = 0.0;
  double prevVal = 0.0;
  units::second_t prevTime = 0.0_s;
  units::second_t elapsedTime = 0.0_s;
  units::second_t currTime = 0.0_s;

};
