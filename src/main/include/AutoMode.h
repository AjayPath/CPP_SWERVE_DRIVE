// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <units/math.h>
#include <units/length.h>
#include "IO/PigeonGyro.h"
#include "Constants.h"
#include "SubsystemIncludes.h"

class AutoMode {
 public:
  
  int id;
  int step;
  std::string name;
  units::degree_t initialAngle;

  // Subsystems
  PigeonGyro* Gyro;

  // Timers
  frc::Timer* autoTimer;
  frc::Timer* totalAutoTimer;

  AutoMode() {
    autoTimer = new frc::Timer();
    totalAutoTimer = new frc::Timer();

    // Get instances of drivebase

    initialAngle = 0_deg; // Depends on auto config
    Gyro->SetAngle(initialAngle.value());
    step = 0;
  }

  ~AutoMode() {}

  virtual void Run() {};
  virtual void ReflectIfRed() {};

  std::string GetName() {
    return name;
  };

  void ResetAuto() {
    step = 0;
    Gyro->SetAngle(initialAngle.value());
    totalAutoTimer->Reset();
    totalAutoTimer->Stop();
  }

  void ResetTimer() {
    autoTimer->Reset();
    autoTimer->Start();
  }

  int GetStep() {
    return step;
  }

  // Vector offsets

  double speed = 50; // 180
  units::second_t timeout = 1.0_s;

};
