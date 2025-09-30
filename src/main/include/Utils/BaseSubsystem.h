// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once
#include "IO/Controllers.h"
#include "Utils/SimPID.h"
#include <rev/SparkMax.h>
#include <units/voltage.h>
#include <units/dimensionless.h>


class BaseSubsystem {
 public:
  
  virtual void RobotInit() {};
  virtual void RobotPeriodic() {};
  virtual void DisabledInit() {};
  virtual void DisabledPeriodic() {};
  virtual void AutonomousInit() {};
  virtual void AutonomousPeriodic() {};
  virtual void TeleopInit() {};
  virtual void TeleopPeriodic() {};

  Controllers* GamePads = Controllers::GetInstance();

};
