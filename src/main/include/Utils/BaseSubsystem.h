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

  void SetMotorPercent(rev::spark::SparkMax& motor, double speed) {
    motor.Set(speed);
  }
  
  void SetMotorVoltage(rev::spark::SparkMax& motor, double voltage) {
    motor.SetVoltage(units::voltage::volt_t(voltage));
  }

  void PositionControl(rev::spark::SparkMax& motor, SimPID& pid, double currentPosition, double maxVoltage = 12.0) {
    double pidOutput = pid.calcPID(currentPosition);
    double voltage = pidOutput * maxVoltage;
    motor.SetVoltage(units::voltage::volt_t(voltage));
  }

  void VelocityControl(rev::spark::SparkMax& motor, SimPID& pid, double currentVelocity, double maxVoltage) {
    double pidOutput = pid.calcPID(currentVelocity);
    double voltage = pidOutput * maxVoltage;
    motor.SetVoltage(units::voltage::volt_t(voltage));
  }

};
