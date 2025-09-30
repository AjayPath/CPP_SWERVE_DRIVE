// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once
#include "Utils/BaseSubsystem.h"
#include "Utils/CustomLoggable.h"

class SubsystemController : public BaseSubsystem, public CustomLoggable {
 public:
  
  static SubsystemController* GetInstance();

  void RobotInit();
  void RobotPeriodic();
  void DisabledInit();
  void DisabledPeriodic();
  void TeleopInit();
  void TeleopPeriodic();
  void AutonomousInit();
  void AutonomousPeriodic();
  void ShowOnDashboard();

  private:
    SubsystemController(void);
    ~SubsystemController(void);

    static SubsystemController* instance;

};
