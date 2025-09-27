// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include "AutoMode.h"
#include "CustomIncludes.h"
#include "SubsystemIncludes.h"
#include "Utils/CustomLoggable.h"

class BaseAuto : public CustomLoggable {
 public:
  
  static BaseAuto* GetInstance();

  // public functions
  void DisabledInit();
  void AutonomousInit();
  void AutonomousPeriodic();
  void ShowOnDashboard();

  // control functions
  void SensorReset();
  void StartTimers();
  void AutoSelector();

  // Getters / Setters
  int GetAutoMode();
  double GetAutoDelay();

  private:
    BaseAuto();
    ~BaseAuto();

    static BaseAuto* instance;

    // Class Variables
    int SelectedAutoMode;
    double AutoDelay;

    // Control FUnction
    void ResetStates();
    std::string GetAutoModeString();

    // subsystems
    PigeonGyro* Gyro;
    Controllers* GamePads;

    std::vector<AutoMode*> AutoModes;

    // Timers
    frc::Timer* DelayTimer;
    frc::Timer* AutoTimer;
    frc::Timer* AutoAwaitTimer;

};
