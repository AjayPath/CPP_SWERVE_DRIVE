// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once
#include "IO/PigeonGyro.h"
#include "CustomIncludes.h"
#include "RevSwerveModule.h"
#include "Utils/BaseSubsystem.h"

class DriveSubsystem : public BaseSubsystem, public CustomLoggable {
 public:
  
  static DriveSubsystem* GetInstance();

  void RobotInit();
  void RobotPeriodic();
  void DisabledInit();
  void DisabledPeriodic();
  void TeleopInit();
  void TeleopPeriodic();
  void AutonomousInit();
  void AutonomousPeriodic();
  void ShowOnDashboard();

  typedef enum DriveStates {
    IDLE,
    FIELD_DRIVE,
    ROBOT_DRIVE
  } DriveStates;

  void ResetDrive();
  void SetState(DriveStates _state);
  DriveStates GetState();
  CustomVector GetRobotVelocity();

 private:
  DriveSubsystem(void);
  ~DriveSubsystem(void);
  static DriveSubsystem* instance;

  void StateMachine();

  void RobotCentricDrive(double xVel, double yVel, double rVel);
  void FieldCentricDrive(double xVel, double yVel, double rVel, double robotYComp = 0.0);

  // Getters & Setters
  std::string GetStateStr();

  std::string SwervePrefix = "FR";

  RevSwerveModule* swerveModule;

  ModuleVelocity swerveModuleVelocity;

  units::turns_per_second_t xMag = 0_tps;
  units::turns_per_second_t yMag = 0_tps;
  units::turns_per_second_t rMag = 0_tps;

  SlewRateLimiter* xLimiter;
  SlewRateLimiter* yLimiter;
  SlewRateLimiter* rLimiter;

  units::degree_t heading;
  units::degree_t desiredHeading;

  DriveStates currState = IDLE;
  DriveStates lastState = IDLE;

  PigeonGyro* Gyro;

  SimPID* TurnPID;
  SimPID* DrivePID;

  frc::Timer* brakeTimer;
  frc::Timer* holdHeadingTimer;

  units::degree_t lastSwerveHeading;

  double oldHeading = 0.0;

  bool resetOffsetValues = false;
  bool brakesEnabled = false;

  static constexpr double TurnP = 0.009;
  static constexpr double TurnI = 0.0;
  static constexpr double TurnD = 0.009;

  static constexpr double DriveP = 0.02;
  static constexpr double DriveI = 0.0;
  static constexpr double DriveD = 0.04;

  static constexpr double FR_STEER_ENCODER_OFFSET_PRACTICE = 0.0;

};
