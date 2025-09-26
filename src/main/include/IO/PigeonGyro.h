// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/Timer.h>
#include <ctre/phoenix6/Pigeon2.hpp>
#include "./Constants.h"

class PigeonGyro {
 public:
  
  /**
   * @brief Gets the singleton instance of PigeonGyro
   * @return Pointer to the singleton PigeonGyro instance
   */
  static PigeonGyro* GetInstance();
  
  /**
   * @brief Constructor for PigeonGyro
   */
  PigeonGyro(void);

  /**
   * @brief Get the current yaw angle of the robot
   * @return Current angle in degrees (units::degree_t)
   */
  units::degree_t GetAngle();

  /**
   * @brief Get the current pitch angle of the robot
   * @return Current pitch in degrees (units::degree_t)
   */
  units::degree_t GetPitch();

  /**
   * @brief Get the current roll angle of the robot
   * @return Current roll in degrees (units::degree_t)
   */
  units::degree_t GetRoll();

  /**
   * @brief Get the rate of change of the yaw angle
   * @return Angular velocity in degrees per second
   */
  double GetAngleRate();

  /**
   * @brief Get the acceleration in the X-axis (forward/backward)
   * @return X-axis acceleration in m/s²
   */
  double GetXAccel();

  /**
   * @brief Get the acceleration in the Y-axis (left/right)
   * @return Y-axis acceleration in m/s²
   */
  double GetYAccel();

  /**
   * @brief Get the acceleration in the Z-axis (up/down)
   * @return Z-axis acceleration in m/s²
   */
  double GetZAccel();

  /**
   * @brief Set the yaw angle of the Pigeon 2 gyroscope
   * @param pos The angle to set in degrees
   */
  void SetAngle(double pos);

  // Signal readings for efficient CAN communication
  ctre::phoenix6::StatusSignal<units::degree_t>* GyroAngleSignal;
  ctre::phoenix6::StatusSignal<units::degree_t>* GyroPitchSignal;

  private:
    ctre::phoenix6::hardware::Pigeon2* Gyro;
    frc::Timer* GyroAngleTimer;
    frc::Timer* GyroPitchTimer;
    frc::Timer* GyroRollTimer;

    double lastAngle;
    double lastRoll;
    double lastPitch;
    
    static PigeonGyro* instance;
};