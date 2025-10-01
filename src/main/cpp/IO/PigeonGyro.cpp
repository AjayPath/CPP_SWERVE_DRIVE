// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "IO/PigeonGyro.h"

PigeonGyro* PigeonGyro::instance = NULL;

/**
 * @brief Gets the singleton instance of PigeonGyro
 * @return Pointer to the singleton PigeonGyro instance
 */
PigeonGyro* PigeonGyro::GetInstance() {
    if (instance == NULL) {
        instance = new PigeonGyro();
    } 
    return instance;
}

/**
 * @brief Constructor for PigeonGyro - initializes hardware and timers
 */
PigeonGyro::PigeonGyro(void) {
    Gyro = new ctre::phoenix6::hardware::Pigeon2(PIGEON_ID, "");
    GyroAngleTimer = new frc::Timer();
    GyroAngleTimer->Start();
    GyroPitchTimer = new frc::Timer();
    GyroPitchTimer->Start();
    GyroRollTimer = new frc::Timer();
    GyroRollTimer->Start();

    GyroAngleSignal = &Gyro->GetYaw();
    GyroPitchSignal = &Gyro->GetRoll();
}

/**
 * @brief Get the current yaw angle of the robot
 * @return Current angle in degrees
 */
units::degree_t PigeonGyro::GetAngle() {
    Gyro->GetYaw().Refresh();
    return units::degree_t(Gyro->GetYaw().GetValue());
}

/**
 * @brief Get the current pitch angle of the robot
 * @return Current pitch in degrees
 */
units::degree_t PigeonGyro::GetPitch() {
    return units::degree_t(Gyro->GetPitch().GetValue());
}

/**
 * @brief Get the current roll angle of the robot
 * @return Current roll in degrees
 */
units::degree_t PigeonGyro::GetRoll() {
    return units::degree_t(Gyro->GetRoll().GetValue());
}

/**
 * @brief Get the rate of change of the yaw angle
 * @return Angular velocity in degrees per second
 */
double PigeonGyro::GetAngleRate() {
    double res = (GetAngle().value() - lastAngle) / GyroAngleTimer->Get().value();
    lastAngle = GetAngle().value();
    GyroAngleTimer->Reset();
    return res;
}

/**
 * @brief Get the acceleration in the X-axis
 * @return X-axis acceleration in m/s²
 */
double PigeonGyro::GetXAccel() {
    return Gyro->GetAccelerationX().GetValue().value();
}

/**
 * @brief Get the acceleration in the Y-axis
 * @return Y-axis acceleration in m/s²
 */
double PigeonGyro::GetYAccel() {
    return Gyro->GetAccelerationY().GetValue().value();
}

/**
 * @brief Get the acceleration in the Z-axis
 * @return Z-axis acceleration in m/s²
 */
double PigeonGyro::GetZAccel() {
    return Gyro->GetAccelerationZ().GetValue().value();
}

/**
 * @brief Set the yaw angle of the Pigeon 2 gyroscope
 * @param pos The angle to set in degrees
 */
void PigeonGyro::SetAngle(double pos) {
    Gyro->SetYaw(units::angle::degree_t(pos));
}