// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Subsystems/RevSwerveModule.h"

RevSwerveModule::RevSwerveModule(int driveID, int steerID,
                                 units::inch_t _xOffset, units::inch_t _yOffset,
                                 bool invertDriveMotor, bool invertSteerMotor) {

    DriveMotor = new rev::spark::SparkMax(driveID, rev::spark::SparkMax::MotorType::kBrushless);
    SteerMotor = new rev::spark::SparkMax(steerID, rev::spark::SparkMax::MotorType::kBrushless);

    isDriveInverted = invertDriveMotor;
    isSteerInverted = invertSteerMotor;

    absoluteEncoderOffset = 0.0;
    lastCommandedAngle = 0_deg;
    isCoastMode = true;

    offset = {_xOffset, _yOffset};

    ConfigureDriveMotor();
    ConfigureSteerMotor();

}

RevSwerveModule::~RevSwerveModule() {
    delete DriveMotor;
    delete SteerMotor;
}

void RevSwerveModule::ConfigureDriveMotor() {
    double positionFactor = DRIVE_GEAR_RATIO * (WHEEL_DIAMETER.value() * std::numbers::pi);
    double velocityFactor = positionFactor / 60.0;

    double freeSpeedRPS = 5676.0 / 60;
    double freeSpeedUnits = freeSpeedRPS * positionFactor;
    double velocityFF = 1.0 / freeSpeedUnits;

    driveConfig
        .SetIdleMode(rev::spark::SparkMaxConfig::IdleMode::kCoast)
        .SmartCurrentLimit(50);

    driveConfig.encoder
        .PositionConversionFactor(positionFactor)
        .VelocityConversionFactor(velocityFactor);

    driveConfig.closedLoop
        .SetFeedbackSensor(rev::spark::ClosedLoopConfig::FeedbackSensor::kPrimaryEncoder)
        .P(0.04)
        .I(0.0)
        .D(0.0)
        .VelocityFF(velocityFF)
        .OutputRange(-1.0, 1.0);
    
    driveConfig.Inverted(isDriveInverted);

    DriveMotor->Configure(driveConfig,
                          rev::spark::SparkMax::ResetMode::kNoResetSafeParameters,
                          rev::spark::SparkMax::PersistMode::kNoPersistParameters);
        
}

void RevSwerveModule::ConfigureSteerMotor() {
    double turningFactor = 360.0;

    steerConfig
        .SetIdleMode(rev::spark::SparkMaxConfig::IdleMode::kBrake)
        .SmartCurrentLimit(20);

    steerConfig.absoluteEncoder
        .Inverted(true)
        .PositionConversionFactor(turningFactor)
        .VelocityConversionFactor(turningFactor / 60.0);

    steerConfig.closedLoop
        .SetFeedbackSensor(rev::spark::ClosedLoopConfig::FeedbackSensor::kAbsoluteEncoder)
        .P(1.0)
        .I(0.0)
        .D(0.0)
        .OutputRange(-1.0, 1.0)
        .PositionWrappingEnabled(true)
        .PositionWrappingMinInput(0.0)
        .PositionWrappingMaxInput(turningFactor);

    steerConfig.Inverted(isSteerInverted);

    SteerMotor->Configure(steerConfig,
                          rev::spark::SparkMax::ResetMode::kNoResetSafeParameters,
                          rev::spark::SparkMax::PersistMode::kNoPersistParameters);

}

void RevSwerveModule::TeleopInit() {

}

void RevSwerveModule::SetVelocity(ModuleVelocity _velocity) {

    _velocity = OptimizeState(_velocity);

    auto driveController = DriveMotor->GetClosedLoopController();
    auto steerController = SteerMotor->GetClosedLoopController();

    driveController.SetReference(_velocity.magnitude.value() * (WHEEL_DIAMETER.value() * std::numbers::pi),
                                 rev::spark::SparkMax::ControlType::kVelocity);

    double angleSetpoint = _velocity.direction.value();

    while (angleSetpoint < 0.0) angleSetpoint += 360.0;
    while (angleSetpoint >= 360.0) angleSetpoint -= 360.0;

    steerController.SetReference(angleSetpoint,
                                 rev::spark::SparkMax::ControlType::kPosition);

    lastCommandedAngle = _velocity.direction;

}

void RevSwerveModule::ResetDriveEncoder() {
    auto encoder = DriveMotor->GetEncoder();
    encoder.SetPosition(0.0);
}

units::degree_t RevSwerveModule::GetSteerAngle() {
    auto absEncoder = SteerMotor->GetAbsoluteEncoder();
    return units::degree_t(absEncoder.GetPosition());
}

units::degree_t RevSwerveModule::GetSteerAngle360() {
    double angle = GetSteerAngle().value();

    while (angle < 0.0) angle += 360.0;
    while (angle >= 360.0) angle -= 360.0;

    return units::degree_t(angle);
}

units::turns_per_second_t RevSwerveModule::GetDriveVelocity() {
    auto encoder = DriveMotor->GetEncoder();
    double velocity_in_per_sec = encoder.GetVelocity();
    double velocity_tps = velocity_in_per_sec / (WHEEL_DIAMETER.value() * std::numbers::pi) / DRIVE_GEAR_RATIO;
    return units::turns_per_second_t(velocity_tps);
}

void RevSwerveModule::BrakeDriveMotor(bool _brakesOn) {
    if (_brakesOn) {
        driveConfig.SetIdleMode(rev::spark::SparkMaxConfig::IdleMode::kBrake);
        isCoastMode = false;
    } else {
        driveConfig.SetIdleMode(rev::spark::SparkMaxConfig::IdleMode::kCoast);
        isCoastMode = true;
    }

    DriveMotor->Configure(driveConfig,
                          rev::spark::SparkMax::ResetMode::kNoResetSafeParameters,
                          rev::spark::SparkMax::PersistMode::kNoPersistParameters);

}

bool RevSwerveModule::IsCoast() {
    return isCoastMode;
}

double RevSwerveModule::GetDriveAppliedOutput() {
    return DriveMotor->GetAppliedOutput() * 12.0;
}

units::inch_t RevSwerveModule::GetDrivePosition() {
    auto encoder = DriveMotor->GetEncoder();
    return units::inch_t(encoder.GetPosition());
}

double RevSwerveModule::GetDriveTemperature() {
    return DriveMotor->GetMotorTemperature();
}

double RevSwerveModule::GetSteerTemperature() {
    return SteerMotor->GetMotorTemperature();
}

CustomPose RevSwerveModule::GetPose() {
    return {offset.GetX(), offset.GetY(), GetSteerAngle()};
}

void RevSwerveModule::SetEncoderOffset(double _offset) {
    absoluteEncoderOffset = _offset;
    steerConfig.absoluteEncoder.ZeroOffset(_offset);
    SteerMotor->Configure(steerConfig,
                          rev::spark::SparkMax::ResetMode::kResetSafeParameters,
                          rev::spark::SparkMax::PersistMode::kPersistParameters);
}

double RevSwerveModule::GetAbsoluteEncoderRaw() {
    auto absEncoder = SteerMotor->GetAbsoluteEncoder();
    return absEncoder.GetPosition() / 360.0;
}

ModuleVelocity RevSwerveModule::OptimizeState(ModuleVelocity _state) {
    
    units::degree_t currentAngle = GetSteerAngle360();
    units::degree_t targetAngle = _state.direction;
    
    while (targetAngle < 0_deg) targetAngle += 360_deg;
    while (targetAngle >= 360_deg) targetAngle -= 360_deg;
    
    units::degree_t delta = targetAngle - currentAngle;
    
    while (delta > 180_deg) delta -= 360_deg;
    while (delta < -180_deg) delta += 360_deg;
    
    if (units::math::fabs(delta) > 90_deg) {
        
        _state.magnitude *= -1.0;

        if (delta > 90_deg) {
            _state.direction = targetAngle - 180_deg;
        } 
        else {
            _state.direction = targetAngle + 180_deg;
        }
        
        while (_state.direction < 0_deg) _state.direction += 360_deg;
        while (_state.direction >= 360_deg) _state.direction -= 360_deg;
    
    }
    
    return _state;

}