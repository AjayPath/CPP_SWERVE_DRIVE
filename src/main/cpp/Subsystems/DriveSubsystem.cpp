// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Subsystems/DriveSubsystem.h"
#include "Subsystems/RevSwerveModule.h"

DriveSubsystem::DriveSubsystem(void) {
    swerveModule = new RevSwerveModule(
        13,
        14,
        0_in,
        0_in,
        false,
        false
    );

    Gyro = PigeonGyro::GetInstance();

    DrivePID = new SimPID();
    TurnPID = new SimPID();

    xLimiter = new SlewRateLimiter(600.0, 20000.0);
    yLimiter = new SlewRateLimiter(600.0, 20000.0);
    rLimiter = new SlewRateLimiter(16, 240);

    brakeTimer = new frc::Timer();
    holdHeadingTimer = new frc::Timer();

    swerveModuleVelocity = {0_tps, 0_deg};
    lastSwerveHeading = swerveModule->GetSteerAngle();

}

DriveSubsystem::~DriveSubsystem(void) {}

void DriveSubsystem::RobotInit() {
    TurnPID->setConstants(TurnP, TurnI, TurnD);
    DrivePID->setConstants(DriveP, DriveI, DriveD);

    Gyro->SetAngle(0);

    heading = Gyro->GetAngle();
    oldHeading = 0.0;

    swerveModule->SetEncoderOffset(FR_STEER_ENCODER_OFFSET_PRACTICE);

}

void DriveSubsystem::RobotPeriodic() {

}

void DriveSubsystem::DisabledInit() {

}

void DriveSubsystem::DisabledPeriodic() {
    if (resetOffsetValues) {
        swerveModule->SetEncoderOffset(0);
        resetOffsetValues = false;
    }
    if (brakeTimer->Get() > 3_s && brakesEnabled == true) {
        swerveModule->BrakeDriveMotor(false);
        brakesEnabled = false;
    }
}

void DriveSubsystem::TeleopInit() {
    heading = Gyro->GetAngle();
    oldHeading = Gyro->GetAngle().value();
    currState = FIELD_DRIVE;
    if (brakesEnabled == false) {
        swerveModule->BrakeDriveMotor(true);
    }
    brakesEnabled = true;
}

void DriveSubsystem::TeleopPeriodic() {
    StateMachine();
}

void DriveSubsystem::AutonomousInit() {

}

void DriveSubsystem::AutonomousPeriodic() {

}

void DriveSubsystem::ShowOnDashboard() {
    
}

void DriveSubsystem::StateMachine() {
    double xVel = Calculations::SignSquare(GamePads->Driver->GetLeftY()) * MAX_SPEED;
    double yVel = Calculations::SignSquare(GamePads->Driver->GetLeftX()) * MAX_SPEED;
    double rVel = Calculations::SignSquare(GamePads->Driver->GetRightX()) * MAX_ROTATIONAL_SPEED;

    double _desiredHeading;
    
    switch (currState)
    {
        case IDLE:
            if (xVel != 0 || yVel != 0 || rVel != 0) {
                currState = FIELD_DRIVE;
            }
            oldHeading = Gyro->GetAngle().value();
            holdHeadingTimer->Reset();
            break;

        case FIELD_DRIVE:
            if (rVel == 0 && holdHeadingTimer->Get() > 350_ms) {
                TurnPID->setDesiredValue(oldHeading);
                rVel = MAX_ROTATIONAL_SPEED * TurnPID->calcPID(Gyro->GetAngle().value());
            }
            else if (rVel != 0) {
                holdHeadingTimer->Reset();
            }
            else {
                oldHeading = Gyro->GetAngle().value();
            }
            FieldCentricDrive(xVel, yVel, rVel);
            break;

        case ROBOT_DRIVE:
            RobotCentricDrive(xVel, yVel, rVel);
            oldHeading = Gyro->GetAngle().value();
            holdHeadingTimer->Reset();
            break;
        
        default:
            break;
    }

    if (GamePads->Driver->GetButton(D_FIELD_DRIVE)) {
        currState = FIELD_DRIVE;
    }
    else if (GamePads->Driver->GetButton(D_ROBOT_DRIVE)) {
        currState = ROBOT_DRIVE;
    }

}

void DriveSubsystem::RobotCentricDrive(double xVel, double yVel, double rVel) {
    // Calculate magnitude based on x, y components
    double vMag = std::sqrt(xVel * xVel + yVel * yVel);
    
    // Scale x, y so magnitude does not exceed max
    if (vMag > MAX_SPEED) {
        xVel = (xVel / vMag) * MAX_SPEED;
        yVel = (yVel / vMag) * MAX_SPEED;
    }
    
    // Convert velocity to turns per second
    xMag = (units::turns_per_second_t(xVel) / (WHEEL_DIAMETER * std::numbers::pi)) / DRIVE_GEAR_RATIO;
    yMag = (units::turns_per_second_t(yVel) / (WHEEL_DIAMETER * std::numbers::pi)) / DRIVE_GEAR_RATIO;
    
    // Calculate module velocity (for single module, rotation just adds to translation)
    units::turns_per_second_t totalMag = units::math::sqrt(
        units::math::pow<2>(xMag) + units::math::pow<2>(yMag)
    );
    
    // Calculate direction angle
    units::degree_t direction = units::math::atan2(yMag, xMag);
    
    // If rotation is requested, add it to magnitude
    if (std::abs(rVel) > 0.01) {
        totalMag += units::turns_per_second_t(std::abs(rVel));
    }
    
    // Clamp to max speed
    if (totalMag > MAX_SPEED_TPS) {
        totalMag = MAX_SPEED_TPS;
    }
    
    swerveModuleVelocity = {totalMag, direction};
    
    // Only set velocity if above threshold, otherwise maintain last heading
    if (units::math::fabs(totalMag) > 0.5_tps) {
        swerveModule->SetVelocity(swerveModuleVelocity);
        lastSwerveHeading = direction;
    } else {
        swerveModule->SetVelocity({0_tps, lastSwerveHeading});
    }
}

void DriveSubsystem::FieldCentricDrive(double xVel, double yVel, double rVel, double robotYComp) {
    // Get current robot angle
    double currAngle = Gyro->GetAngle().value();
    
    // Apply slew rate limiting
    xVel = xLimiter->Calculate(xVel);
    yVel = yLimiter->Calculate(yVel);
    rVel = rLimiter->Calculate(rVel);
    
    // Convert field-centric to robot-centric
    double robotCentricX = xVel * cos(currAngle * DEG_TO_RAD) + yVel * sin(currAngle * DEG_TO_RAD);
    double robotCentricY = -xVel * sin(currAngle * DEG_TO_RAD) + yVel * cos(currAngle * DEG_TO_RAD);
    
    // Apply Y compensation
    if (robotYComp != 0) {
        robotYComp -= robotCentricY;
    }
    
    // Reset compensation if moving backwards
    if (robotCentricX < 0) {
        robotYComp = 0.0;
    }
    
    // Pass to robot-centric drive with compensation
    RobotCentricDrive(robotCentricX, robotCentricY + robotYComp, rVel);
}

void DriveSubsystem::ResetDrive() {
    Gyro->SetAngle(0.0);
    heading = Gyro->GetAngle();
    holdHeadingTimer->Reset();
}


void DriveSubsystem::SetState(DriveStates _state) {
    currState = _state;
}

DriveSubsystem::DriveStates DriveSubsystem::GetState() {
    return currState;
}

std::string DriveSubsystem::GetStateStr() {
    switch (currState) {
        case IDLE:
            return "IDLE";
            break;

        case FIELD_DRIVE:
            return "FIELD_DRIVE";
            break;

        case ROBOT_DRIVE:
            return "ROBOT_DRIVE";
            break;

        default:
            return "DRIVE BASE IS REALLY BROKEN!!!!!!!!";
            break;
    }
}

DriveSubsystem* DriveSubsystem::GetInstance() {
    if (instance == NULL) {
        instance = new DriveSubsystem();
    }
    return instance;
}

DriveSubsystem* DriveSubsystem::instance = NULL;