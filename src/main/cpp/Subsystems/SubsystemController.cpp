// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Subsystems/SubsystemController.h"

SubsystemController* SubsystemController::instance = NULL;

SubsystemController* SubsystemController::GetInstance() {
    if (instance == NULL) {
        instance = new SubsystemController();
    }
    return instance;
}

SubsystemController::SubsystemController(void) {
    _Drive = DriveSubsystem::GetInstance();
} 

SubsystemController::~SubsystemController(void) {

}

// FRC Functions
void SubsystemController::RobotInit() {
    _Drive->RobotInit();
}

void SubsystemController::RobotPeriodic() {
    _Drive->RobotPeriodic();
}

void SubsystemController::DisabledInit() {
    _Drive->DisabledInit();
}

void SubsystemController::DisabledPeriodic() {
    _Drive->DisabledPeriodic();
}

void SubsystemController::TeleopInit() {
    _Drive->TeleopInit();
}

void SubsystemController::TeleopPeriodic() {
    _Drive->TeleopPeriodic();
}

void SubsystemController::AutonomousInit() {
    _Drive->AutonomousInit();
}

void SubsystemController::AutonomousPeriodic() {
    _Drive->AutonomousPeriodic();
}

void SubsystemController::ShowOnDashboard() {
    
}
