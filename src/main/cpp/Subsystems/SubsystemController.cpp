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

} 

SubsystemController::~SubsystemController(void) {

}

// FRC Functions
void SubsystemController::RobotInit() {
    
}

void SubsystemController::RobotPeriodic() {
    
}

void SubsystemController::DisabledInit() {
    
}

void SubsystemController::DisabledPeriodic() {
    
}

void SubsystemController::TeleopInit() {
    
}

void SubsystemController::TeleopPeriodic() {
    
}

void SubsystemController::AutonomousInit() {
    
}

void SubsystemController::AutonomousPeriodic() {
}

void SubsystemController::ShowOnDashboard() {
    
}
