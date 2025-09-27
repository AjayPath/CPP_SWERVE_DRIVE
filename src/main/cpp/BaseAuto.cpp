// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "BaseAuto.h"

// include all autos here?

BaseAuto *BaseAuto::instance = NULL;

BaseAuto::BaseAuto() {
    SelectedAutoMode = 0.0;
    AutoDelay = 0.0;

    // Get subystem instances

    AutoTimer = new frc::Timer();
    AutoAwaitTimer = new frc::Timer();
    DelayTimer = new frc::Timer();

    AutoModes = std::vector<AutoMode *>({});

}

BaseAuto::~BaseAuto() {}

BaseAuto *BaseAuto::GetInstance() {
    if (instance == NULL) {
        instance = new BaseAuto();
    }
    return instance;
}

void BaseAuto::SensorReset() {
    //reset drive sensor
}

void BaseAuto::StartTimers() {
    DelayTimer->Start();
    DelayTimer->Reset();
    AutoAwaitTimer->Start();
}

void BaseAuto::AutoSelector() {
    if (AutoAwaitTimer->Get() > 3.1_s && !GamePads->Driver->GetButton(CustomXbox::Button::LS) && !GamePads->Driver->GetButton(CustomXbox::Button::RS)) {
        for (int i = 0; i < (int)AutoModes.size(); i++) {
            if (GamePads->Driver->GetButton(static_cast<CustomXbox::Button>(i + 1))) {
                SelectedAutoMode = i;
                if (fabs(GamePads->Driver->GetLeftY()) > 0.01) {
                    AutoDelay = (round(fabs(GamePads->Driver->GetLeftY()) * 10 * 2.0) / 2.0);
                }
            }
        }
    } 
    if (GamePads->Driver->GetButton(CustomXbox::Button::LS)) {
        AutoDelay = 0.0;
    }
}

int BaseAuto::GetAutoMode() {
    return SelectedAutoMode;
}

double BaseAuto::GetAutoDelay() {
    return AutoDelay;
}

void BaseAuto::ResetStates() {
    // Reset drive state
}

void BaseAuto::DisabledInit() {
    AutoAwaitTimer->Start();
    AutoAwaitTimer->Reset();
}

void BaseAuto::AutonomousInit() {
    SensorReset();
    StartTimers();
    ResetStates();

    AutoModes[SelectedAutoMode]->ResetAuto();
    AutoModes[SelectedAutoMode]->ReflectIfRed();
}

void BaseAuto::AutonomousPeriodic() {
    if (DelayTimer->Get().value() > AutoDelay) {
        AutoModes[SelectedAutoMode]->Run();
    }
}

std::string BaseAuto::GetAutoModeString() {
    return AutoModes[SelectedAutoMode]->GetName();
}

void BaseAuto::ShowOnDashboard() {
    frc::SmartDashboard::PutString("CurrentAuto", GetAutoModeString());
    frc::SmartDashboard::PutNumber("DelayTimer", GetAutoDelay());
    frc::SmartDashboard::PutNumber("Auto Step", AutoModes[SelectedAutoMode]->GetStep());
}

