// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "IO/CustomXbox.h"

/**
 * @brief Constructor for CustomXbox controller wrapper
 * @param port The port number the Xbox controller is connected to
 */
CustomXbox::CustomXbox(int port) {
    joystick = new frc::XboxController(port);
    checkJoystick = false;
}

/**
 * @brief Destructor for CustomXbox
 */
CustomXbox::~CustomXbox() {}

/**
 * @brief Checks if a button is currently pressed
 * @param _button The button to check (supports standard buttons, triggers, and D-pad)
 * @return True if button is pressed and controller is connected, false otherwise
 */
bool CustomXbox::GetButton(Button _button) {
    if ((joystick->GetPort() == 1 && joystick->IsConnected()) || joystick->GetPort() == 0)
    {
        checkJoystick = true;
    } else {
        checkJoystick = false;
    }

    if (checkJoystick) {

        if (_button == RT) {
            return joystick->GetRightTriggerAxis() >= TRIGGER_DEADBAND;
        }
        else if (_button == LT) {
            return joystick->GetLeftTriggerAxis() >= TRIGGER_DEADBAND;
        }
        else if (_button == DU) {
            return joystick->GetPOV() == 0;
        }
        else if (_button == DR) {
            return joystick->GetPOV() == 90;
        }
        else if (_button == DD) {
            return joystick->GetPOV() == 180;
        }
        else if (_button == DL) {
            return joystick->GetPOV() == 270;
        }

        return joystick->GetRawButton(_button);

    }
    else {
        return false;
    }

}

/**
 * @brief Checks if a button has been released
 * @param _button The button to check for release
 * @return True if button was released and controller is connected, false otherwise
 */
bool CustomXbox::GetButtonReleased(Button _button) {
    if ((joystick->GetPort() == 1 && joystick->IsConnected()) || joystick->GetPort() == 0) {
        return joystick->GetRawButtonReleased(_button);
    }
    else {
        return false;
    }
}

/**
 * @brief Gets the left joystick X axis value with deadband
 * @return X axis value between -1 and 1, or 0 if controller disconnected
 */
double CustomXbox::GetLeftX() {
    if ((joystick->GetPort() == 1 && joystick->IsConnected()) || joystick->GetPort() == 0) {
        checkJoystick = true;
    }
    else {
        checkJoystick = false;
    }
    if (checkJoystick) {
        if (fabs(joystick->GetLeftX()) < STICK_DEADBAND) {
            return 0.0;
        }
        return -joystick->GetLeftX();
    }
    else {
        return 0.0;
    }
}

/**
 * @brief Gets the left joystick Y axis value with deadband
 * @return Y axis value between -1 and 1, or 0 if controller disconnected
 */
double CustomXbox::GetLeftY() {
    if ((joystick->GetPort() == 1 && joystick->IsConnected()) || joystick->GetPort() == 0) {
        checkJoystick = true;
    }
    else {
        checkJoystick = false;
    }
    if (checkJoystick) {
        if (fabs(joystick->GetLeftY()) < STICK_DEADBAND) {
            return 0.0;
        }
        return -joystick->GetLeftY();
    }
    else {
        return 0.0;
    }
}

/**
 * @brief Gets the right joystick X axis value with deadband
 * @return X axis value between -1 and 1, or 0 if controller disconnected
 */
double CustomXbox::GetRightX() {
    if ((joystick->GetPort() == 1 && joystick->IsConnected()) || joystick->GetPort() == 0) {
        checkJoystick = true;
    }
    else {
        checkJoystick = false;
    }
    if (checkJoystick) {
        if (fabs(joystick->GetRightX()) < STICK_DEADBAND) {
            return 0.0;
        }
        return -joystick->GetRightX();
    }
    else {
        return 0.0;
    }
}

/**
 * @brief Gets the right joystick Y axis value with deadband
 * @return Y axis value between -1 and 1, or 0 if controller disconnected
 */
double CustomXbox::GetRightY() {
    if ((joystick->GetPort() == 1 && joystick->IsConnected()) || joystick->GetPort() == 0) {
        checkJoystick = true;
    }
    else {
        checkJoystick = false;
    }
    if (checkJoystick) {
        if (fabs(joystick->GetRightY()) < STICK_DEADBAND) {
            return 0.0;
        }
        return -joystick->GetRightY();
    }
    else {
        return 0.0;
    }
}

/**
 * @brief Calculates the angle of the left joystick in degrees
 * @return Angle in degrees from -180 to 180, useful for swerve drive direction
 */
double CustomXbox::GetLeftPlusMinus180() {
    return atan2(GetLeftX(), GetLeftY()) * (180 / std::numbers::pi);
}

/**
 * @brief Sets controller rumble/vibration intensity
 * @param intensity Rumble strength from 0.0 (off) to 1.0 (maximum)
 */
void CustomXbox::SetRumble(double intensity) {
    if ((joystick->GetPort() == 1 && joystick->IsConnected()) || joystick->GetPort() == 0) {
        joystick->SetRumble(frc::XboxController::RumbleType::kBothRumble, intensity);
    }
}