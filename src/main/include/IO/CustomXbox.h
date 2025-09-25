// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once
#include <frc/XboxController.h>
#include <math.h>

/**
 * @brief Custom wrapper class for Xbox Controller functionality
 * 
 * This class provides a simplified interface for Xbox controller input handling.
 * It maps controller buttons to easy-to-use enums and provides methods for
 * reading analog stick values, button states, and controlling rumble feedback.
 * Includes built-in deadband handling for analog inputs.
 */
class CustomXbox {
 public:

    /**
     * @brief Enumeration of all Xbox controller buttons and inputs
     * 
     * Maps physical controller buttons to easy-to-reference enum values.
     * Includes face buttons, bumpers, triggers, stick clicks, and D-pad directions.
     */
    enum Button {
    A = 1,      // A button (bottom face button)
    B,          // B button (right face button)
    X,          // X button (left face button)
    Y,          // Y button (top face button)
    LB,         // Left bumper
    RB,         // Right bumper
    BACK,       // Back/Select button
    START,      // Start/Menu button
    LS,         // Left stick click
    RS,         // Right stick click

    // Triggers are analog inputs but can be used as buttons
    LT,         // Left trigger
    RT,         // Right trigger

    // D-PAD directional buttons
    DU,         // D-pad Up
    DD,         // D-pad Down
    DL,         // D-pad Left
    DR          // D-pad Right
  };

  /// @brief Constructor - creates Xbox controller interface for specified port
  /// @param port The USB port number the controller is connected to (typically 0-3)
  CustomXbox(int port);
  
  /// @brief Destructor - cleans up controller resources
  ~CustomXbox();

  /// @brief Set controller rumble/vibration intensity
  /// @param intensity Rumble strength from 0.0 (off) to 1.0 (maximum)
  void SetRumble(double intensity);
  
  /// @brief Check if a button is currently pressed
  /// @param _button The button to check (from Button enum)
  /// @return true if button is currently pressed, false otherwise
  bool GetButton(Button _button);
  
  /// @brief Check if a button was just released (useful for single-press detection)
  /// @param _button The button to check (from Button enum) 
  /// @return true if button was just released this frame, false otherwise
  bool GetButtonReleased(Button _button);
  
  /// @brief Get left analog stick X-axis value
  /// @return Horizontal position from -1.0 (full left) to +1.0 (full right)
  double GetLeftX();
  
  /// @brief Get left analog stick Y-axis value  
  /// @return Vertical position from -1.0 (full down) to +1.0 (full up)
  double GetLeftY();
  
  /// @brief Get right analog stick X-axis value
  /// @return Horizontal position from -1.0 (full left) to +1.0 (full right)
  double GetRightX();
  
  /// @brief Get right analog stick Y-axis value
  /// @return Vertical position from -1.0 (full down) to +1.0 (full up)
  double GetRightY();
  
  /// @brief Get left stick angle in degrees from -180 to +180
  /// @return Angle in degrees where 0° is right, 90° is up, ±180° is left, -90° is down
  double GetLeftPlusMinus180();

  private:
    frc::XboxController* joystick;        // Pointer to WPILib Xbox controller object
    const double TRIGGER_DEADBAND = 0.05; // Minimum trigger value to register as pressed
    const double STICK_DEADBAND = 0.05;   // Minimum stick movement to register (prevents drift)
    double currRumble = 0.0;              // Current rumble intensity setting
    bool checkJoystick = false;           // Flag for joystick validation/checking
};