// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once
#include <math.h>
#include "./CustomXbox.h"

/**
 * @brief Singleton class that manages two Xbox controllers for robot operation
 * 
 * This class implements the Singleton design pattern to provide global access
 * to two CustomXbox controller instances - one for the driver and one for the
 * operator. This is a common pattern in FRC robotics where you typically have:
 * - Driver controller: controls robot movement (drivetrain)
 * - Operator controller: controls robot mechanisms (arm, shooter, intake, etc.)
 * 
 * The singleton ensures only one instance exists and can be accessed from anywhere
 * in the codebase without passing controller references around.
 */

class Controllers {
 
 public:
  /// @brief Get the singleton instance of the Controllers class
  /// @return Pointer to the single Controllers instance (creates it if it doesn't exist)
  static Controllers* GetInstance();
  
  CustomXbox* Driver;    // Controller for robot driving/movement
  CustomXbox* Operator;  // Controller for robot mechanisms/tools

  private:
   /// @brief Private constructor - prevents direct instantiation (singleton pattern)
   Controllers();
   
   /// @brief Private destructor - handles cleanup of controller instances
   ~Controllers();
   
   /// @brief Static pointer to the single instance of this class
   static Controllers* instance;
};