// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once
#include <units/angle.h>

class Calculations {
 public:
  
  /// @brief Square the input and keep the sign
  /// @param input The value to square while preserving its sign
  /// @return The squared value with original sign preserved
  static double SignSquare(double input) {
    if (input < 0) {
      return (-input * input);
    } 
    else {
      return (input * input);
    }
  }

  /// @brief Convert angle to -180 to 180
  /// @param input Reference to the angle in degrees to normalize (modified in place)
  static void NormalizeAngle(units::angle::degree_t &input) {
    while (input < -180_deg) {
      input += 360_deg;
    }
    while (input > 180_deg) {
      input -= 360_deg;
    }
  }

  /// @brief limit the input value
  /// @param input The value to limit
  /// @param LIMIT The maximum absolute value allowed
  /// @return The input value clamped between -LIMIT and +LIMIT
  static double LimitOutput(double input, const double LIMIT) {
    if (input > LIMIT) {
      return LIMIT;
    }
    else if (input < -LIMIT) {
      return (-LIMIT);
    } else {
      return input;
    }
  }

  /// @brief Calculate the Pythagorean theorem result (hypotenuse) for two double values
  /// @param x First coordinate value
  /// @param y Second coordinate value
  /// @return The distance/hypotenuse calculated as sqrt(x² + y²)
  static double pyth(double x, double y) {
    return sqrt(pow(x, 2) + pow (y, 2));
  }

  /// @brief Calculate the Pythagorean theorem result (hypotenuse) for two integer values
  /// @param x First coordinate value
  /// @param y Second coordinate value  
  /// @return The distance/hypotenuse calculated as sqrt(x² + y²)
  static double pyth(int x, int y) {
    return sqrt(pow(x, 2) + pow(y, 2));
  }

  /// @brief Return the maximum value between two double numbers
  /// @param num1 First number to compare
  /// @param num2 Second number to compare
  /// @return The larger of the two input values
  static double Max(double num1, double num2) {
    if (num1 > num2) {
      return num1;
    } else {
      return num2;
    }
  }

  /// @brief Return the sign of a value (1 for positive/zero, -1 for negative)
  /// @param value The number to check the sign of
  /// @return 1.0 if value is positive or zero, -1.0 if value is negative
  static double GetSign(double value) {
    if (value >= 0.0) {
      return 1;
    } else {
      return -1;
    }
  }

};