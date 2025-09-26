// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <units/math.h>
#include <units/length.h>

#include "Calculations.h"

// A Vector is a quanity with a magnitude and direction
// The displacement of an object

class CustomVector {
 public:

  // ============================================================================
  // CONSTRUCTORS
  // ============================================================================

  /**
   * @brief Default constructor
   */
  CustomVector() {}

  /**
   * @brief Create vector from x,y coordinates
   * @param _x X coordinate
   * @param _y Y coordinate
   */
  CustomVector(units::inch_t _x, units::inch_t _y) {
    x = _x;
    y = _y;
    angle = units::math::atan2(y, x);
    mag = units::math::sqrt(units::math::pow<2>(x) + units::math::pow<2>(y));
  }

  /**
   * @brief Create vector from x,y coordinates with angle override
   * @param _x X coordinate
   * @param _y Y coordinate
   * @param _angle Override angle for zero vectors
   */
  CustomVector(units::inch_t _x, units::inch_t _y, units::degree_t _angle) {
    x = _x;
    y = _y;

    if (x == 0_in && y == 0_in) {
      angle = _angle;
    }
    else {
      angle = units::math::atan2(y, x);
    }
    mag = units::math::sqrt(units::math::pow<2>(x) + units::math::pow<2>(y));
  }

  /**
   * @brief Create vector from magnitude and angle
   * @param _mag Vector magnitude
   * @param _angle Vector angle
   */
  CustomVector(units::inch_t _mag, units::degree_t _angle) {
    mag = _mag;
    angle = _angle;
    Calculations::NormalizeAngle(angle);
    x = mag * units::math::cos(angle);
    y = mag * units::math::sin(angle);
  }

  // ============================================================================
  // VECTOR OPERATIONS
  // ============================================================================

  /**
   * @brief Subtract vector from this vector
   * @param v Vector to subtract
   */
  void SubtractFrom(CustomVector v) {
    x -= v.GetX();
    y -= v.GetY();
    angle = units::math::atan2(y, x);
    mag = units::math::sqrt(units::math::pow<2>(x) + units::math::pow<2>(y));
  }

  /**
   * @brief Add vector to this vector
   * @param v Vector to add
   */
  void Add(CustomVector v) {
    x += v.GetX();
    y += v.GetY();
    angle = units::math::atan2(y, x);
    mag = units::math::sqrt(units::math::pow<2>(x) + units::math::pow<2>(y));
  }

  /**
   * @brief Rotate vector by gyro angle
   * @param GyroAngle Rotation angle
   */
  void TransformVector(units::degree_t GyroAngle) {
    angle += GyroAngle;
    x = mag * units::math::cos(angle);
    y = mag * units::math::sin(angle);
  } 

  /**
   * @brief Reflect vector across Y-axis
   */
  void ReflectY() {
    x = -x;
    angle = units::math::atan2(units::math::sin(angle), -units::math::cos(angle));
  }

  // ============================================================================
  // GETTERS & SETTERS
  // ============================================================================

  /**
   * @brief Get X coordinate
   * @return X coordinate value
   */
  units::inch_t GetX() {
    return x;
  }

  /**
   * @brief Get Y coordinate
   * @return Y coordinate value
   */
  units::inch_t GetY() {
    return y;
  }

  /**
   * @brief Get magnitude
   * @return Vector magnitude
   */
  units::inch_t GetMag() {
    return mag;
  }

  /**
   * @brief Get angle
   * @return Vector angle
   */
  units::degree_t GetAngle() {
    return angle;
  }

  /**
   * @brief Set Y coordinate
   * @param _y New Y value
   */
  void SetY(units::inch_t _y) {
    y = _y;
    angle = units::math::atan2(y, x);
    mag = units::math::sqrt(units::math::pow<2>(x) + units::math::pow<2>(y));
  }

  /**
   * @brief Set X coordinate
   * @param _x New X value
   */
  void SetX(units::inch_t _x) {
    x = _x;
    angle = units::math::atan2(y, x);
    mag = units::math::sqrt(units::math::pow<2>(x) + units::math::pow<2>(y));
  }

  /**
   * @brief Set magnitude
   * @param _mag New magnitude value
   */
  void SetMag(units::inch_t _mag) {
    x = _mag * units::math::cos(angle);
    y = _mag * units::math::sin(angle);
    mag = _mag;
  }

  /**
   * @brief Set angle
   * @param _angle New angle value
   */
  void SetAngle(units::degree_t _angle) {
    x = mag * units::math::cos(_angle);
    y = mag * units::math::sin(_angle);
    angle = _angle;
  }

  // ============================================================================
  // PRINT STATEMENTS
  // ============================================================================

  /**
   * @brief Print vector with prefix
   * @param pre Prefix string
   */
  void Print(const char* pre) {
    printf("%s: x:%f y:%f a:%f\n", pre, x.value(), y.value(), angle.value());
  }

  /**
   * @brief Print vector values
   */
  void Print() {
    printf("x:%f y:%f a:%f\n", x.value(), y.value(), angle.value());
  }

  /**
   * @brief Default constructor
   */
  CustomVector() {}

 private:
  units::inch_t x = 0_in;
  units::inch_t y = 0_in;
  units::inch_t mag = 0_in;
  units::degree_t angle = 0_deg;

};