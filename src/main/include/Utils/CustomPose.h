// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once
#include <units/angle.h>
#include <units/velocity.h>
#include <units/angular_velocity.h>
#include <units/math.h>
#include <units/length.h>
#include <Utils/CustomVector.h>

/**
 * @brief Represents a robot's pose (position and orientation) on the field
 */
class CustomPose {
 public:

  // ============================================================================
  // CONSTRUCTORS
  // ============================================================================
  
  /**
   * @brief Constructs a CustomPose with specified position and angle
   * @param _x The x-coordinate in inches
   * @param _y The y-coordinate in inches
   * @param _angle The angle in degrees
   */
  CustomPose(units::inch_t _x, units::inch_t _y, units::degree_t _angle) {
    x = _x;
    y = _y;
    angle = _angle;
  }

  /**
   * @brief Default constructor - initializes pose to origin with 0 degree angle
   */
  CustomPose() {
    x = 0_in;
    y = 0_in;
    angle = 0_deg;  //90_deg, I belive changes for starting orientation
  }

  // ============================================================================
  // FUNCTIONS
  // ============================================================================

  /**
   * @brief Reflects the pose across the Y-axis (flips x-coordinate and adjusts angle)
   */
  void ReflectY() {
    x = -x;
    angle = units::math::atan2(units::math::sin(angle), -units::math::cos(angle));
  }

  /**
   * @brief Adds a vector to the current pose position
   * @param v The vector to add to the pose
   */
  void Add(CustomVector v) {
    x += v.GetX();
    y += v.GetY();
  }

  /**
   * @brief Subtracts a vector from the current pose position
   * @param v The vector to subtract from the pose
   */
  void Subtract(CustomVector v) {
    x -= v.GetX();
    y -= v.GetY();
  }

  /**
   * @brief Calculates the displacement vector between this pose and another pose
   * @param p The pose to subtract from this pose
   * @return CustomVector representing the displacement
   */
  CustomVector Subtract(CustomPose p) {
    return {x - p.GetX(), y - p.GetY()};
  }

  /**
   * @brief Sets the pose to specified position and angle
   * @param _x The new x-coordinate in inches
   * @param _y The new y-coordinate in inches
   * @param _angle The new angle in degrees
   */
  void SetPose(units::inch_t _x, units::inch_t _y, units::degree_t _angle) {
    x = _x;
    y = _y;
    angle = _angle;
  }

  /**
   * @brief Sets the pose to match another pose
   * @param _pose The pose to copy values from
   */
  void SetPose(CustomPose _pose) {
    x = _pose.GetX();
    y = _pose.GetY();
    angle = _pose.GetAngle();
  }

  /**
   * @brief Updates the pose angle based on gyroscope reading
   * @param GyroAngle The gyroscope angle to add to current angle
   */
  void Transform(double GyroAngle) {
    angle += units::degree_t(GyroAngle);
    Calculations::NormalizeAngle(angle);
  }

  // ============================================================================
  // GETTERS & SETTERS
  // ============================================================================

  /**
   * @brief Gets the x-coordinate of the pose
   * @return The x-coordinate in inches
   */
  units::inch_t GetX() {
    return x;
  }

  /**
   * @brief Gets the y-coordinate of the pose
   * @return The y-coordinate in inches
   */
  units::inch_t GetY() {
    return y;
  }

  /**
   * @brief Gets the angle of the pose
   * @return The angle in degrees
   */
  units::degree_t GetAngle() {
    return angle;
  }

  /**
   * @brief Sets the x-coordinate of the pose
   * @param _x The new x-coordinate in inches
   */
  void SetX(units::inch_t _x) {
    x = _x;
  }

  /**
   * @brief Sets the y-coordinate of the pose
   * @param _y The new y-coordinate in inches
   */
  void SetY(units::inch_t _y) {
    y = _y;
  }

  /**
   * @brief Sets the angle of the pose
   * @param _angle The new angle in degrees
   */
  void SetAngle(units::degree_t _angle) {
    angle = _angle;
  }

  // ============================================================================
  // PRINT STATEMENTS
  // ============================================================================

  /**
   * @brief Prints the pose with a custom prefix
   * @param pre The prefix string to display before pose values
   */
  void Print(const char* pre) {
    printf("%s: x:%f y:%f a:%f\n", pre, x.value(), y.value(), angle.value());
  }

  /**
   * @brief Prints the pose with a custom prefix and index number
   * @param pre The prefix string to display before pose values
   * @param i The index number to display
   */
  void Print(const char* pre, int i) {
    printf("%s: %d: x:%f y:%f a:%f\n", pre, i, x.value(), y.value(), angle.value());
  }

  /**
   * @brief Prints the pose with just an index number
   * @param i The index number to display before pose values
   */
  void Print(int i) {
    printf("%d: x:%f y:%f a:%f\n", i, x.value(), y.value(), angle.value());
  }

  /**
   * @brief Prints the pose values without any prefix
   */
  void Print() {
    printf("x:%f y:%f a:%f\n", x.value(), y.value(), angle.value());
  }

 private:
  units::inch_t x;    ///< X-coordinate of the pose in inches
  units::inch_t y;    ///< Y-coordinate of the pose in inches
  units::degree_t angle; ///< Angle of the pose in degrees

};