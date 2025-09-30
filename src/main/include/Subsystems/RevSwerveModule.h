#pragma once
#include "./CustomIncludes.h"
#include <rev/SparkMax.h>
#include <rev/config/SparkMaxConfig.h>

// REV-based swerve module implementation
// Supports SparkMax with NEO motors and Through Bore Absolute Encoders
// Absolute encoder shares CAN ID with turning motor

class RevSwerveModule {
   public:
    /**
     * @brief Construct a new REV Swerve Module
     * @param driveID CAN ID for drive motor (SparkMax)
     * @param steerID CAN ID for steer motor (SparkMax) - absolute encoder shares this ID
     * @param _xOffset X offset from robot center in inches
     * @param _yOffset Y offset from robot center in inches
     * @param invertDriveMotor Invert drive motor direction
     * @param invertSteerMotor Invert steer motor direction (for turning motor, not encoder)
     */
    RevSwerveModule(int driveID, int steerID, 
                    units::inch_t _xOffset, units::inch_t _yOffset, 
                    bool invertDriveMotor = false, bool invertSteerMotor = false);
    
    ~RevSwerveModule();

    // FRC methods
    void TeleopInit();

    /**
     * @brief Set the output of the module to the new swerve state
     * @param _velocity OPRModuleVelocity with magnitude and direction
     */
    void SetVelocity(ModuleVelocity _velocity);

    /**
     * @brief Reset the encoder position of the drive motor to 0
     */
    void ResetDriveEncoder();

    // Getters/Setters

    /**
     * @brief Set the neutral mode of drive motor
     * @param _brakesOn Brakes enabled (true = brake, false = coast)
     */
    void BrakeDriveMotor(bool _brakesOn);

    /**
     * @brief Get the angle of the steering motor in degrees (from absolute encoder)
     * @return Current angle of the module
     */
    units::degree_t GetSteerAngle();

    /**
     * @brief Get the angle of a module from 0-360 degrees
     * @return The angle of the module from 0-360 degrees
     */
    units::degree_t GetSteerAngle360();

    /**
     * @brief Get the current velocity of the drive motor
     * @return Angular velocity of drive motor in TPS (turns per second)
     */
    units::turns_per_second_t GetDriveVelocity();

    /**
     * @brief Get the applied output of the drive motor
     * @return Applied output voltage (-12V to +12V)
     */
    double GetDriveAppliedOutput();

    /**
     * @brief Get the current distance the drive motor has traveled
     * @return Distance drive motor travels in inches
     */
    units::inch_t GetDrivePosition();

    /**
     * @brief Returns the temperature of the drive motor
     * @return Temperature in Celsius
     */
    double GetDriveTemperature();

    /**
     * @brief Returns the temperature of the steer motor
     * @return Temperature in Celsius
     */
    double GetSteerTemperature();

    /**
     * @brief Get the current pose of a module
     * @return Pose with x/y offset and current angle of module
     */
    CustomPose GetPose();

    /**
     * @brief Check if drive motor is in coast mode
     * @return true if coast, false if brake
     */
    bool IsCoast();

    /**
     * @brief Set the absolute encoder offset for zeroing
     * @param _offset Offset in rotations (0.0 to 1.0)
     */
    void SetEncoderOffset(double _offset);

    /**
     * @brief Get the raw absolute encoder position (before offset)
     * @return Position in rotations (0.0 to 1.0)
     */
    double GetAbsoluteEncoderRaw();

    // Public variables
    CustomVector offset{0_in, 0_in};

   private:
    // Motor controllers (use 2025 API)
    rev::spark::SparkMax* DriveMotor;
    rev::spark::SparkMax* SteerMotor;

    // Configuration objects (created before motors are configured)
    rev::spark::SparkMaxConfig driveConfig;
    rev::spark::SparkMaxConfig steerConfig;

    // Configuration storage
    double absoluteEncoderOffset;
    bool isDriveInverted;
    bool isSteerInverted;
    bool isCoastMode;

    // Last commanded state for optimization
    units::degree_t lastCommandedAngle;

    /**
     * @brief Optimize the swerve module state to minimize rotation
     * @param _state Desired state
     * @return Optimized state
     */
    ModuleVelocity OptimizeState(ModuleVelocity _state);

    /**
     * @brief Configure drive motor with SparkMaxConfig
     */
    void ConfigureDriveMotor();

    /**
     * @brief Configure steer motor with SparkMaxConfig
     */
    void ConfigureSteerMotor();
};