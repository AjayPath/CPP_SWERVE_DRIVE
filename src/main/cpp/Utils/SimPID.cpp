// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Utils/SimPID.h"

/**
 * @brief Constructor that initializes the PID controller with given constants
 * @param p Proportional gain constant
 * @param i Integral gain constant  
 * @param d Derivative gain constant
 * @param epislon Error tolerance for determining when target is reached
 */
SimPID::SimPID(float p, float i, float d, float epislon) {
    m_p = p;
    m_i = i;
    m_d = d;
    m_errorEpsilon = epislon;
    m_desiredValue = 0;
    m_oldDesiredValue = 0;
    m_previousValue = 0;
    m_firstCycle = true;
    m_maxOutput = 1.0;
    m_cycleCount = 0;
    m_minCycleCount = 10;
    m_izone = 0;

    pidTimer = new frc::Timer();
    pidTimer->Start();
    pidTimer->Reset();
}

/**
 * @brief Updates the PID constants (P, I, D gains)
 * @param p New proportional gain
 * @param i New integral gain
 * @param d New derivative gain
 */
void SimPID::setConstants(float p, float i, float d) {
    m_p = p;
    m_i = i;
    m_d = d;
} 

/**
 * @brief Sets the integral zone - range where integral accumulation is allowed
 * @param zone Maximum error magnitude for integral accumulation (0 = no limit)
 */
void SimPID::setIzone(float zone) {
    m_izone = zone;
}

/**
 * @brief Sets the error tolerance for determining when target is reached
 * @param epsilon Error tolerance value
 */
void SimPID::setErrorEpsilon(float epsilon) {
    m_errorEpsilon = epsilon;
}

/**
 * @brief Sets the maximum increment for error accumulation
 * @param inc Maximum error increment per cycle
 */
void SimPID::setErrorIncrement(float inc) {
    m_errorIncrement = inc;
}

/**
 * @brief Sets the target setpoint value
 * @param val Desired target value
 */
void SimPID::setDesiredValue(float val) {
    m_desiredValue = val;
}

/**
 * @brief Sets the maximum output limit (must be between 0.0 and 1.0)
 * @param max Maximum absolute output value
 */
void SimPID::setMaxOutput(float max) {
    if (max >= 0.0 && max <= 1.0) {
        m_maxOutput = max;
    }
}

/**
 * @brief Resets the accumulated integral error to zero
 */
void SimPID::resetErrorSum(void) {
    m_errorSum = 0;
}

/**
 * @brief Sets minimum number of cycles within tolerance before reporting done
 * @param n Number of consecutive cycles required
 */
void SimPID::setMinDoneCycles(int n) {
    m_minCycleCount = n;
}

/**
 * @brief Calculates PID output based on current sensor value
 * Uses derivative of process variable to avoid derivative kick on setpoint changes
 * @param currentValue Current sensor reading/process variable
 * @return PID controller output (clamped to maxOutput range)
 */
float SimPID::calcPID(float currentValue) {

    // Initialize P, I, D components
    float pVal = 0.0;
    float iVal = 0.0;
    float dVal = 0.0;

    // First cycle initialization - don't calculate derivative
    if (m_firstCycle) {
        m_previousValue = currentValue;
        m_previousError = m_desiredValue - currentValue;
        m_firstCycle = false;
        pidTimer->Reset();
    }

    // Reset states when setpoint changes
    if (m_oldDesiredValue != m_desiredValue) {
        m_firstCycle = true;
        m_errorSum = 0.0;
    }

    // Calculate Proportional term: P = Kp * error
    float error = m_desiredValue - currentValue;
    pVal = m_p * error;

    // Calculate Integral term: accumulate error over time
    if (fabs(error) > m_errorEpsilon) {
        // Only accumulate if within I-zone (or no I-zone set)
        if (m_izone == 0.0 || fabs(error) <= m_izone) {
            m_errorSum += error;
        }
    }
    else {
        // Reset integral when within error tolerance
        m_errorSum = 0.0;
    }

    iVal = m_i * m_errorSum;

    // Calculate Derivative term: based on rate of change of process variable
    double dt = (double)pidTimer->Get();
    if (dt > 0.0 && !m_firstCycle) {
        float velocity = (currentValue - m_previousValue) / dt;
        dVal = m_d * velocity;
    } 
    else {
        dVal = 0.0;
    }

    // Combine all terms (subtract D to dampen motion toward setpoint)
    float output = pVal + iVal - dVal;

    // Clamp output to maximum limits
    if (output > m_maxOutput) {
        output = m_maxOutput;
    }
    else if (output < -m_maxOutput) {
        output = -m_maxOutput;
    }

    // Store values for next cycle
    m_previousValue = currentValue;
    m_previousError = error;
    pidTimer->Reset();
    m_oldDesiredValue = m_desiredValue;

    return output;
}

/**
 * @brief Checks if the controller has reached and stabilized at the target
 * Requires the system to stay within error tolerance for minimum cycle count
 * @return True if target reached and stable, false otherwise
 */
bool SimPID::isDone(void) {
    // Check if current value is within tolerance band
    if (m_previousValue <= m_desiredValue + m_errorEpsilon
            && m_previousValue >= m_desiredValue - m_errorEpsilon
                && !m_firstCycle) {

                    // Count consecutive cycles within tolerance
                    if (m_cycleCount >= m_minCycleCount) {
                        m_cycleCount = 0;
                        return true;
                    }
                    else {
                        m_cycleCount++;
                    }

                }
                return false;
}