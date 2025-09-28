// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Utils/SlewRateLimiter.h"

SlewRateLimiter::SlewRateLimiter(double _rateLimit, double _jerkLimit) {
    maxRateLimit = _rateLimit;
    jerkLimit = _jerkLimit;
    currRateLimit = 0.0;
    prevVal = 0.0;
    prevTime = 0.0_s;
    currTime = 0.0_s;
}

double SlewRateLimiter::Calculate(double _input) {
    double output = 0.0;
    currTime = frc::Timer::GetFPGATimestamp();

    elapsedTime = currTime - prevTime;

    if (_input > (prevVal + currRateLimit * elapsedTime.value())) {
        output = prevVal + currRateLimit * elapsedTime.value();
        currRateLimit = Calculations::LimitOutput(currRateLimit + jerkLimit * elapsedTime.value(), maxRateLimit);
    }
    else if (_input < (prevVal - currRateLimit * elapsedTime.value())) {
        output = prevVal - currRateLimit * elapsedTime.value();
        currRateLimit = Calculations::LimitOutput(currRateLimit + jerkLimit * elapsedTime(), maxRateLimit);
    }
    else {
        output = _input;
        currRateLimit = (prevVal - _input) * elapsedTime.value();
    }
    prevTime = currTime;
    prevVal = output;
    return output;
}
 
void SlewRateLimiter::Reset(double _input) {
    prevVal = _input;
    prevTime = frc::Timer::GetFPGATimestamp();
}
