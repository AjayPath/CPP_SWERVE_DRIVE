// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once
#include "frc/Timer.h"

class SimPID {
 public:
  
  SimPID(float p = 0.0, float i = 0.0, float d = 0.0, float epislon = 0.0);

  void setConstants(float p, float i, float d);
  void setIzone(float zone);
  void setErrorEpsilon(float epislon);
  void setErrorIncrement(float inc);
  void setDesiredValue(float val);
  void setMaxOutput(float max);
  void resetErrorSum(void);
  float calcPID(float current);
  bool isDone(void);
  void setMinDoneCycles(int n);

 private:

  float m_p;
  float m_i;
  float m_d;
  float m_izone;
  float m_desiredValue;
  float m_oldDesiredValue;
  double m_previousValue;
  float m_previousError;
  float m_errorSum;
  float m_errorIncrement;
  float m_errorEpsilon;
  bool m_firstCycle;
  float m_maxOutput;
  int m_minCycleCount;
  int m_cycleCount;

  frc::Timer *pidTimer;

};
