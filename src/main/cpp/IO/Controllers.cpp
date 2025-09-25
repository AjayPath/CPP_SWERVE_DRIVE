// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "IO/Controllers.h"

Controllers *Controllers::instance = NULL;

/**
 * @brief Constructor that creates Driver and Operator controllers
 * Creates CustomXbox instances for Driver (port 0) and Operator (port 1)
 */
Controllers::Controllers(void) {
    Driver = new CustomXbox(0);
    Operator = new CustomXbox(1);
}

/**
 * @brief Destructor for Controllers singleton
 */
Controllers::~Controllers(void) {}

/**
 * @brief Gets the single instance of Controllers (Singleton pattern)
 * Creates a new instance if one doesn't exist, otherwise returns existing instance
 * @return Pointer to the Controllers singleton instance
 */
Controllers *Controllers::GetInstance() {
    if (instance == NULL) {
        instance = new Controllers();
    }
    return instance;
}