/**
 * @file main.h
 * @brief Main header file for the project.
 *
 * This file includes all the necessary headers for the project and provides
 * the main error handler function prototype. It acts as a central point
 * for including project-wide dependencies.
 */

// Standard include guard to prevent multiple inclusions of this file.
#ifndef __MAIN_H
#define __MAIN_H

// This block ensures that the C code can be correctly linked when compiled
// with a C++ compiler.
#ifdef __cplusplus
extern "C" {
#endif

// --- Includes ---

// Main header for the STM32F4 series HAL (Hardware Abstraction Layer).
// This provides access to all the standard peripheral drivers.
#include "stm32f4xx_hal.h"

// Custom header for TIM (Timer) peripheral initialization, likely for PWM servo control.
#include "timer.h"

// Custom header for I2S (Inter-IC Sound) peripheral initialization for the microphones.
#include "i2s.h"

// Header for the main application logic, the drone tracker.
#include "drone_tracker.h"

// --- Function Prototypes ---

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 *
 * It's a global error handler, typically called by HAL functions on failure.
 * The implementation usually consists of an infinite loop to halt the system
 * and prevent further execution after a critical error.
 */
void Error_Handler(void);

// Closes the extern "C" block for C++ compatibility.
#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
