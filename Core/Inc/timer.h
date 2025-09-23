/**
 * @file timer.h
 * @brief Header file for Timer (TIM) peripheral initialization.
 *
 * This file provides the function prototype for configuring a hardware timer,
 * which is likely used for generating PWM signals to control servo motors.
 */

// Include guard to prevent this header from being included multiple times
// in a single compilation unit.
#ifndef INC_TIMER_H_
#define INC_TIMER_H_

// Include the main project header to get access to the STM32 HAL library definitions.
#include "main.h"

/**
 * @brief Initializes a single Timer peripheral in PWM mode.
 *
 * @param timerHandle A pointer to a TIM_HandleTypeDef structure that will be
 * configured by this function.
 * @return HAL_StatusTypeDef Returns HAL_OK if the initialization is successful,
 * otherwise returns a HAL error code.
 */
HAL_StatusTypeDef TIMER_Init(TIM_HandleTypeDef* timerHandle);

#endif /* INC_TIMER_H_ */
