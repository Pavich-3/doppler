/**
 * @file timer.c
 * @brief Implementation for Timer (TIM) peripheral initialization.
 *
 * This file configures a hardware timer to generate PWM signals,
 * which are used to control the position of servo motors.
 */

#include "timer.h"

/**
 * @brief Initializes a single Timer peripheral in PWM mode for servo control.
 *
 * @param timerHandle A pointer to a TIM_HandleTypeDef structure that will be
 * configured by this function.
 * @return HAL_StatusTypeDef Returns HAL_OK if the initialization is successful.
 */
HAL_StatusTypeDef TIMER_Init(TIM_HandleTypeDef* timerHandle)
{
	// --- Base Timer Configuration ---
	timerHandle->Instance = TIM1; // Use the TIM1 peripheral.

	/*
	 * NOTE on PWM Frequency Calculation:
	 * The timer's clock frequency (after the APB2 prescaler) is divided by (Prescaler + 1)
	 * to get the counter clock frequency. The PWM period is then determined by (Period + 1).
	 *
	 * Assuming the APB2 timer clock is 50 MHz:
	 * Counter Clock = 50,000,000 / (249 + 1) = 200,000 Hz (0.2 MHz).
	 * Each tick of the counter takes 1 / 200,000 s = 5 microseconds.
	 * PWM Frequency = Counter Clock / (Period + 1) = 200,000 / (1999 + 1) = 100 Hz.
	 * PWM Period = 1 / 100 Hz = 10 ms (or 2000 ticks).
	 *
	 * CRITICAL ISSUE: The pulse values used in your project (750-2250) are larger
	 * than the Period (1999). This will cause incorrect behavior.
	 *
	 * SUGGESTED FIX for standard 50 Hz servo control (20ms period):
	 * - To make 1 tick = 1 microsecond: Prescaler = (50MHz / 1MHz) - 1 = 49.
	 * - To make the period 20ms (20,000 us): Period = 20000 - 1 = 19999.
	 * With these settings, a pulse value of 1500 would correctly generate a 1.5ms pulse.
	 */
	timerHandle->Init.Prescaler = 249;
	timerHandle->Init.Period = 1999;
	timerHandle->Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;   // No clock division.
	timerHandle->Init.CounterMode = TIM_COUNTERMODE_UP;         // Count up from 0 to Period.
	timerHandle->Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE; // Enable auto-reload preload for smooth updates.

	// Apply the base timer configuration. Call Error_Handler on failure.
	if (HAL_TIM_PWM_Init(timerHandle) != HAL_OK) Error_Handler();

	// --- PWM Channel Configuration ---
	// This structure defines the common settings for the PWM channels.
	TIM_OC_InitTypeDef sConfig = {
			.OCMode = TIM_OCMODE_PWM1,          // Use standard PWM1 mode.
			.Pulse = 0,                         // Initial pulse width is 0 (will be set later).
			.OCPolarity = TIM_OCPOLARITY_HIGH,    // The pulse is active high.
			.OCFastMode = TIM_OCFAST_ENABLE     // Enable fast mode.
	};

	// Configure Channel 2 (e.g., for Azimuth servo) with the settings above.
	if (HAL_TIM_PWM_ConfigChannel(timerHandle, &sConfig, TIM_CHANNEL_2) != HAL_OK) Error_Handler();
	// Configure Channel 3 (e.g., for Elevation servo) with the same settings.
	if (HAL_TIM_PWM_ConfigChannel(timerHandle, &sConfig, TIM_CHANNEL_3) != HAL_OK) Error_Handler();

	// --- Start PWM Signal Generation ---
	// Start generating the PWM signal on both configured channels.
	if (HAL_TIM_PWM_Start(timerHandle, TIM_CHANNEL_2) != HAL_OK) Error_Handler();
	if (HAL_TIM_PWM_Start(timerHandle, TIM_CHANNEL_3) != HAL_OK) Error_Handler();

	// Return HAL_OK to indicate successful initialization.
	return HAL_OK;
}
