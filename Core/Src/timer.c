#include "timer.h"

HAL_StatusTypeDef TIMER_Init(TIM_HandleTypeDef* timerHandle)
{
	timerHandle->Instance = TIM1;
	timerHandle->Init.Prescaler = 249;
	timerHandle->Init.Period = 1999;
	timerHandle->Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	timerHandle->Init.CounterMode = TIM_COUNTERMODE_UP;
	timerHandle->Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
	if (HAL_TIM_PWM_Init(timerHandle) != HAL_OK) Error_Handler();

	TIM_OC_InitTypeDef sConfig = {
			.OCMode = TIM_OCMODE_PWM1,
			.Pulse = 0,
			.OCPolarity = TIM_OCPOLARITY_HIGH,
			.OCFastMode = TIM_OCFAST_ENABLE
	};
	if (HAL_TIM_PWM_ConfigChannel(timerHandle, &sConfig, TIM_CHANNEL_2) != HAL_OK) Error_Handler();
	if (HAL_TIM_PWM_ConfigChannel(timerHandle, &sConfig, TIM_CHANNEL_3) != HAL_OK) Error_Handler();

	if (HAL_TIM_PWM_Start(timerHandle, TIM_CHANNEL_2) != HAL_OK) Error_Handler();
	if (HAL_TIM_PWM_Start(timerHandle, TIM_CHANNEL_3) != HAL_OK) Error_Handler();

	return HAL_OK;
}
