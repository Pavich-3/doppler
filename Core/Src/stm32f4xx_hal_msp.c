/**
 * @file stm32f4xx_hal_msp.c
 * @brief MCU Support Package.
 *
 * This file provides the low-level hardware initialization functions (MSP callbacks)
 * that are called by the high-level HAL_Init functions. For example, HAL_I2S_Init()
 * calls HAL_I2S_MspInit() to configure the specific GPIOs, clocks, and DMA
 * required by that I2S peripheral.
 */

#include "main.h"

// --- Global DMA Handles ---
// These are declared globally because they are used by the I2S MSP functions
// and their corresponding interrupt handlers.
DMA_HandleTypeDef dmaI2S1Handle = {0};
DMA_HandleTypeDef dmaI2S4Handle = {0};

/**
  * @brief  Initializes the Global MSP.
  * This function is called from HAL_Init() and performs core-level initializations.
  */
void HAL_MspInit(void)
{
  // Enable the clock for the System Configuration Controller (SYSCFG).
  __HAL_RCC_SYSCFG_CLK_ENABLE();
  // Enable the clock for the Power Controller (PWR).
  __HAL_RCC_PWR_CLK_ENABLE();
}

/**
  * @brief  Initializes the MSP for the TIM PWM mode.
  * @param  htim: pointer to a TIM_HandleTypeDef structure.
  */
void HAL_TIM_PWM_MspInit(TIM_HandleTypeDef* htim)
{
	// Check if the timer instance is TIM1.
	if (htim->Instance == TIM1)
	{
		// Enable the clock for the TIM1 peripheral.
		__HAL_RCC_TIM1_CLK_ENABLE();
		// Enable the clock for GPIOA, which is used for TIM1's output pins.
		__HAL_RCC_GPIOA_CLK_ENABLE();

		// Configure GPIO pins for TIM1's channels.
		GPIO_InitTypeDef GPIO_InitStruct = {
				.Pin = GPIO_PIN_10 | GPIO_PIN_9,      // Pins PA9 and PA10.
				.Mode = GPIO_MODE_AF_PP,            // Mode: Alternate Function, Push-Pull.
				.Pull = GPIO_NOPULL,                // No pull-up or pull-down resistor.
				.Speed = GPIO_SPEED_FREQ_LOW,       // Low speed is sufficient for servos.
				.Alternate = GPIO_AF1_TIM1          // Connect these pins to TIM1.
		};
		HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
	}
}

/**
  * @brief  Configures a DMA handle with common settings for I2S reception.
  * @param  dmaHandle: pointer to the DMA_HandleTypeDef to configure.
  */
static void Configure_I2S_DMA(DMA_HandleTypeDef* dmaHandle)
{
	dmaHandle->Init.Direction = DMA_PERIPH_TO_MEMORY;       // Data flows from the peripheral (I2S) to memory.
	dmaHandle->Init.PeriphInc = DMA_PINC_DISABLE;         // Do not increment the peripheral address.
	dmaHandle->Init.MemInc = DMA_MINC_ENABLE;             // Increment the memory address after each transfer.
	dmaHandle->Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD; // Peripheral data size is a 32-bit word.
	dmaHandle->Init.MemDataAlignment = DMA_MDATAALIGN_WORD;    // Memory data size is a 32-bit word.
	dmaHandle->Init.Mode = DMA_CIRCULAR;                  // Use circular mode for continuous audio streaming.
	dmaHandle->Init.Priority = DMA_PRIORITY_LOW;          // Set DMA priority to low.
	dmaHandle->Init.FIFOMode = DMA_FIFOMODE_ENABLE;       // Enable the DMA FIFO.
	dmaHandle->Init.FIFOThreshold = DMA_FIFO_THRESHOLD_HALFFULL; // Set FIFO threshold.
	dmaHandle->Init.MemBurst = DMA_MBURST_INC4;           // Use a burst of 4 increments for memory.
	dmaHandle->Init.PeriphBurst = DMA_PBURST_INC4;        // Use a burst of 4 increments for the peripheral.
}

/**
  * @brief  Initializes the MSP for the I2S peripheral.
  * @param  hi2s: pointer to a I2S_HandleTypeDef structure.
  */
void HAL_I2S_MspInit(I2S_HandleTypeDef* hi2s)
{
	// Common GPIO configuration for I2S pins.
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;

	// Enable clocks for GPIOA (used by both) and the DMA2 controller.
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_DMA2_CLK_ENABLE();

	// --- Configuration for I2S1 (SPI1) ---
	if (hi2s->Instance == SPI1)
	{
		// Enable the clock for the SPI1 peripheral.
		__HAL_RCC_SPI1_CLK_ENABLE();

		// Configure GPIO pins for I2S1 (WS, CK, SD).
		GPIO_InitStruct.Pin = GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_7;
		GPIO_InitStruct.Alternate = GPIO_AF5_SPI1; // Connect pins to SPI1/I2S1.
		HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

		// Configure the DMA stream for I2S1 Rx.
		dmaI2S1Handle.Instance = DMA2_Stream2;
		dmaI2S1Handle.Init.Channel = DMA_CHANNEL_3;
		Configure_I2S_DMA(&dmaI2S1Handle);
		if (HAL_DMA_Init(&dmaI2S1Handle) != HAL_OK) Error_Handler();

		// Link the I2S handle with the DMA handle for reception.
		__HAL_LINKDMA(hi2s, hdmarx, dmaI2S1Handle);

		// Configure and enable the DMA interrupt in the NVIC.
		HAL_NVIC_SetPriority(DMA2_Stream2_IRQn, 5, 0);
		HAL_NVIC_EnableIRQ(DMA2_Stream2_IRQn);
	}
	// --- Configuration for I2S4 (SPI4) ---
	else if (hi2s->Instance == SPI4)
	{
		// Enable clocks for the SPI4 peripheral and GPIOB.
		__HAL_RCC_SPI4_CLK_ENABLE();
		__HAL_RCC_GPIOB_CLK_ENABLE();

		// Configure GPIO pins for I2S4. Note that they are on different ports.
		GPIO_InitStruct.Pin = GPIO_PIN_1; // I2S4_SD on PA1
		GPIO_InitStruct.Alternate = GPIO_AF6_SPI4;
		HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

		GPIO_InitStruct.Pin = GPIO_PIN_12 | GPIO_PIN_13; // I2S4_WS on PB12, I2S4_CK on PB13
		GPIO_InitStruct.Alternate = GPIO_AF6_SPI4;
		HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

		// Configure the DMA stream for I2S4 Rx.
		dmaI2S4Handle.Instance = DMA2_Stream0;
		dmaI2S4Handle.Init.Channel = DMA_CHANNEL_4;
		Configure_I2S_DMA(&dmaI2S4Handle);
		if (HAL_DMA_Init(&dmaI2S4Handle) != HAL_OK) Error_Handler();

		// Link the I2S handle with the DMA handle for reception.
		__HAL_LINKDMA(hi2s, hdmarx, dmaI2S4Handle);

		// Configure and enable the DMA interrupt in the NVIC.
		HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 5, 0);
		HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);
	}
}

/**
  * @brief  De-Initializes the MSP for the I2S peripheral.
  * @param  hi2s: pointer to a I2S_HandleTypeDef structure.
  */
void HAL_I2S_MspDeInit(I2S_HandleTypeDef* hi2s)
{
  // NOTE: This function appears to be configured for SPI2, not SPI1 or SPI4 used in this project.
  if(hi2s->Instance==SPI2)
  {
    // Disable the peripheral clock.
    __HAL_RCC_SPI2_CLK_DISABLE();

    // De-initialize the GPIO pins.
    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_10|GPIO_PIN_12|GPIO_PIN_14|GPIO_PIN_15);

    // De-initialize the DMA stream.
    HAL_DMA_DeInit(hi2s->hdmarx);

    // Disable the interrupt.
    HAL_NVIC_DisableIRQ(SPI2_IRQn);
  }
}

/**
  * @brief  De-Initializes the MSP for the TIM PWM mode.
  * @param  htim_pwm: pointer to a TIM_HandleTypeDef structure.
  */
void HAL_TIM_PWM_MspDeInit(TIM_HandleTypeDef* htim_pwm)
{
  if(htim_pwm->Instance==TIM1)
  {
    // Disable the TIM1 peripheral clock.
    __HAL_RCC_TIM1_CLK_DISABLE();
  }
}
