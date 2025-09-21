#include "main.h"

DMA_HandleTypeDef dmaI2S1Handle = {0};
DMA_HandleTypeDef dmaI2S4Handle = {0};

void HAL_MspInit(void)
{
  __HAL_RCC_SYSCFG_CLK_ENABLE();
  __HAL_RCC_PWR_CLK_ENABLE();
}

void HAL_TIM_PWM_MspInit(TIM_HandleTypeDef* htim)
{
	if (htim->Instance == TIM1)
	{
		__HAL_RCC_TIM1_CLK_ENABLE();
		__HAL_RCC_GPIOA_CLK_ENABLE();

		GPIO_InitTypeDef GPIO_InitSturct = {
				.Pin = GPIO_PIN_10 | GPIO_PIN_9,
				.Mode = GPIO_MODE_AF_PP,
				.Pull = GPIO_NOPULL,
				.Speed = GPIO_SPEED_FREQ_LOW,
				.Alternate = GPIO_AF1_TIM1
		};
		HAL_GPIO_Init(GPIOA, &GPIO_InitSturct);
	}
}

static void Configure_I2S_DMA(DMA_HandleTypeDef* dmaHandle)
{
	dmaHandle->Init.Direction = DMA_PERIPH_TO_MEMORY;
	dmaHandle->Init.PeriphInc = DMA_PINC_DISABLE;
	dmaHandle->Init.MemInc = DMA_MINC_ENABLE;
	dmaHandle->Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;
	dmaHandle->Init.MemDataAlignment = DMA_MDATAALIGN_WORD;
	dmaHandle->Init.Mode = DMA_CIRCULAR;
	dmaHandle->Init.Priority = DMA_PRIORITY_LOW;
	dmaHandle->Init.FIFOMode = DMA_FIFOMODE_ENABLE;
	dmaHandle->Init.FIFOThreshold = DMA_FIFO_THRESHOLD_HALFFULL;
	dmaHandle->Init.MemBurst = DMA_MBURST_INC4;
	dmaHandle->Init.PeriphBurst = DMA_PBURST_INC4;
}

void HAL_I2S_MspInit(I2S_HandleTypeDef* hi2s)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;


	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_DMA2_CLK_ENABLE();

	if (hi2s->Instance == SPI1)
	{
		__HAL_RCC_SPI1_CLK_ENABLE();

		GPIO_InitStruct.Pin = GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_7;
		GPIO_InitStruct.Alternate = GPIO_AF5_SPI1;
		HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

		dmaI2S1Handle.Instance = DMA2_Stream2;
		dmaI2S1Handle.Init.Channel = DMA_CHANNEL_3;
		Configure_I2S_DMA(&dmaI2S1Handle);
		if (HAL_DMA_Init(&dmaI2S1Handle) != HAL_OK) Error_Handler();
		__HAL_LINKDMA(hi2s, hdmarx, dmaI2S1Handle);

		HAL_NVIC_SetPriority(DMA2_Stream2_IRQn, 5, 0);
		HAL_NVIC_EnableIRQ(DMA2_Stream2_IRQn);
	}

	else if (hi2s->Instance == SPI4)
	{
		__HAL_RCC_SPI4_CLK_ENABLE();
		__HAL_RCC_GPIOB_CLK_ENABLE();


		GPIO_InitStruct.Pin = GPIO_PIN_1;
		GPIO_InitStruct.Alternate = GPIO_AF6_SPI4;
		HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

		GPIO_InitStruct.Pin = GPIO_PIN_12 | GPIO_PIN_13;
		GPIO_InitStruct.Alternate = GPIO_AF6_SPI4;
		HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

		dmaI2S4Handle.Instance = DMA2_Stream0;
		dmaI2S4Handle.Init.Channel = DMA_CHANNEL_4;
		Configure_I2S_DMA(&dmaI2S4Handle);
		if (HAL_DMA_Init(&dmaI2S4Handle) != HAL_OK) Error_Handler();
		__HAL_LINKDMA(hi2s, hdmarx, dmaI2S4Handle);

		HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 5, 0);
		HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);
	}
}

void HAL_I2S_MspDeInit(I2S_HandleTypeDef* hi2s)
{
  if(hi2s->Instance==SPI2)
  {
    __HAL_RCC_SPI2_CLK_DISABLE();

    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_10|GPIO_PIN_12|GPIO_PIN_14|GPIO_PIN_15);

    HAL_DMA_DeInit(hi2s->hdmarx);

    HAL_NVIC_DisableIRQ(SPI2_IRQn);
  }
}


void HAL_TIM_PWM_MspDeInit(TIM_HandleTypeDef* htim_pwm)
{
  if(htim_pwm->Instance==TIM1)
  {
    __HAL_RCC_TIM1_CLK_DISABLE();
  }
}
