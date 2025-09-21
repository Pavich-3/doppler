#include "main.h"
#include "stdbool.h"

#define AUDIO_BUFFER_SIZE 2048

int32_t i2s1_rx_buffer[AUDIO_BUFFER_SIZE * 2];
int32_t i2s4_rx_buffer[AUDIO_BUFFER_SIZE * 2];

volatile int32_t* i2s1_data_ptr = NULL;
volatile int32_t* i2s4_data_ptr = NULL;
volatile bool i2s1_data_ready = false;
volatile bool i2s4_data_ready = false;

TIM_HandleTypeDef timerHandle = {0};
I2S_HandleTypeDef i2s1Handle = {0};
I2S_HandleTypeDef i2s4Handle = {0};

I2S_HandleTypeDef* i2sHandles[2] = {&i2s1Handle, &i2s4Handle};
SPI_TypeDef* i2sInstances[2] = {SPI1, SPI4};

void SystemClock_Config(void);

int main(void)
{
  HAL_Init();
  SystemClock_Config();
  TIMER_Init(&timerHandle);
  I2S_Init(i2sHandles, i2sInstances, 2);
  HAL_I2S_Receive_DMA(&i2s1Handle, (uint16_t*)i2s1_rx_buffer, AUDIO_BUFFER_SIZE * 4);
  HAL_I2S_Receive_DMA(&i2s4Handle, (uint16_t*)i2s4_rx_buffer, AUDIO_BUFFER_SIZE * 4);

  while (1)
  {

  }
}

void HAL_I2S_RxHalfCpltCallback(I2S_HandleTypeDef *hi2s)
{
  if (hi2s->Instance == SPI1)
  {
    i2s1_data_ptr = &i2s1_rx_buffer[0];
    i2s1_data_ready = true;
  }
  else if (hi2s->Instance == SPI4)
  {
    i2s4_data_ptr = &i2s4_rx_buffer[0];
    i2s4_data_ready = true;
  }
}

void HAL_I2S_RxCpltCallback(I2S_HandleTypeDef *hi2s)
{
  if (hi2s->Instance == SPI1)
  {
    i2s1_data_ptr = &i2s1_rx_buffer[AUDIO_BUFFER_SIZE];
    i2s1_data_ready = true;
  }
  else if (hi2s->Instance == SPI4)
  {
    i2s4_data_ptr = &i2s4_rx_buffer[AUDIO_BUFFER_SIZE];
    i2s4_data_ready = true;
  }
}

void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  __HAL_RCC_PLL_PLLM_CONFIG(16);

  __HAL_RCC_PLL_PLLSOURCE_CONFIG(RCC_PLLSOURCE_HSE);

  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) Error_Handler();

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSE;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK) Error_Handler();

  HAL_RCC_EnableCSS();

  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_I2S;
  PeriphClkInitStruct.PLLI2S.PLLI2SN = 192;
  PeriphClkInitStruct.PLLI2S.PLLI2SM = 16;
  PeriphClkInitStruct.PLLI2S.PLLI2SR = 6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK) Error_Handler();
}


void Error_Handler(void)
{
  __disable_irq();
  while (1)
  {
  }
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{

}
#endif
