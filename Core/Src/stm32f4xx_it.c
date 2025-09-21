#include "main.h"
#include "stm32f4xx_it.h"

extern DMA_HandleTypeDef dmaI2S1Handle;
extern DMA_HandleTypeDef dmaI2S4Handle;

void DMA2_Stream2_IRQHandler(void)
{
	HAL_DMA_IRQHandler(&dmaI2S1Handle);
}

void DMA2_Stream0_IRQHandler(void)
{
	HAL_DMA_IRQHandler(&dmaI2S4Handle);
}

void NMI_Handler(void)
{
  HAL_RCC_NMI_IRQHandler();
   while (1)
  {
  }
}

void HardFault_Handler(void)
{
  while (1)
  {

  }
}

void MemManage_Handler(void)
{
  while (1)
  {

  }
}

void BusFault_Handler(void)
{
  while (1)
  {

  }
}

void UsageFault_Handler(void)
{
  while (1)
  {

  }
}

void SVC_Handler(void)
{

}

void DebugMon_Handler(void)
{

}

void PendSV_Handler(void)
{

}

void SysTick_Handler(void)
{
  HAL_IncTick();
}
