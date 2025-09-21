#include <i2s.h>

HAL_StatusTypeDef I2S_Init(I2S_HandleTypeDef* i2sHandles[], SPI_TypeDef* i2sInstances[], size_t count)
{
	if (i2sHandles == NULL || i2sInstances == NULL)
		return HAL_ERROR;

	for(size_t i = 0; i < count; i++)
	{
		if (i2sHandles[i] != NULL && i2sInstances[i] != NULL)
		{
			i2sHandles[i]->Instance = i2sInstances[i];
			i2sHandles[i]->Init.Mode = I2S_MODE_MASTER_RX;
			i2sHandles[i]->Init.Standard = I2S_STANDARD_PHILIPS;
			i2sHandles[i]->Init.DataFormat = I2S_DATAFORMAT_24B;
			i2sHandles[i]->Init.MCLKOutput = I2S_MCLKOUTPUT_DISABLE;
			i2sHandles[i]->Init.AudioFreq = I2S_AUDIOFREQ_32K;
			i2sHandles[i]->Init.CPOL = I2S_CPOL_LOW;
			i2sHandles[i]->Init.ClockSource = I2S_CLOCK_PLL;
			i2sHandles[i]->Init.FullDuplexMode = I2S_FULLDUPLEXMODE_DISABLE;

			if (HAL_I2S_Init(i2sHandles[i]) != HAL_OK) Error_Handler();
		}
	}

	return HAL_OK;
}
