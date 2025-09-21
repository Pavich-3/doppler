#ifndef INC_I2S_H_
#define INC_I2S_H_

#include "main.h"

HAL_StatusTypeDef I2S_Init(I2S_HandleTypeDef* i2sHandles[], SPI_TypeDef* i2sInstances[], size_t count);

#endif
