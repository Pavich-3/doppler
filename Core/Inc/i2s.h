/**
 * @file i2s.h
 * @brief Header file for I2S peripheral initialization.
 *
 * This file contains the function prototype for initializing multiple I2S peripherals
 * using the STM32 HAL library.
 */

#ifndef INC_I2S_H_ // Include guard to prevent multiple inclusions of this header.
#define INC_I2S_H_

#include "main.h" // Include main header for HAL definitions and system-wide configurations.

/**
 * @brief Initializes one or more I2S peripherals.
 * @brief This function iterates through arrays of I2S handles and instances to configure them.
 *
 * @param i2sHandles   An array of pointers to I2S_HandleTypeDef structures.
 * @param i2sInstances An array of pointers to the I2S peripheral instances (e.g., SPI1, SPI4).
 * @param count        The number of I2S peripherals to initialize (the size of the arrays).
 * @return HAL_StatusTypeDef HAL_OK on success, or a HAL_ERROR status on failure.
 */
HAL_StatusTypeDef I2S_Init(I2S_HandleTypeDef* i2sHandles[], SPI_TypeDef* i2sInstances[], size_t count);

#endif /* INC_I2S_H_ */
