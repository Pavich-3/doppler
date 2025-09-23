/**
 * @file i2s.c
 * @brief Implementation for I2S peripheral initialization.
 *
 * This file contains the function to configure and initialize multiple I2S peripherals
 * in a standardized way for audio input from microphones.
 */

#include "i2s.h" // Include the corresponding header file.

/**
 * @brief Initializes one or more I2S peripherals.
 * @details This function iterates through arrays of I2S handles and instances to configure them
 * with a common set of parameters for master receive mode (e.g., for microphones).
 *
 * @param i2sHandles   An array of pointers to I2S_HandleTypeDef structures.
 * @param i2sInstances An array of pointers to the I2S peripheral instances (e.g., SPI1, SPI4).
 * @param count        The number of I2S peripherals to initialize (the size of the arrays).
 * @return HAL_StatusTypeDef HAL_OK on success, or a HAL_ERROR status on failure.
 */
HAL_StatusTypeDef I2S_Init(I2S_HandleTypeDef* i2sHandles[], SPI_TypeDef* i2sInstances[], size_t count)
{
	// --- Input Validation ---
	// Perform a basic safety check to ensure the provided arrays are not null pointers.
	if (i2sHandles == NULL || i2sInstances == NULL) return HAL_ERROR;

	// --- Peripheral Configuration Loop ---
	// Iterate through each I2S peripheral that needs to be initialized.
	for (size_t i = 0; i < count; i++)
	{
		// Check if the specific handle and instance for this iteration are valid.
		if (i2sHandles[i] != NULL && i2sInstances[i] != NULL)
		{
			// Assign the hardware peripheral instance (e.g., SPI1) to the handle.
			i2sHandles[i]->Instance = i2sInstances[i];

			// --- I2S Configuration Parameters ---
			i2sHandles[i]->Init.Mode = I2S_MODE_MASTER_RX;       // Set as a Master device receiving data.
			i2sHandles[i]->Init.Standard = I2S_STANDARD_PHILIPS;  // Use the standard Philips I2S protocol.
			i2sHandles[i]->Init.DataFormat = I2S_DATAFORMAT_24B;  // Set data format to 24 bits per sample.
			i2sHandles[i]->Init.MCLKOutput = I2S_MCLKOUTPUT_DISABLE; // Disable the master clock output as it's not needed.
			i2sHandles[i]->Init.AudioFreq = I2S_AUDIOFREQ_32K;    // Set the target audio sampling frequency to 32 kHz.
			i2sHandles[i]->Init.CPOL = I2S_CPOL_LOW;              // Set the clock polarity to low when idle.
			i2sHandles[i]->Init.ClockSource = I2S_CLOCK_PLL;      // Use the dedicated PLLI2S as the clock source for accuracy.
			i2sHandles[i]->Init.FullDuplexMode = I2S_FULLDUPLEXMODE_DISABLE; // Disable full-duplex mode.

			// --- Initialize the Peripheral ---
			// Call the HAL function to apply the configuration. If it fails, call the global error handler.
			if (HAL_I2S_Init(i2sHandles[i]) != HAL_OK) Error_Handler();
		}
	}

	// Return HAL_OK to indicate that all peripherals were initialized successfully.
	return HAL_OK;
}
