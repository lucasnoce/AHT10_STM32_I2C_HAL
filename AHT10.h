/*
 * AHT10.h
 *
 *  Created on: Nov 23, 2023
 *      Author: LucasNoce
 */

#ifndef INC_AHT10_H_
#define INC_AHT10_H_

#include "stm32f1xx_hal.h"  // Needed for I2C


/* Defines ------------------------------------------------------------*/
#define AHT10_I2C_ADDR				(0x38 << 1)  // 0b0111_000x -> x = r/w (0 = write, 1 = read)

#define AHT10_I2C_READ				0x1
#define AHT10_I2C_WRITE				0x0

#define AHT10_I2C_CMD_INIT			0xE1  // 0b1110_0001 - Initialization
#define AHT10_I2C_CMD_START_MEAS	0xAC  // 0b1010_1100 - Start Measurement -> Wait 75ms and check status bit 7 (0 = ready, 1 = busy)
#define AHT10_I2C_CMD_DATA_MEAS		0x33  // 0b0011_0011 - Parameter to Start Measurement
#define AHT10_I2C_CMD_SRST			0xBA  // 0b1011_1010 - Soft Reset
#define AHT10_I2C_CMD_NORMAL		0xAB  // 0b1010_1011 - Normal Cycle Mode
#define AHT10_I2C_CMD_DATA_NOP		0x00  // 0b0000_0000 - Parameter to Start Measurement and Set Mode

#define AHT10_MODE_NORMAL			0x00  // Enable Normal Mode
#define AHT10_MODE_CYCLE			0x20  // Enable Cycle Mode
#define AHT10_ENABLE_CAL			0x08  // Load Factory Calibration coeff

#define AHT10_DELAY_MEASURMENT		80    // At least 75ms
#define AHT10_DELAY_POWER_ON		25    // At least 20ms
#define AHT10_DELAY_SOFT_RESET		20    // Less than 20ms




/* Sensor Struct ------------------------------------------------------------*/
typedef struct{
	I2C_HandleTypeDef *i2cHandle;	// I2C handle
	uint8_t status;					// Status Byte
	uint8_t opMode;					// Operation Mode
	float temp_C;					// Temperature data (*C)
	float humid_100;				// Humidity data (%)
} AHT10;


/* Initialization ------------------------------------------------------------*/
HAL_StatusTypeDef AHT10_Init(AHT10 *dev, I2C_HandleTypeDef *i2cHandle);


/* Data Acquisition ------------------------------------------------------------*/
HAL_StatusTypeDef AHT10_RequestMeasurement(AHT10 *dev);
HAL_StatusTypeDef AHT10_ReadTempHumid(AHT10 *dev);
HAL_StatusTypeDef AHT10_RequestAndReadTempHumid(AHT10 *dev);
HAL_StatusTypeDef AHT10_GetStatus(AHT10 *dev);


/* Operation ------------------------------------------------------------*/
HAL_StatusTypeDef AHT10_SoftReset(AHT10 *dev);
HAL_StatusTypeDef AHT10_SetMode(AHT10 *dev, uint8_t mode);


#endif /* INC_AHT10_H_ */
