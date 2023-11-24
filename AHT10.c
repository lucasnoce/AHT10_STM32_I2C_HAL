/*
 * AHT10.c
 *
 *  Created on: Nov 23, 2023
 *      Author: lucas
 */

#include "AHT10.h"



/* Initialization ------------------------------------------------------------*/

/**
  * @brief	Initializes the sensor.
  * @param	dev Pointer to a AHT10 structure that contains the configuration
  *             information for the specified AHT10.
  * @param	i2cHandle Pointer to a I2C_HandleTypeDef structure that contains
  *                the configuration information for the specified I2C.
  * @retval HAL status.
  */
HAL_StatusTypeDef AHT10_Init(AHT10 *dev, I2C_HandleTypeDef *i2cHandle){
	dev->status = 0;
	dev->opMode = AHT10_MODE_NORMAL;
	dev->temp_C = 0.0f;
	dev->humid_100 = 0.0f;
	dev->i2cHandle = i2cHandle;

	HAL_StatusTypeDef retStatus = AHT10_GetStatus(dev);

	if ((dev->status & 0x08) == 0){
		uint8_t AHT10_i2c_tx[3];

		AHT10_i2c_tx[0] = AHT10_I2C_CMD_INIT;
		AHT10_i2c_tx[1] = AHT10_ENABLE_CAL;
		AHT10_i2c_tx[2] = AHT10_MODE_NORMAL;

		retStatus = HAL_I2C_Master_Transmit(dev->i2cHandle, AHT10_I2C_ADDR, (uint8_t *)AHT10_i2c_tx, 3, 100);
	}

	return retStatus;
}



/* Data Acquisition ------------------------------------------------------------*/

/**
  * @brief	Send command and parameters to request a new measurement.
  * @note	Measurement values are ready after 75ms.
  * @param	dev Pointer to a AHT10 structure that contains the configuration
  *             information for the specified AHT10.
  * @retval HAL status.
  */
HAL_StatusTypeDef AHT10_RequestMeasurement(AHT10 *dev){
	uint8_t AHT10_i2c_tx[3];

	AHT10_i2c_tx[0] = AHT10_I2C_CMD_START_MEAS;
	AHT10_i2c_tx[1] = AHT10_I2C_CMD_DATA_MEAS;
	AHT10_i2c_tx[2] = AHT10_I2C_CMD_DATA_NOP;

	return HAL_I2C_Master_Transmit(dev->i2cHandle, AHT10_I2C_ADDR, (uint8_t *)AHT10_i2c_tx, 3, 100);
}

/**
  * @brief	Read the measurement data.
  * @note	Must have previously sent a measurement request.
  * @param	dev Pointer to a AHT10 structure that contains the configuration
  *             information for the specified AHT10.
  * @retval HAL status.
  */
HAL_StatusTypeDef AHT10_ReadTempHumid(AHT10 *dev){
	uint8_t AHT10_i2c_rawData[6];
	uint32_t AHT10_i2c_rx;

	HAL_StatusTypeDef retStatus = HAL_I2C_Master_Receive(dev->i2cHandle, (AHT10_I2C_ADDR | 0x01), AHT10_i2c_rawData, 6, 100);

	if (retStatus == HAL_OK){
		dev->status = AHT10_i2c_rawData[0];

		AHT10_i2c_rx = ((uint32_t)AHT10_i2c_rawData[1] << 12) | ((uint32_t)AHT10_i2c_rawData[2] << 4) | (AHT10_i2c_rawData[3] >> 4);
		dev->humid_100 = (float) (AHT10_i2c_rx / 1048576.0) * 100.0;

		AHT10_i2c_rx = (((uint32_t)AHT10_i2c_rawData[3] & 0xF) << 16) | ((uint32_t)AHT10_i2c_rawData[4] << 8) | AHT10_i2c_rawData[5];;
		dev->temp_C = (float) ((AHT10_i2c_rx / 1048576.0) * 200.0) - 50.0;
	}

	return retStatus;
}

/**
  * @brief	Send command and parameters to request a new measurement, wait
  * 		75ms and read the measurement data.
  * @param	dev Pointer to a AHT10 structure that contains the configuration
  *             information for the specified AHT10.
  * @retval HAL status.
  */
HAL_StatusTypeDef AHT10_RequestAndReadTempHumid(AHT10 *dev){
	HAL_StatusTypeDef retStatus = AHT10_RequestMeasurement(dev);

	if (retStatus != HAL_OK){
		return retStatus;
	}

	HAL_Delay(AHT10_DELAY_MEASURMENT);

	return AHT10_ReadTempHumid(dev);
}

/**
  * @brief	Get sensor status byte.
  * @note	Bit[7]:   Busy (1) or not busy (0).
  *			Bit[6:5]: Operation Mode (00 = Normal, 01 = Cycle, 1x = CMD).
  *			Bit[4]:   Reserved.
  *			Bit[3]:   Calibration enabled (1) or disabled (0).
  *			Bit[2:0]: Reserved.
  * @param	dev Pointer to a AHT10 structure that contains the configuration
  *             information for the specified AHT10.
  * @retval HAL status.
  */
HAL_StatusTypeDef AHT10_GetStatus(AHT10 *dev){
	uint8_t AHT10_i2c_rawData[6];

	HAL_StatusTypeDef retStatus = HAL_I2C_Master_Receive(dev->i2cHandle, (AHT10_I2C_ADDR | 0x01), AHT10_i2c_rawData, 6, 100);

	dev->status = AHT10_i2c_rawData[0];

	return retStatus;
}



/* Operation ------------------------------------------------------------*/

/**
  * @brief	Send soft reset command.
  * @param	dev Pointer to a AHT10 structure that contains the configuration
  *             information for the specified AHT10.
  * @retval HAL status.
  */
HAL_StatusTypeDef AHT10_SoftReset(AHT10 *dev){
	uint8_t AHT10_i2c_tx = AHT10_I2C_CMD_SRST;
	return HAL_I2C_Master_Transmit(dev->i2cHandle, AHT10_I2C_ADDR, &AHT10_i2c_tx, 1, 100);
}

/**
  * @brief	Change Operation Mode.
  * @param	dev Pointer to a AHT10 structure that contains the configuration
  *             information for the specified AHT10.
  * @param	mode Desired Operation Mode (00 = Normal, 01 = Cycle, 1x = CMD).
  * @retval HAL status.
  */
HAL_StatusTypeDef AHT10_SetMode(AHT10 *dev, uint8_t mode){
	dev->opMode = mode;

	uint8_t AHT10_i2c_tx[3];

	if (mode == 0){
		AHT10_i2c_tx[0] = AHT10_I2C_CMD_NORMAL;
		AHT10_i2c_tx[1] = AHT10_I2C_CMD_DATA_NOP;
		AHT10_i2c_tx[2] = AHT10_I2C_CMD_DATA_NOP;
	}
	else{
		AHT10_i2c_tx[0] = AHT10_I2C_CMD_INIT;
		AHT10_i2c_tx[1] = AHT10_MODE_CYCLE | AHT10_ENABLE_CAL;
		AHT10_i2c_tx[2] = AHT10_I2C_CMD_DATA_NOP;
	}

	return HAL_I2C_Master_Transmit(dev->i2cHandle, AHT10_I2C_ADDR, (uint8_t *)AHT10_i2c_tx, 3, 100);
}
