/*
 * ina219.c
 *
 *  Created on: May 13, 2021
 *      Author: zeynep
 */

#include "ina219.h"

extern I2C_HandleTypeDef hi2c1;
#define i2cHandler hi2c1

/****************************READ & WRITE FUNCTIONS*****************************************/

static HAL_StatusTypeDef I2C_Write(uint8_t reg_addr, uint16_t reg_data, uint8_t addr) {

	uint8_t i2cData[3];
	i2cData[0] = reg_addr;
	i2cData[1] = (uint8_t)(reg_data >> 8); // MSB First
	i2cData[2] = (uint8_t)(reg_data & 0xFF);

	return HAL_I2C_Master_Transmit(&i2cHandler, addr<<1, i2cData, 3, 100);
}

static HAL_StatusTypeDef I2C_Read(uint8_t reg_addr,uint16_t *data, uint8_t addr){

	if(HAL_I2C_Master_Transmit(&i2cHandler, addr<<1, &reg_addr, 1, 100) == HAL_OK){
		HAL_Delay(5);
		uint8_t reg_data[2];
		if(HAL_I2C_Master_Receive(&i2cHandler, addr<<1, reg_data, 2, 100) == HAL_OK){
			*data = (reg_data[0] << 8) | (reg_data[1]);
			return HAL_OK;
		}
	}
	return HAL_ERROR;
}
/*******************************************************************************************/

HAL_StatusTypeDef INA219_Init(INA219 *ina){
	ina->addr = INA219_ADDRESS;
	ina->currentLSB = (float) (MAX_EXPECTED_CURRENT/32767.0);
	ina->powerLSB = (float) (20 * ina->currentLSB);
	ina->cal = (uint16_t) ( 0.04096 / ( ina->currentLSB * SHUNT_RESISTOR ) );

	return I2C_Write(INA219_CALIBRATION_REGISTER, ina->cal, ina->addr);
}

HAL_StatusTypeDef INA219_Config(INA219 *ina){
	return I2C_Write(INA219_CONFIG_REGISTER, INA219_ADC_CONFIG_MASK, ina->addr);
}

HAL_StatusTypeDef INA219_GetBusVoltage(INA219 *ina){

	uint16_t voltage;
	if(I2C_Read(INA219_BUS_VOLTAGE_REGISTER, &voltage, ina->addr) == HAL_OK){
		ina->busVoltage_V = (uint16_t) ((voltage >> 3) * 0.004);
		ina->busVoltage_mV = ina->busVoltage_V * 1000;
		return HAL_OK;
	}
	return HAL_ERROR;
}

HAL_StatusTypeDef INA219_GetShuntVoltage(INA219 *ina){

	uint16_t voltage;
	if(I2C_Read(INA219_SHUNT_VOLTAGE_REGISTER, &voltage, ina->addr) == HAL_OK){
		ina->shuntVoltage_V = ((uint16_t) voltage) * 0.00001;
		ina->shuntVoltage_mV = ina->shuntVoltage_V * 1000;
		return HAL_OK;
	}
	return HAL_ERROR;
}

HAL_StatusTypeDef INA219_GetCurrent(INA219 *ina){

	uint16_t current;
	if(I2C_Read(INA219_CURRENT_REGISTER, &current, ina->addr) == HAL_OK){
		if(!current){
			uint16_t calibration = 0;
			if(I2C_Read(INA219_CALIBRATION_REGISTER, &calibration, ina->addr) == HAL_OK){
				if(calibration == 0){
					I2C_Write(INA219_CALIBRATION_REGISTER, ina->cal, ina->addr);
					I2C_Write(INA219_CONFIG_REGISTER, INA219_ADC_CONFIG_MASK, ina->addr);
				}
				HAL_Delay(10);
				I2C_Read(INA219_CURRENT_REGISTER, &current, ina->addr);
			}
		}
		ina->current_A = ((int16_t)current) * ina->currentLSB;
		ina->current_mA = ina->current_A * 1000;


		return HAL_OK;
	}
	return HAL_ERROR;
}

HAL_StatusTypeDef INA219_GetPower_mW(INA219 *ina){

	uint16_t power;
	if(I2C_Read(INA219_CURRENT_REGISTER, &power, ina->addr) == HAL_OK){
		while(!power){
			uint16_t value = 0;
			I2C_Read(INA219_POWER_REGISTER, &value, ina->addr);

			if(!value){
				I2C_Write(INA219_CALIBRATION_REGISTER, ina->cal, ina->addr);
				//I2C_Write(INA219_CONFIG_REGISTER, INA219_DEFAULT_CONFIG_MASK);
			}
			HAL_Delay(10);
			I2C_Read(INA219_POWER_REGISTER, &power, ina->addr);
		}
		ina->power_mW = ((int16_t)power) * ina->powerLSB;
		ina->power_W = ina->power_mW * 0.001;
		return HAL_OK;
	}
	return HAL_ERROR;
}
HAL_StatusTypeDef INA219_PowerOnReset(INA219* ina){
	return I2C_Write(INA219_CONFIG_REGISTER,(1<<15), ina->addr);
}
