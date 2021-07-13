/*
 * ina219.h
 *
 *  Created on: May 13, 2021
 *      Author: zeynep
 */

#ifndef INC_INA219_H_
#define INC_INA219_H_

#include "stm32f4xx_hal.h"
#include <stdint.h>

#define MAX_EXPECTED_CURRENT 			3  //A
#define SHUNT_RESISTOR 					0.1//Ohm

#define INA219_ADDRESS 				   (0x40) //A0 -> GND & A1 -> GND

//Registers
#define INA219_CONFIG_REGISTER         (0x00)
#define INA219_SHUNT_VOLTAGE_REGISTER  (0x01)
#define INA219_BUS_VOLTAGE_REGISTER    (0x02)
#define INA219_POWER_REGISTER		   (0x03)
#define INA219_CURRENT_REGISTER        (0x04)
#define INA219_CALIBRATION_REGISTER    (0x05)

#define INA219_DEFAULT_CONFIG_MASK     (0x399F)
#define INA219_ADC_CONFIG_MASK         (0x3FFF)

typedef struct{
	double currentLSB;
	double powerLSB;
	uint32_t cal;
	uint16_t busVoltage_mV;
	double shuntVoltage_mV;
	double current_mA;
	double power_mW;
	double busVoltage_V;
	double shuntVoltage_V;
	double current_A;
	double power_W;
	uint8_t addr;
}INA219;

/*******************FUNCTION*PROTOTYPES*******************************************/
HAL_StatusTypeDef INA219_Init(INA219 *ina);
HAL_StatusTypeDef INA219_Config(INA219 *ina);
HAL_StatusTypeDef INA219_GetBusVoltage(INA219 *ina);
HAL_StatusTypeDef INA219_GetShuntVoltage(INA219 *ina);
HAL_StatusTypeDef INA219_GetCurrent(INA219 *ina);
HAL_StatusTypeDef INA219_GetPower_mW(INA219 *ina);
HAL_StatusTypeDef INA219_PowerOnReset(INA219 *ina);
/********************************************************************************/

#endif /* INC_INA219_H_ */
