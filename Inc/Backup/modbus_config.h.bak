
#ifndef  MODBUS_CONFIG_H
#define  MODBUS_CONFIG_H



#include <stdbool.h>
#include <stdint.h>
#include <mb_slave.h>
//#include <stm32f100xb.h>
//#include <stm32f1xx_hal_gpio.h>
#include <tim.h>
#include <mb.h>
#include <usart.h>


#define MB_ADDR		0x01

#define MB_FUNC_READ_COILS_ENABLE					0
#define MB_FUNC_READ_DIGITAL_INPUTS				0
#define MB_FUNC_READ_HOLDING_REGISTERS 		1
#define MB_FUNC_READ_INPUT_REGISTERS 			0
#define MB_FUNC_WRITE_SINGLE_COIL 				0
#define MB_FUNC_WRITE_SINGLE_REG 					1
#define MB_FUNC_WRITE_MULTI_REGS 					1
#define MB_FUNC_READ_EXCEPTION_STATUS 		0
#define MB_FUNC_WRITE_MULTI_COIL					0
#define MB_FUNC_READ_FILE_RECORD					0
#define MB_FUNC_WRITE_FILE_RECORD					0
#define MB_FUNC_READ_DEVICE_ID 						0


#define GPIO_BIT_SET		1
#define GPIO_BIT_RESET	0
#define rxE				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_RESET);
#define txE				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_SET);

extern MbDeviceData_t m_mbParam;

extern void mb_tx( MbDeviceData_t *Params );

extern bool bridge_delay_flag1;

#endif		//MODBUS_CONFIG_H
