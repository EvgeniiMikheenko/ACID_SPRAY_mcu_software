/**
  ******************************************************************************
  * File Name          : main.h
  * Description        : This file contains the common defines of the application
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2018 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H
  /* Includes ------------------------------------------------------------------*/

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private define ------------------------------------------------------------*/

#define ADC_TEMP_Pin GPIO_PIN_0
#define ADC_TEMP_GPIO_Port GPIOC
#define HEATER_Pin GPIO_PIN_2
#define HEATER_GPIO_Port GPIOC
#define DROP_ADC_Pin GPIO_PIN_3
#define DROP_ADC_GPIO_Port GPIOC
#define PUMP_PWM_Pin GPIO_PIN_1
#define PUMP_PWM_GPIO_Port GPIOA
#define PUMP_EN_Pin GPIO_PIN_2
#define PUMP_EN_GPIO_Port GPIOA
#define WHITE_LED_PWM_Pin GPIO_PIN_3
#define WHITE_LED_PWM_GPIO_Port GPIOA
#define OUT_FREQ_Pin GPIO_PIN_6
#define OUT_FREQ_GPIO_Port GPIOA
#define IR_ADC_Pin GPIO_PIN_7
#define IR_ADC_GPIO_Port GPIOA
#define OE_Pin GPIO_PIN_0
#define OE_GPIO_Port GPIOB
#define S1_Pin GPIO_PIN_1
#define S1_GPIO_Port GPIOB
#define S0_Pin GPIO_PIN_2
#define S0_GPIO_Port GPIOB
#define S3_Pin GPIO_PIN_10
#define S3_GPIO_Port GPIOB
#define S2_Pin GPIO_PIN_11
#define S2_GPIO_Port GPIOB
#define I11_Pin GPIO_PIN_12
#define I11_GPIO_Port GPIOB
#define I12_Pin GPIO_PIN_13
#define I12_GPIO_Port GPIOB
#define I13_Pin GPIO_PIN_14
#define I13_GPIO_Port GPIOB
#define I14_Pin GPIO_PIN_15
#define I14_GPIO_Port GPIOB
#define IO1_Pin GPIO_PIN_6
#define IO1_GPIO_Port GPIOC
#define I02_Pin GPIO_PIN_7
#define I02_GPIO_Port GPIOC
#define I03_Pin GPIO_PIN_8
#define I03_GPIO_Port GPIOC
#define I04_Pin GPIO_PIN_9
#define I04_GPIO_Port GPIOC
#define IR_LED_PWM_Pin GPIO_PIN_8
#define IR_LED_PWM_GPIO_Port GPIOA
#define LED1_PWM_Pin GPIO_PIN_9
#define LED1_PWM_GPIO_Port GPIOA
#define LED2_PWM_Pin GPIO_PIN_10
#define LED2_PWM_GPIO_Port GPIOA
#define LED3_PWM_Pin GPIO_PIN_11
#define LED3_PWM_GPIO_Port GPIOA
#define RS485_HOST_TXEN_Pin GPIO_PIN_15
#define RS485_HOST_TXEN_GPIO_Port GPIOA
#define RS485_HOST_TXD_Pin GPIO_PIN_10
#define RS485_HOST_TXD_GPIO_Port GPIOC
#define RS485_HOST_RXD_Pin GPIO_PIN_11
#define RS485_HOST_RXD_GPIO_Port GPIOC
#define PHASE1_Pin GPIO_PIN_3
#define PHASE1_GPIO_Port GPIOB
#define PHASE2_Pin GPIO_PIN_4
#define PHASE2_GPIO_Port GPIOB
#define PHASE3_Pin GPIO_PIN_5
#define PHASE3_GPIO_Port GPIOB
#define PHASE4_Pin GPIO_PIN_6
#define PHASE4_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */
#define MODBUS_DEVICE_INFO 			"ACID_SPRAY"
#define	InfoSizeRegNum						60000
#define	InfoRegNum								60001

#define VREF        3300



#define FLASH_LOCATION			((uint32_t)0x08020000-2*FLASH_PAGE_SIZE)			//FLASH_BANK1_END
#define EndAddr					((uint32_t)0x08020000)
#define FLASH_SIZE				(((unsigned short *)&Params.Crc - (unsigned short *) & Params.write_reg)/2+1)
#define CONFIG_REGISTERS_COUNT 	((unsigned short *)&Params.Crc - (unsigned short *) & Params.write_reg)    



#define UP		1
#define DOWN	0



#define OE(mode)	 			(mode == UP ? HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET): HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET))
#define S0(mode)	 			(mode == UP ? HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_SET): HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_RESET))
#define S1(mode)	 			(mode == UP ? HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET): HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET))
#define S2(mode)	 			(mode == UP ? HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, GPIO_PIN_SET): HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, GPIO_PIN_RESET))
#define S3(mode)	 			(mode == UP ? HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_SET): HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_RESET))




#define HEATER(mode)	 			(mode == UP ? HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, GPIO_PIN_SET): HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, GPIO_PIN_RESET))


#define PUMP_EN(mode)	 			(mode == UP ? HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_SET): HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_RESET))




#define PHASE1(mode)	 			(mode == UP ? HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_SET): HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_RESET))
#define PHASE2(mode)	 			(mode == UP ? HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_SET): HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_RESET))
#define PHASE3(mode)	 			(mode == UP ? HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_SET): HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET))
#define PHASE4(mode)	 			(mode == UP ? HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET): HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET))


#define I11(mode)	 			(mode == UP ? HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET): HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET))
#define I12(mode)	 			(mode == UP ? HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_SET): HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_RESET))
#define I13(mode)	 			(mode == UP ? HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET): HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET))
#define I14(mode)	 			(mode == UP ? HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_SET): HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_RESET))


#define I01(mode)	 			(mode == UP ? HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_SET): HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_RESET))
#define I02(mode)	 			(mode == UP ? HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_SET): HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_RESET))
#define I03(mode)	 			(mode == UP ? HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_SET): HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_RESET))
#define I04(mode)	 			(mode == UP ? HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_SET): HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_RESET))

#define RED     (1<<0)
#define GREEN   (1<<1)
#define BLUE    (1<<2)
#define WHITE   (1<<3)
#define IR      (1<<4)
#define FULL    (1<<8)
#define MID     (1<<9)
#define LOW     (1<<10)
#define OFF     (1<<11)
#define OE_FLAG (1<<12)
#define CHANGE_FLAG (1<<13)
#define HEATER_FLAG (1<<14)



/* USER CODE END Private defines */

void _Error_Handler(char *, int);

#define Error_Handler() _Error_Handler(__FILE__, __LINE__)

/**
  * @}
  */ 

/**
  * @}
*/ 

#endif /* __MAIN_H */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
