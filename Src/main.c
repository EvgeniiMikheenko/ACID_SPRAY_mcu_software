/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f1xx_hal.h"
#include "adc.h"
#include "rtc.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* USER CODE BEGIN Includes */
#include <mb_slave.h>
#include <modbus_config.h>
#include <stdbool.h>
#include <crc16.h>
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

#define TIM3_period_us 			2000

typedef struct  {
    uint16_t ccr1_val;
    uint16_t ccr2_val;
} Scal_str;

typedef struct  {
    Scal_str Full;
    Scal_str Mid;
    Scal_str Low;
} Color_str;

typedef struct  {
    Color_str Red;
    Color_str Green;
    Color_str Blue;
    Color_str White;
} ColorScal_str;

ColorScal_str out_freq_val;

bool isPassword = false;
uint32_t m_hostTimeout = 0;
bool m_isPasswordValid = false;
mb_Regs Params;


unsigned * BUF1 = (unsigned *) &(Params.write_reg);
unsigned * BUF2 = (unsigned *) FLASH_LOCATION;
volatile uint32_t NbrOfPage = 0x00;
uint32_t EraseCounter = 0x00, Address = 0x00;
mb_Regs * Flash_Params = (mb_Regs *) FLASH_LOCATION;
unsigned short * PARAMS_BUF = (unsigned short *) & (Params.write_reg);
HAL_StatusTypeDef flash_status;
uint8_t red, green, blue, white, ir, pump;
uint16_t ccr1_val, ccr2_val;
uint16_t oe;
MbErrorCodes_t usart_init_msg;
extern  RTC_TimeTypeDef m_time;
extern  RTC_DateTypeDef m_date;
extern DMA_HandleTypeDef hdma_adc1;
bool is_init = false;
HAL_StatusTypeDef status;
uint16_t lpBuf[3];
uint8_t step_num = 0;
uint8_t step_num2 = 0;
bool frwd = true;
bool prew_frwd;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
void  Flash_Store( void );
bool Flash_Restore( void );
MbExceptionCodes_t mb_write_registers( uint16_t startAddr, uint16_t count, uint8_t *lpSrcBuf );
MbExceptionCodes_t mb_read_registers( uint8_t *lpDstBuf, uint16_t startAddr, uint16_t count );
extern int ExitCritSection( void );
void usart_int_enable( void );
void SetDutyCycle( TIM_HandleTypeDef *htim, uint8_t chnl, uint8_t prcnt );
void Set_PWM( void );
void SelectInpColor(uint8_t color, uint8_t scal);
void Color_Sens_Set(mb_Regs *Prm);
void Get_ColorSens_Val(  mb_Regs *Prm );

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_ADC1_Init();
  MX_RTC_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_USART3_UART_Init();
 
  /* USER CODE BEGIN 2 */
  MX_TIM4_Init();  
  usart_int_enable();
  Flash_Restore();
  mb_slave_init( &m_mbParam );
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
    __enable_irq();
  HAL_ADC_Start(&hadc1);
  status = HAL_ADC_Start_DMA( &hadc1, (uint32_t*)lpBuf, 3 );

//  HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_1);
//  HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_2);
  
  
  HAL_TIM_IC_Start(&htim3, TIM_CHANNEL_1);
  HAL_TIM_IC_Start(&htim3, TIM_CHANNEL_2);
  SetPeriod( &htim3, TIM3_period_us, 32000000 );
  HAL_NVIC_SetPriority(TIM3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(TIM3_IRQn);
  

  HAL_TIM_Base_Start_IT(&htim4);
  
  
  
  
  NVIC_EnableIRQ(USART3_IRQn);
  
	usart_int_enable();
    __HAL_UART_ENABLE_IT(&huart3, UART_IT_RXNE);
   usart_init_msg = mb_slave_init( &m_mbParam );
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
    OE(DOWN);
    oe = 0;
   is_init = true;
  HAL_GPIO_WritePin(PHASE1_GPIO_Port, PHASE1_Pin, RESET);
  HAL_GPIO_WritePin(PHASE2_GPIO_Port, PHASE2_Pin, RESET);
  pump = 84;
  SetDutyCycle(&htim2, 2, pump);
  HAL_GPIO_WritePin(PUMP_EN_GPIO_Port, PUMP_EN_Pin, SET);
  while (1)
  {
  /* USER CODE END WHILE */
      
    green = red = blue = white = Params.White_LVL; 
    SetDutyCycle(&htim2, 2, Params.Pump_LVL);
    SetDutyCycle(&htim1, 2, green);
    SetDutyCycle(&htim1, 3, blue);
    SetDutyCycle(&htim1, 4, red);
    SetDutyCycle(&htim1, 1, Params.IR_LVL);
    SetDutyCycle(&htim2, 4, white);
     
      
      
      
    
    Color_Sens_Set(&Params);
 //   filter_sel >= 3 ?  filter_sel = 0:filter_sel++;  
      HAL_Delay(10);
      
      
      
      
      
      /*
          SetDutyCycle(&htim2, 2, Params.Pump_LVL);
    SetDutyCycle(&htim1, 2, Params.Red_LVL);
    SetDutyCycle(&htim1, 3, Params.Green_LVL);      -> blue
      SetDutyCycle(&htim1, 4, Params.Blue_LVL);       -> red
    SetDutyCycle(&htim1, 1, Params.IR_LVL);
    SetDutyCycle(&htim2, 4, Params.White_LVL);
      */
  /* USER CODE BEGIN 3 */

  }
  /* USER CODE END 3 */

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL8;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_RTC|RCC_PERIPHCLK_ADC;
  PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSI;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV4;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}




/* USER CODE BEGIN 4 */
bool step = false;
uint8_t shift;
uint16_t wait_step = 0;
uint16_t wait = 0;
uint8_t stp_n = 1;
bool quarter = true;
void HAL_SYSTICK_Callback(void)
{
    if(!is_init)
        return;
   
    if(Params.write_reg && HEATER_FLAG)
        HAL_GPIO_WritePin(HEATER_GPIO_Port, HEATER_Pin, GPIO_PIN_SET);
    else
        HAL_GPIO_WritePin(HEATER_GPIO_Port, HEATER_Pin, GPIO_PIN_RESET);
//    if(frwd != prew_frwd)
//    {
//       
//    }
//    if(wait == 0)
//    {
if(!quarter)
{    
        GPIOB->ODR &= ~0x18;
        GPIOB->ODR |= ((0x3 << (step_num + 1)) & 0x18);
        step_num >= 3 ? step_num = 0: step_num++;
        wait = wait_step;
}
 else
 {     
    switch(stp_n)
    {
        case 1:
            I01(UP);
            I11(UP);
            PHASE1(UP); //X
            I02(DOWN);
            I12(DOWN);
            PHASE2(DOWN);
            if(!frwd)
                stp_n = 16;
            else stp_n++;
        break;
        case 2:
            I01(DOWN);
            I11(UP);
            PHASE1(UP);
            I02(DOWN);
            I12(DOWN);
            PHASE2(DOWN);
            if(!frwd)
                stp_n--;
            else stp_n++;
        break;
        case 3:
            I01(DOWN);  //UP
            I11(DOWN);
            PHASE1(UP);
            I02(DOWN);  //UP
            I12(DOWN);
            PHASE2(DOWN);
            if(!frwd)
                stp_n--;
            else stp_n++;
        break;
        case 4:
            I01(DOWN);
            I11(DOWN);
            PHASE1(UP);
            I02(DOWN);
            I12(UP);
            PHASE2(DOWN);
            if(!frwd)
                stp_n--;
            else stp_n++;
        break;
        case 5:
            I01(DOWN);
            I11(DOWN);
            PHASE1(UP);
            I02(UP);
            I12(UP);
            PHASE2(DOWN);       //X
            if(!frwd)
                stp_n--;
            else stp_n++;
        break;
        case 6:
            I01(DOWN);
            I11(DOWN);
            PHASE1(UP);
            I02(DOWN);
            I12(UP);
            PHASE2(UP);
            if(!frwd)
                stp_n--;
            else stp_n++;
        break;
        case 7:
            I01(DOWN);  //UP
            I11(DOWN);
            PHASE1(UP);
            I02(DOWN);  //UP
            I12(DOWN);
            PHASE2(UP);
            if(!frwd)
                stp_n--;
            else stp_n++;
        break;
        case 8:
            I01(DOWN);
            I11(UP);
            PHASE1(UP);
            I02(DOWN);
            I12(DOWN);
            PHASE2(UP);
            if(!frwd)
                stp_n--;
            else stp_n++;
        break;
        case 9:
            I01(UP);
            I11(UP);
            PHASE1(UP);     //X
            I02(DOWN);
            I12(DOWN);
            PHASE2(UP);
            if(!frwd)
                stp_n--;
            else stp_n++;
        break;
        case 10:
            I01(DOWN);
            I11(UP);
            PHASE1(DOWN);
            I02(DOWN);
            I12(DOWN);
            PHASE2(UP);
            if(!frwd)
                stp_n--;
            else stp_n++;
        break;
        case 11:
            I01(DOWN);      //UP
            I11(DOWN);
            PHASE1(DOWN);
            I02(DOWN);      //UP
            I12(DOWN);
            PHASE2(UP);
            if(!frwd)
                stp_n--;
            else stp_n++;
        break;
        case 12:
            I01(DOWN);
            I11(DOWN);
            PHASE1(DOWN);
            I02(DOWN);
            I12(UP);
            PHASE2(UP);
            if(!frwd)
                stp_n--;
            else stp_n++;
        break;
        case 13:
            I01(DOWN);
            I11(DOWN);
            PHASE1(DOWN);
            I02(UP);
            I12(UP);
            PHASE2(DOWN);   //X
            if(!frwd)
                stp_n--;
            else stp_n++;
        break;
        case 14:
            I01(DOWN);
            I11(DOWN);
            PHASE1(DOWN);
            I02(DOWN);
            I12(UP);
            PHASE2(DOWN);
            if(!frwd)
                stp_n--;
            else stp_n++;
        break;
        case 15:
            I01(DOWN);  //UP
            I11(DOWN);
            PHASE1(DOWN);
            I02(DOWN);  //UP
            I12(DOWN);
            PHASE2(DOWN);
            if(!frwd)
                stp_n--;
            else stp_n++;
        break;
        case 16:
            I01(DOWN);
            I11(UP);
            PHASE1(DOWN);
            I02(DOWN);
            I12(DOWN);
            PHASE2(DOWN);
            if(!frwd)
                stp_n--;
            else stp_n = 1;
        break;
    }
    
}    
    
//    }
//    else
//        wait--;
    
//    if( !step )
//    {
//        GPIOB->ODR &= ~0x60;
//        GPIOB->ODR |= ((0x3 << (step_num2 + 3)) & 0x60);
//        step_num2 >= 3 ? step_num2 = 0: step_num2++;
//        step = true;
//    }
//    else 
//        step = false;
    
 switch( step_num2 )
 {
     case 0:
         PHASE3(UP);  //X
         I03(UP);
         I13(UP);
        PHASE4(DOWN);
        I04(DOWN);
        I14(DOWN);
     step_num2 = 1;
     break;
     case 1:
         PHASE3(UP);
         I03(DOWN); //UP
         I13(DOWN);   
        PHASE4(DOWN);
        I04(DOWN);  //UP
        I14(DOWN);
        step_num2 = 2;
     break;
     case 2:
         PHASE3(UP);
        I03(DOWN);  //UP
        I13(DOWN);
        PHASE4(UP); //X
        I04(UP);  //UP
        I14(UP);
        step_num2 = 3;
     break;
     case 3:
         PHASE3(UP);
        I03(DOWN);  //UP
        I13(DOWN);
        PHASE4(UP);
        I04(DOWN); //UP
        I14(DOWN);
        step_num2 = 4;
     break;
     case 4:
         PHASE3(UP); //X
        I03(UP);
        I13(UP);
        PHASE4(UP);
        I04(DOWN);
        I14(DOWN);
        step_num2 = 5;
     break;
     case 5:
         PHASE3(DOWN);
        I03(DOWN);  //UP
        I13(DOWN);
        PHASE4(UP);
        I04(DOWN); //UP
        I14(DOWN);
        step_num2 = 6;
     break;
     case 6:
         PHASE3(DOWN);
         I03(DOWN);
        I13(DOWN);
        PHASE4(UP); //X
        I04(UP);
        I14(UP);
        step_num2 = 7;
     break;
     case 7:
         PHASE3(DOWN);
        I03(DOWN);   //UP
        I13(DOWN);
        PHASE4(DOWN);
        I04(DOWN);  //UP
        I14(DOWN);
        step_num2 = 0;
     break;
     default:
         step_num2 = 0;
     break;
 }
 
 prew_frwd = frwd;

    Get_ColorSens_Val(&Params);

    
    ccr1_val = htim3.Instance->CCR1;
    ccr2_val = htim3.Instance->CCR2;
   // Params.Drop_ADC = (hadc1.Instance->DR*VREF)<<14;
    HAL_ADC_Start(&hadc1);
    Params.Temperature = (lpBuf[0] * VREF)>>12;
    Params.Drop_ADC =  (lpBuf[1] * VREF)>>12;   
   // Params.Ir_ADC = (lpBuf[2] * VREF)>>12;
    status = HAL_ADC_Start_DMA( &hadc1, (uint32_t*)lpBuf, 3 );
  /* NOTE : This function Should not be modified, when the callback is needed,
            the HAL_SYSTICK_Callback could be implemented in the user file
   */
}

void Get_ColorSens_Val( mb_Regs *Prm )
{
    
        uint16_t filter_sel, scal_sel = 0;
        if(Prm->ColorSens_Flags & RED)
            filter_sel = RED;
        else if(Prm->ColorSens_Flags & GREEN)
            filter_sel = GREEN;
        if(Prm->ColorSens_Flags & BLUE)
            filter_sel = BLUE;
        else if(Prm->ColorSens_Flags & WHITE)
            filter_sel = WHITE;
        if(Prm->ColorSens_Flags & FULL)
            scal_sel = FULL;
        else if(Prm->ColorSens_Flags & MID)
            scal_sel = MID;
        if(Prm->ColorSens_Flags & LOW)
            scal_sel = LOW;
        else if(Prm->ColorSens_Flags & OFF)
            scal_sel = OFF;
    
    if(filter_sel == RED)
    {
        switch(scal_sel)
        {
            case LOW:
                out_freq_val.Red.Low.ccr1_val = htim3.Instance->CCR1;
                out_freq_val.Red.Low.ccr2_val = htim3.Instance->CCR2;
                break;
            case FULL:
                out_freq_val.Red.Full.ccr1_val = htim3.Instance->CCR1;
                out_freq_val.Red.Full.ccr2_val = htim3.Instance->CCR2;
                break;
            case MID:
                out_freq_val.Red.Mid.ccr1_val = htim3.Instance->CCR1;
                out_freq_val.Red.Mid.ccr2_val = htim3.Instance->CCR2;
                break;
        }
        Params.Red_freq = htim3.Instance->CCR2;
        
    }
    if( filter_sel == BLUE)
    {
        switch(scal_sel)
        {
            case LOW:
                out_freq_val.Blue.Low.ccr1_val = htim3.Instance->CCR1;
                out_freq_val.Blue.Low.ccr2_val = htim3.Instance->CCR2;
                break;
            case FULL:
                out_freq_val.Blue.Full.ccr1_val = htim3.Instance->CCR1;
                out_freq_val.Blue.Full.ccr2_val = htim3.Instance->CCR2;
                break;
            case MID:
                out_freq_val.Blue.Mid.ccr1_val = htim3.Instance->CCR1;
                out_freq_val.Blue.Mid.ccr2_val = htim3.Instance->CCR2;
                break;
        }
        Params.Blue_freq = htim3.Instance->CCR2;
    }
    if( filter_sel == GREEN )
    {
        switch(scal_sel)
        {
            case LOW:
                out_freq_val.Green.Low.ccr1_val = htim3.Instance->CCR1;
                out_freq_val.Green.Low.ccr2_val = htim3.Instance->CCR2;
                break;
            case FULL:
                out_freq_val.Green.Full.ccr1_val = htim3.Instance->CCR1;
                out_freq_val.Green.Full.ccr2_val = htim3.Instance->CCR2;
                break;
            case MID:
                out_freq_val.Green.Mid.ccr1_val = htim3.Instance->CCR1;
                out_freq_val.Green.Mid.ccr2_val = htim3.Instance->CCR2;
                break;
        } 
        Params.Green_freq = htim3.Instance->CCR2;
    }
    if(filter_sel == WHITE )
    {
        switch(scal_sel)
        {
            case LOW:
                out_freq_val.White.Low.ccr1_val = htim3.Instance->CCR1;
                out_freq_val.White.Low.ccr2_val = htim3.Instance->CCR2;
                break;
            case FULL:
                out_freq_val.White.Full.ccr1_val = htim3.Instance->CCR1;
                out_freq_val.White.Full.ccr2_val = htim3.Instance->CCR2;
                break;
            case MID:
                out_freq_val.White.Mid.ccr1_val = htim3.Instance->CCR1;
                out_freq_val.White.Mid.ccr2_val = htim3.Instance->CCR2;
                break;
        }
        Params.White_freq = htim3.Instance->CCR2;
    }
}
uint16_t chng = 0;
void Color_Sens_Set(mb_Regs *Prm)
{       
        uint16_t filter_sel, scal_sel = 0;
    
    
        if(Prm->ColorSens_Flags & CHANGE_FLAG)
        {
                Prm->ColorSens_Flags &= ~(RED + GREEN + BLUE + WHITE);
                Prm->ColorSens_Flags |= (1<<chng);
                chng >=3 ? chng = 0:chng++;
            
        }
    
        if(Prm->ColorSens_Flags & RED)
            filter_sel = RED;
        else if(Prm->ColorSens_Flags & GREEN)
            filter_sel = GREEN;
        if(Prm->ColorSens_Flags & BLUE)
            filter_sel = BLUE;
        else if(Prm->ColorSens_Flags & WHITE)
            filter_sel = WHITE;
        
            
        switch(filter_sel)
        {
            case RED:
                S2(DOWN);
                S3(DOWN);
                break;
            case GREEN:
                S2(UP);
                S3(UP);
                break;
            case BLUE:
                S2(DOWN);
                S3(UP);
                break;
            case WHITE:
                S2(UP);
                S3(DOWN);
                break;
            default:
                break;
        };
        
        if(Prm->ColorSens_Flags & FULL)
            scal_sel = FULL;
        else if(Prm->ColorSens_Flags & MID)
            scal_sel = MID;
        if(Prm->ColorSens_Flags & LOW)
            scal_sel = LOW;
        else if(Prm->ColorSens_Flags & OFF)
            scal_sel = OFF;
        
        
        switch(scal_sel)
        {
            case OFF:
                S0(DOWN);
                S1(DOWN);
                break;
            case FULL:
                S0(UP);
                S1(UP);
                break;
            case LOW:
                S0(DOWN);
                S1(UP);
                break;
            case MID:
                S0(UP);
                S1(DOWN);
                break;
            default:
                break;
        }
        if(Prm->ColorSens_Flags & OE_FLAG)
            OE(DOWN);
        else
            OE(UP);
        
        
}




MbExceptionCodes_t mb_read_registers( uint8_t *lpDstBuf, uint16_t startAddr, uint16_t count ) {
	if(lpDstBuf == NULL)
		return MB_EXC_ILLEGAL_FUNCTION;
	
	isPassword = false;
	m_hostTimeout = 0;
	m_isPasswordValid = false;
	uint16_t endAddr = startAddr + count;
	
	int registersCount = sizeof(Params) / 2;
	
	if(startAddr < registersCount) {	
		// ?????? ???????????? ?????????
		if( endAddr > registersCount )
			return MB_EXC_ILLEGAL_DATA_ADDR;
		
		uint16_t *lpRegs = (uint16_t*)&Params;
		uint16_t reg;
		//
		for(uint32_t i = startAddr, j = 0; i < endAddr; i++, j += 2) {
			reg = lpRegs[i];
			lpDstBuf[j] = (uint8_t)(reg >> 8);
			lpDstBuf[j + 1] = (uint8_t)(reg);
		}
		//
		return MB_EXC_NONE;
	}
	
	//-------------------------------------------------------------------------
	uint16_t size = strlen( MODBUS_DEVICE_INFO );
	
	// ?????? ?????????? ???????? ? ?????????? ?? ??????????
	if(startAddr == InfoSizeRegNum) {
		
		if(count != 1) {
			return MB_EXC_ILLEGAL_DATA_VALUE;
		}
		//
		lpDstBuf[0] = (uint8_t)size >> 8;
		lpDstBuf[1] = (uint8_t)(size);
		//
		return MB_EXC_NONE;
	}
	
	// -----------------------------------------------------------------------
	// ?????? ?????????? ?? ??????????
	if(startAddr == InfoRegNum) {
		
		if(count != size) {
			return MB_EXC_ILLEGAL_DATA_VALUE;
		}
		
		char *ptr = MODBUS_DEVICE_INFO; // tBuf;
		uint16_t reg;
		
		for(uint32_t i = 0, j = 0; i < size; i++, j += 2) {
			reg = (uint16_t)ptr[i];
			lpDstBuf[j] = (uint8_t)(reg >> 8);
			lpDstBuf[j + 1] = (uint8_t)(reg);
		}
		//
		return MB_EXC_NONE;
	}
	
	return MB_EXC_ILLEGAL_FUNCTION;
};



MbExceptionCodes_t mb_write_registers( uint16_t startAddr, uint16_t count, uint8_t *lpSrcBuf ) {
	
//	if( !isPassword )
//	{
//	
//		if(((lpSrcBuf[0]<<8) + lpSrcBuf[1]) == Password	)
//			isPassword = true;
//			
//		return NULL;
//	}
	
	
	
	if(lpSrcBuf == NULL)
		return MB_EXC_ILLEGAL_FUNCTION;
	
	m_hostTimeout = 0;
	m_isPasswordValid = false;
	uint16_t endAddr = startAddr + count;
	uint16_t *tmp_ptr = (uint16_t *)&Params;
	
	uint16_t tmp_reg_val;
	
	
	//int i = startAddr;
	int j = startAddr;
	
	
	
	for( int i = startAddr; i < endAddr; i++)
	{
		tmp_reg_val = (lpSrcBuf[j - startAddr ] << 8) + lpSrcBuf[j - startAddr+1];
		
		//tmp_ptr[i+2] = lpSrcBuf[i - startAddr];
		tmp_ptr[i] = tmp_reg_val;
		j+=2;
//		tmp_ptr [1] = 0x77;
//		tmp_ptr [0] = 0x9988;
//	
	}
	
	
	
	

		//
		return MB_EXC_NONE;
	
	

	//return MB_EXC_ILLEGAL_FUNCTION;
	
};



void  Flash_Store( void )
{

		EnterCritSection();
		
		uint8_t j=0;
		for ( int i=0;i<FLASH_SIZE;i++)
		{
			if (BUF1[i]!=BUF2[i])
				j=1;
		}
		
		if(!j) {
			ExitCritSection();
			return;
		}
	
		HAL_FLASH_Unlock();
		
		
		NbrOfPage = (EndAddr - FLASH_LOCATION) / FLASH_PAGE_SIZE;
//		HAL_FLASH_Unlock();
		//стереть
		for(EraseCounter = 0; (EraseCounter < NbrOfPage); EraseCounter++)
		{
			FLASH_PageErase(FLASH_LOCATION + (FLASH_PAGE_SIZE * EraseCounter));
			while(FLASH->SR & FLASH_SR_BSY )
			{}
			//FLASH_WaitForLastOperation(10);
		}
		HAL_FLASH_Lock();

		Address = FLASH_LOCATION;
		int i=FLASH_SIZE;

		Params.Crc = CalcCrc16((unsigned char *) & (Params.write_reg), FLASH_SIZE*4-4);
		
		HAL_FLASH_Unlock();

		for (i=0;i<FLASH_SIZE;i++) {
			if (BUF1[i]!=BUF2[i])
			{
				FLASH->CR &= ~(FLASH_CR_PER);
				FLASH->CR |= FLASH_CR_PG;
				flash_status = HAL_FLASH_Program( FLASH_TYPEPROGRAM_WORD, FLASH_LOCATION+(i<<2), BUF1[i]);
				while(FLASH->SR & FLASH_SR_BSY )
				{}
			}
		}
		//flash_status = HAL_FLASH_Program( FLASH_TYPEPROGRAM_WORD, 0x08007C96, 0x09);
			
		HAL_FLASH_Lock();
			
		ExitCritSection();
				
	
};

bool Flash_Restore( void )
{
	if (CalcCrc16((unsigned char *) Flash_Params, FLASH_SIZE*4-4) != *((uint16_t *)(FLASH_LOCATION + CONFIG_REGISTERS_COUNT*2)))
	
	{
		for (int i = 0; i < FLASH_SIZE ; i++)
			PARAMS_BUF[i] = 0;
		return false;
		
	}
	else
	{
		for (int i = 0; i < FLASH_SIZE  ; i++)
			BUF1[i] = BUF2[i];
		return true;
		
	}
};

void Time_Update( mb_Regs *Prm)
{

	HAL_RTC_GetTime(&hrtc, &m_time, RTC_FORMAT_BIN);
	Prm->DT.Time.Seconds = m_time.Seconds;
	Prm->DT.Time.Minutes = m_time.Minutes;
	Prm->DT.Time.Hours = m_time.Hours;
	
	HAL_RTC_GetDate(&hrtc, &m_date, RTC_FORMAT_BIN);
	Prm->DT.Date.Day = m_date.Date;
	Prm->DT.Date.Month  = m_date.Month;
	Prm->DT.Date.Year = m_date.Year;

};







void Time_Set( mb_Regs * Prm )
{

	m_time.Seconds = Prm->DT.Time.Seconds;
	m_time.Minutes = Prm->DT.Time.Minutes;
	m_time.Hours = Prm->DT.Time.Hours;
	HAL_RTC_SetTime( &hrtc, &m_time, RTC_FORMAT_BIN);
	
	
	m_date.Date = Prm->DT.Date.Day;
	m_date.Month = Prm->DT.Date.Month;
	m_date.Year = Prm->DT.Date.Year;
	HAL_RTC_SetDate( &hrtc, &m_date, RTC_FORMAT_BIN);
	
	
};

void usart_int_enable( void )
{

				
						// ???????????????????????????????????????????????
			/* Enable the UART Parity Error Interrupt */
			__HAL_UART_ENABLE_IT(&huart3, UART_IT_PE);

			/* Enable the UART Error Interrupt: (Frame error, noise error, overrun error) */
			__HAL_UART_ENABLE_IT(&huart3, UART_IT_ERR);

			/* Enable the UART Data Register not empty Interrupt */
			__HAL_UART_ENABLE_IT(&huart3, UART_IT_RXNE);
	
	

		//	__HAL_UART_ENABLE_IT(&huart1, UART_IT_TXE);
			
			

};
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) 
{
//		static bool isRidgeEn = false;
		
		if( huart->Instance == USART3)
		{
			
			
			uint8_t data = (uint8_t)huart->Instance->DR;
			
			MbErrorCodes_t err_msg = mb_slave_recive( &m_mbParam, data );
			
//			
//			uint8_t data11 = huart->Instance->DR;
//			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_SET);
//			HAL_UART_Transmit_IT(&huart1, &data11, 1);
//			while(HAL_UART_GetState( &huart1 ) == HAL_UART_STATE_BUSY)
//				{};
//			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_RESET);
		
		};

};


void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
	if( huart->Instance == USART3)
	{
	 if( huart->Instance == USART3 )
			mb_tx(&m_mbParam);
	}
};


void USART3_IRQHandler(void)						// второе определение закомментировано в stm32f1xx_it.c
{

	
		HAL_UART_IRQHandler( &huart3 );

		return;

};



void SetDutyCycle( TIM_HandleTypeDef *htim, uint8_t chnl, uint8_t prcnt )
{
    switch( chnl )
    {
        case 1:
            htim->Instance->CCR1 = (htim->Instance->ARR * prcnt)/100;
        break;
        case 2:
            htim->Instance->CCR2 = (htim->Instance->ARR * prcnt)/100;
        break;
        case 3:
            htim->Instance->CCR3 = (htim->Instance->ARR * prcnt)/100;
        break;
        case 4:
            htim->Instance->CCR4 = (htim->Instance->ARR * prcnt)/100;
        break;
        default: break;
    }


};

void Set_PWM( void )
{

    SetDutyCycle(&htim2, 2, Params.Pump_LVL);
    SetDutyCycle(&htim1, 2, Params.Red_LVL);
    SetDutyCycle(&htim1, 3, Params.Green_LVL);
    SetDutyCycle(&htim1, 4, Params.Blue_LVL);
    SetDutyCycle(&htim1, 1, Params.IR_LVL);
    SetDutyCycle(&htim2, 4, Params.White_LVL);
    
    
};




void TIM4_IRQHandler( void )				// второе определение закомментировано в stm32f1xx_it.c
{
			__HAL_TIM_CLEAR_IT( &htim4, TIM_IT_UPDATE);
					mb_slave_timer_expired(&m_mbParam);

}
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{

  /* NOTE : This function Should not be modified, when the callback is needed,
            the __HAL_TIM_IC_CaptureCallback could be implemented in the user file
   */
}

void DMA1_Channel1_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Channel1_IRQn 0 */

  /* USER CODE END DMA1_Channel1_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_adc1);
  /* USER CODE BEGIN DMA1_Channel1_IRQn 1 */


  /* USER CODE END DMA1_Channel1_IRQn 1 */
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void _Error_Handler(char * file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1) 
  {
  }
  /* USER CODE END Error_Handler_Debug */ 
}

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
