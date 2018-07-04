
#include <modbus_config.h>
//#include <modbs_master.h>
#include <stdint.h>
#include <stdbool.h>
#include "tim.h"


#define MOBUS_PACKET_SIZE	100
#define NULL		0
#define DEVICE_INFO 	0

#define RE_ON			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_SET)	
#define RE_OFF		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_RESET)



extern int mb_read_flag;
extern bool bridge;
//extern MbParam_t    PARAMS;

uint8_t m_mbRxBuf[MOBUS_PACKET_SIZE];
uint8_t m_mbTxBuf[MOBUS_PACKET_SIZE];

extern void rs485_start_tx( uint8_t data );
extern uint32_t GetSystemTime( void );

void host_init( void );
bool usart_init( uint32_t baudrate, uint8_t stopBits, uint8_t parity );
bool usart_enable( bool rxEn, bool txEn );

bool mb_timer_init( uint32_t us );
bool mb_timer_enable( void );
bool mb_timer_disable( void );
bool mb_timer_reset( void );

void mb_event( MBEvent_t event );
MbExceptionCodes_t mb_read_registers( uint8_t *lpDstBuf, uint16_t startAddr, uint16_t count );
MbExceptionCodes_t mb_write_registers( uint16_t startAddr, uint16_t count, uint8_t *lpSrcBuf );




uint32_t *m_lpRwRegsMode = NULL;
int m_RwRegsModeLen = 0;

MbDeviceData_t m_mbParam = 
{MB_ADDR, 					// uint8_t address;
	0, 							// uint8_t mode;
	
	false, 						// bool isInit;
	0, 							// uint32_t status;
	MB_SLAVE_STATE_IDLE,		// MbSlaveState_t state;
	
	115200, 					// uint32_t baudrate;
	1, 							// uint8_t stopBits;
	0, 							// uint8_t parity;
	
	m_mbRxBuf, 					// uint8_t *lpRxBuf;
	MOBUS_PACKET_SIZE, 		// unsigned int rxBufSize;
	0, 							// unsigned int rxIndex;
	
	m_mbTxBuf, 					// uint8_t *lpTxBuf;
	MOBUS_PACKET_SIZE, 		// unsigned int txBufSize;
	0, 							// unsigned int txIndex;
	0, 							// unsigned int txCount;
	
	usart_init, 				// pxMBSerialPortInit lpSerialInitPtr;
	usart_enable, 				// pxMBSerialPortEnable lpSerialEnablePtr;
	
	mb_timer_init, 				// pxMBTimerInit lpTimerInitPtr;
	mb_timer_enable, 			// pxMBTimerEneble lpTimerEnablePtr;
	mb_timer_disable, 			// pxMBTimerDisable lpTimerDisablePtr;
	mb_timer_reset, 			// pxMBTimerReset lpTimerResetPtr;
	
	mb_event, 					// pxMbEvent lpMbRiseEvent;
	
	NULL, 						// pxMbReadCoils lpExecuteReadCoils;
	NULL, 						// pxMbReadDescreteInputs lpExecuteReadDescreteInputs;
	mb_read_registers, 			// pxMbReadRegisters lpExecuteReadRegisters;
	NULL, 						// pxMbReadInputRegisters lpExecuteReadInputRegisters,
	mb_write_registers, 		// pxMbWriteRegisters lpExecuteWriteRegisters;
	NULL, 						// pxMbReadExcStatus lpExecuteReadExcStatus;
	NULL,						// pxMbWriteMultiCoils lpExecuteWriteMultiCoils;};


};

bool usart_init( uint32_t baudrate, uint8_t stopBits, uint8_t parity ) {
	return true;
}

bool usart_enable( bool rxEn, bool txEn )
{
//	if(txEn)
//	{
//		RE_ON;
//		return true;
//	}
//	if(rxEn)
//	{
//		RE_OFF;
//		return true;
//	}
	return true;
};



bool mb_timer_init( uint32_t us )			//// us  = 347
{
	us = 347;
	MX_TIM2_Init();
	SetPeriod( &htim2, us, 72000000 );
	return true;
};


bool mb_timer_enable( void )
{

	HAL_TIM_Base_Start_IT(&htim2);
	return true;

};

bool mb_timer_reset( void )
{

	TIM2->CNT = 0;
	HAL_TIM_Base_Start_IT(&htim2);
	return true;
};

bool mb_timer_disable( void )
{

		HAL_TIM_Base_Stop_IT(&htim2);
		return true;
};

//void TIM16_IRQHandler(void)
//{

//		//взять принятые и отправить  дальше
//		__HAL_TIM_CLEAR_IT( &htim16, TIM_IT_UPDATE);
//		
//	
//	
//	
//	
//};
	

//void get_device_info(uint8_t* lpDstBuf, uint32_t bufSize, uint32_t *lpWrLen) {
//	
//	if(lpWrLen != NULL)
//		*lpWrLen = 0;
//	
//	if((lpDstBuf == NULL) || (bufSize == 0))
//		return;
//	
//	int len = strlen(DEVICE_INFO);
//	*lpWrLen = len * 2;
//	
//	int index = 0;
//	char * str = DEVICE_INFO;
//	for(int i = 0; i < len; i++) {
//		index = i * 2;
//		
//		lpDstBuf[index] = 0x00;
//		lpDstBuf[index + 1] = str[i];
//	}
//};




void mb_event( MBEvent_t event ) {
	
	
	switch ( event ) {
	case MB_EVT_NONE:
		
		break;
	case MB_EVT_PARSE:
		mb_slave_parse_packet( &m_mbParam );
		break;
	case MB_EVT_TX:
		RE_ON;
		//rs485_start_tx( 0 );
		mb_tx( &m_mbParam );
		break;
	case MB_EVT_ADDR_INVALID:
			//HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET);				//DE
	//		bridge = true;

	//		PARAMS.Flags.bridge_delay_flag = false;
	//		bridge_delay_flag1	= false;
//			mb_stop_tx(&PARAMS);
//			mb_tx_reset(&PARAMS);
//			uint8_t messagesize;
//			messagesize = m_mbParam.rxIndex;
//	

//			
//	
//			
//			
//			
//	
//			for( int i = 0; i < messagesize; i++)
//			{
//					PARAMS.txBuf[i] = m_mbParam.lpRxBuf[i];
//			}
//			PARAMS.Flags.isRecvPacket = 0;
//			PARAMS.Flags.isPacket = true;
//			
//			mb_start_tx(&PARAMS, messagesize);



			
		break;
	default:
		
		break;
	}
};


//uint32_t m_hostTimeout = 0;
//bool m_isPasswordValid = false;


//MbExceptionCodes_t mb_read_registers( uint8_t *lpDstBuf, uint16_t startAddr, uint16_t count ) {
//	if(lpDstBuf == NULL)
//		return MB_EXC_ILLEGAL_FUNCTION;
//	
//	m_hostTimeout = 0;
//	m_isPasswordValid = false;
//	uint16_t endAddr = startAddr + count;
//	
//	int registersCount = sizeof(Params) / 2;
//	
//	if(startAddr < registersCount) {	
//		// ?????? ???????????? ?????????
//		if( endAddr > registersCount )
//			return MB_EXC_ILLEGAL_DATA_ADDR;
//		
//		uint16_t *lpRegs = (uint16_t*)&Params;
//		uint16_t reg;
//		//
//		for(uint32_t i = startAddr, j = 0; i < endAddr; i++, j += 2) {
//			reg = lpRegs[i];
//			lpDstBuf[j] = (uint8_t)(reg >> 8);
//			lpDstBuf[j + 1] = (uint8_t)(reg);
//		}
//		//
//		return MB_EXC_NONE;
//	}

//	//-------------------------------------------------------------------------
////	uint16_t size = strlen( MODBUS_DEVICE_INFO );
////	
////	// ?????? ?????????? ???????? ? ?????????? ?? ??????????
////	if(startAddr == InfoSizeRegNum) {
////		
////		if(count != 1) {
////			return MB_EXC_ILLEGAL_DATA_VALUE;
////		}
////		//
////		lpDstBuf[0] = (uint8_t)size >> 8;
////		lpDstBuf[1] = (uint8_t)(size);
////		//
////		return MB_EXC_NONE;
////	}
//	
//	// -----------------------------------------------------------------------
//	// ?????? ?????????? ?? ??????????
////	if(startAddr == InfoRegNum) {
////		
////		if(count != size) {
////			return MB_EXC_ILLEGAL_DATA_VALUE;
////		}
////		
////		char *ptr = MODBUS_DEVICE_INFO; // tBuf;
////		uint16_t reg;
////		
////		for(uint32_t i = 0, j = 0; i < size; i++, j += 2) {
////			reg = (uint16_t)ptr[i];
////			lpDstBuf[j] = (uint8_t)(reg >> 8);
////			lpDstBuf[j + 1] = (uint8_t)(reg);
////		}
////		//
////		return MB_EXC_NONE;
////	}
//	
//	return MB_EXC_ILLEGAL_FUNCTION;
//};




//MbExceptionCodes_t mb_write_registers( uint16_t startAddr, uint16_t count, uint8_t *lpSrcBuf ) {
//	
//	
//	
//	

//	return MB_EXC_ILLEGAL_FUNCTION;
//	
//};





void rs485_start_tx( uint8_t data )
{
		
	//HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET);			//RE;
	m_mbParam.txIndex = data;
	
				__HAL_UART_ENABLE_IT(&huart1, UART_IT_PE);

			/* Enable the UART Error Interrupt: (Frame error, noise error, overrun error) */
			__HAL_UART_ENABLE_IT(&huart1, UART_IT_ERR);

			/* Enable the UART Data Register not empty Interrupt */
			__HAL_UART_ENABLE_IT(&huart1, UART_IT_RXNE);
	
	HAL_UART_Transmit_IT( &huart1, m_mbParam.lpTxBuf, 1);


};

void mb_tx( MbDeviceData_t *Params )
{

		if( Params->txIndex < Params->txCount)
	 //
			HAL_UART_Transmit_IT( &huart1, &Params->lpTxBuf[Params->txIndex++], 1);
		else 
		{
		
				RE_OFF;
				
				mb_slave_start_packet( &m_mbParam );
				
		}
			

			return;


};

