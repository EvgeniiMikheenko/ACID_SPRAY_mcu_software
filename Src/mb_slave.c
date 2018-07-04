//==============================================================================
//
//==============================================================================

#include <mb_slave.h>
#include <crc16.h>
//#include <utils.h>

//------------------------------------------------------------------------------
extern int EnterCritSection( void );
extern int ExitCritSection( void );
//------------------------------------------------------------------------------
void mb_slave_send_error( MbDeviceData_t *lpDevData, uint8_t func, uint8_t errCode );
//------------------------------------------------------------------------------

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

volatile int IrqCounter = 0;
int EnterCritSection( void )
{
		__disable_irq();
		IrqCounter++;
		return 0;

};

int ExitCritSection( void )
{
		IrqCounter--;
		if( IrqCounter == 0 )
				__enable_irq();
		
		return 0;
	
};


MbErrorCodes_t mb_slave_init( MbDeviceData_t *lpDevData ) {
	if(lpDevData == (MbDeviceData_t *)NULL)
		return MB_ERR_INVALID_DATA;
	//
	lpDevData->isInit = false;
	
	if(lpDevData->lpSerialInitPtr == NULL)
		return MB_ERR_INVALID_FUNCTION_PTR;
	
	if(lpDevData->lpSerialEnablePtr == NULL)
		return MB_ERR_INVALID_FUNCTION_PTR;
	
	if(lpDevData->lpTimerInitPtr == NULL)
		return MB_ERR_INVALID_FUNCTION_PTR;
	
	if(lpDevData->lpTimerResetPtr == NULL)
		return MB_ERR_INVALID_FUNCTION_PTR;
	
	if(lpDevData->lpTimerEnablePtr == NULL)
		return MB_ERR_INVALID_FUNCTION_PTR;
	
	if(lpDevData->lpTimerDisablePtr == NULL)
		return MB_ERR_INVALID_FUNCTION_PTR;
	
	if( (lpDevData->lpRxBuf == NULL) || (lpDevData->rxBufSize == 0))
		return MB_ERR_INVALID_DATA;
	
	if( (lpDevData->lpTxBuf == NULL) || (lpDevData->txBufSize == 0))
		return MB_ERR_INVALID_DATA;
	
	if( (lpDevData->address < MB_DEVICE_ADDRESS_MIN) || (lpDevData->address > MB_DEVICE_ADDRESS_MAX))
		return MB_ERR_INVALID_DEVICE_ADDR;
	
	if( !(*lpDevData->lpSerialInitPtr)( lpDevData->baudrate, lpDevData->stopBits, lpDevData->parity ) )
		return MB_ERR_SERIAL_INIT;
	
	if( !(*lpDevData->lpSerialEnablePtr)( false, false ) )
		return MB_ERR_SERIAL_INIT;
	
	uint32_t timerInterval = ( ((10 * 10 * 1000000UL) / lpDevData->baudrate) );
	
	if( !(*lpDevData->lpTimerInitPtr)( timerInterval ) )
	   return MB_ERR_TIMER_INIT;
		
	(*lpDevData->lpTimerDisablePtr)();
	
	lpDevData->isInit = true;
	lpDevData->rxIndex = 0;
	lpDevData->txCount = 0;
	lpDevData->txIndex = 0;
	lpDevData->state = MB_SLAVE_STATE_IDLE;
	
	if( !(*lpDevData->lpSerialEnablePtr)( true, true ) )
		return MB_ERR_SERIAL_INIT;
	
	return MB_ERR_NONE;
}

MbErrorCodes_t mb_slave_parse_packet( MbDeviceData_t *lpDevData ) {
	if(lpDevData == (MbDeviceData_t *)NULL)
		return MB_ERR_INVALID_DATA;
	
	if( !lpDevData->isInit )
		return MB_ERR_NOINIT;
	
	if( (lpDevData->status & MB_SLAVE_STATUS_PARSE_ENABLE) == 0 ) {
		// Обработка пакета не разрешена
		return MB_ERR_NONE;
	}
	
	EnterCritSection();
	{
		if( (lpDevData->status & MB_SLAVE_STATUS_PARSED_START)) {
			// Обработка пакета уже запущена
			ExitCritSection();
			return MB_ERR_NONE;
		}
		lpDevData->status |= MB_SLAVE_STATUS_PARSED_START;
	}
	ExitCritSection();
	
	if( lpDevData->rxIndex <= ( MB_PACKET_SIZE_MIN ) ) {
		// Размер пакета не может быть маньше MB_PACKET_SIZE_MIN
		mb_slave_start_packet( lpDevData );
		return MB_ERR_PACKET_SIZE_MIN;
	}
	
	uint16_t rxCrc = CalcCrc16( lpDevData->lpRxBuf, lpDevData->rxIndex );
	if(rxCrc != 0) {
		// CRC не верна
		mb_slave_start_packet( lpDevData );
		return MB_ERR_PACKET_CRC;
	}
	
	if( lpDevData->address != lpDevData->lpRxBuf[MB_PACKET_ADDR_INDEX] ) {
		// Адрес не наш
		if( lpDevData->lpMbRiseEvent != NULL ) {
				(*lpDevData->lpMbRiseEvent)( MB_EVT_ADDR_INVALID );
		}
		mb_slave_start_packet( lpDevData );
		return MB_ERR_NONE;
	}
	
	uint8_t *lpRxBuf = lpDevData->lpRxBuf;
	uint8_t *lpTxBuf = lpDevData->lpTxBuf;
	//
	uint16_t startAddr, count, byteCount;
	MbExceptionCodes_t result;
	uint32_t txSize = 0;
	uint8_t func = lpDevData->lpRxBuf[ MB_PACKET_FUNC_CODE_INDEX ];
	

	
	
	
	
	switch( func ) {
		
#if MB_FUNC_READ_COILS_ENABLE > 0
	//--------------------------------------------------------------------------
	// Read Coils support
	//--------------------------------------------------------------------------
	case MB_CMD_READ_COILS: 			/* = 0x01 - Чтение статуса выходов 						*/
		startAddr = (uint16_t)((lpRxBuf[2] << 8) | lpRxBuf[3]);
		count = (uint16_t)((lpRxBuf[4] << 8) | lpRxBuf[5]);
		
		if( lpDevData->lpExecuteReadCoils == NULL ) {
			mb_slave_send_error( lpDevData, func, MB_EXC_ILLEGAL_FUNCTION );
			break;
		}
		
		result = (*lpDevData->lpExecuteReadCoils)( &lpTxBuf[3], startAddr, count, &byteCount );
		if(result != MB_EXC_NONE) {
			mb_slave_send_error( lpDevData, func, result );
			break;
		}
		// Отправляем данные
		txSize = 3 + byteCount + 2;
					
		lpTxBuf[ MB_PACKET_ADDR_INDEX ] 		= lpDevData->address;
		lpTxBuf[ MB_PACKET_FUNC_CODE_INDEX ] 	= func;
		lpTxBuf[ 2 ] = byteCount;
		
		break;
#endif // MB_FUNC_READ_COILS_ENABLE > 0
		
#if MB_FUNC_READ_DIGITAL_INPUTS > 0
	//--------------------------------------------------------------------------
	// Read Descrete Inputs support
	//--------------------------------------------------------------------------	
	case MB_CMD_READ_DINPUTS: 			/* = 0x02 - Чтение состояния дискретных входов 			*/
		startAddr = (uint16_t)((lpRxBuf[2] << 8) | lpRxBuf[3]);
		count = (uint16_t)((lpRxBuf[4] << 8) | lpRxBuf[5]);
		
		if( lpDevData->lpExecuteReadDescreteInputs == NULL ) {
			mb_slave_send_error( lpDevData, func, MB_EXC_ILLEGAL_FUNCTION );
			break;
		}
		
		result = (*lpDevData->lpExecuteReadDescreteInputs)( &lpTxBuf[3], startAddr, count, &byteCount );
		if(result != MB_EXC_NONE) {
			mb_slave_send_error( lpDevData, func, result );
			break;
		}
		// Отправляем данные
		txSize = 3 + byteCount + 2;
					
		lpTxBuf[ MB_PACKET_ADDR_INDEX ] 		= lpDevData->address;
		lpTxBuf[ MB_PACKET_FUNC_CODE_INDEX ] 	= func;
		lpTxBuf[ 2 ] = byteCount;
			
		break;
#endif // MB_FUNC_READ_DIGITAL_INPUTS > 0
	
#if MB_FUNC_READ_HOLDING_REGISTERS > 0
	//--------------------------------------------------------------------------
	// Read Holding Regirters support
	//--------------------------------------------------------------------------	
	case MB_CMD_READ_HOLDING_REGS: 		/* = 0x03 - Чтение содержимого регистров 				*/
		startAddr = (uint16_t)((lpRxBuf[2] << 8) | lpRxBuf[3]);
		count = (uint16_t)((lpRxBuf[4] << 8) | lpRxBuf[5]);
		
		if((count == 0) || (count > MODBUS_REGS_IOCOUNT_MAX)) {
			mb_slave_send_error( lpDevData, func, MB_EXC_ILLEGAL_DATA_VALUE );
			break;
		}
		
		if( lpDevData->lpExecuteReadRegisters == NULL ) {
			mb_slave_send_error( lpDevData, func, MB_EXC_ILLEGAL_FUNCTION );
			break;
		}
		
		result = (*lpDevData->lpExecuteReadRegisters)( &lpTxBuf[3], startAddr, count );
		if(result != MB_EXC_NONE) {
			mb_slave_send_error( lpDevData, func, result );
			break;
		}
		
		byteCount = count << 1;
		// Отправляем данные
		txSize = 3 + byteCount + 2;
					
		lpTxBuf[ MB_PACKET_ADDR_INDEX ] 		= lpDevData->address;
		lpTxBuf[ MB_PACKET_FUNC_CODE_INDEX ] 	= func;
		lpTxBuf[ 2 ] = byteCount;
		
		break;
#endif // MB_FUNC_READ_HOLDING_REGISTERS > 0

#if MB_FUNC_READ_INPUT_REGISTERS > 0
	//--------------------------------------------------------------------------
	// Read Input Regirters support
	//--------------------------------------------------------------------------	
	case MB_CMD_READ_INPUT_REGS: 		/* = 0x04 - Чтение содержимого входных регистров 		*/
		startAddr = (uint16_t)((lpRxBuf[2] << 8) | lpRxBuf[3]);
		count = (uint16_t)((lpRxBuf[4] << 8) | lpRxBuf[5]);
		
		if((count == 0) || (count > MODBUS_REGS_IOCOUNT_MAX)) {
			mb_slave_send_error( lpDevData, func, MB_EXC_ILLEGAL_DATA_VALUE );
			break;
		}
		
		if( lpDevData->lpExecuteReadInputRegisters == NULL ) {
			mb_slave_send_error( lpDevData, func, MB_EXC_ILLEGAL_FUNCTION );
			break;
		}
		
		result = (*lpDevData->lpExecuteReadInputRegisters)( &lpTxBuf[3], startAddr, count );
		if(result != MB_EXC_NONE) {
			mb_slave_send_error( lpDevData, func, result );
			break;
		}
		
		byteCount = count << 1;
		// Отправляем данные
		txSize = 3 + byteCount + 2;
					
		lpTxBuf[ MB_PACKET_ADDR_INDEX ] 		= lpDevData->address;
		lpTxBuf[ MB_PACKET_FUNC_CODE_INDEX ] 	= func;
		lpTxBuf[ 2 ] = byteCount;
		break;
#endif // MB_FUNC_READ_INPUT_REGISTERS > 0
		
#if MB_FUNC_WRITE_SINGLE_COIL > 0	
	//--------------------------------------------------------------------------
	// Write Single Coil support
	//--------------------------------------------------------------------------		
	case MB_CMD_WRITE_SINGLE_COIL: 		/* = 0x05 - Уствновка единичного выхода в ON или OFF 	*/
		#error "Modbus Fuction code 0x05 not supported"
		break;
#endif // MB_FUNC_WRITE_SINGLE_COIL > 0
		
#if MB_FUNC_WRITE_SINGLE_REG > 0
	//--------------------------------------------------------------------------
	// Write Single Register support
	//--------------------------------------------------------------------------			
	case MB_CMD_WRITE_SINGLE_REG: 		/* = 0x06 - Запись в единичный регистр 					*/
		startAddr = (uint16_t)((lpRxBuf[2] << 8) | lpRxBuf[3]);
		count = 1;
		
		if( lpDevData->lpExecuteWriteRegisters == NULL ) {
			mb_slave_send_error( lpDevData, func, MB_EXC_ILLEGAL_FUNCTION );
			break;
		}
		
		result = (*lpDevData->lpExecuteWriteRegisters)( startAddr, count, &lpRxBuf[4]);
		if(result != MB_EXC_NONE) {
			mb_slave_send_error( lpDevData, func, result );
			break;
		}
		
		// Отправляем данные
		txSize = 2 + 4 + 2;
					
		lpTxBuf[ MB_PACKET_ADDR_INDEX ] 		= lpDevData->address;
		lpTxBuf[ MB_PACKET_FUNC_CODE_INDEX ] 	= func;
		lpTxBuf[ 2 ] = lpRxBuf[ 2 ];
		lpTxBuf[ 3 ] = lpRxBuf[ 3 ];
		lpTxBuf[ 4 ] = lpRxBuf[ 4 ];
		lpTxBuf[ 5 ] = lpRxBuf[ 5 ];
		
		break;
#endif // MB_FUNC_WRITE_SINGLE_REG > 0
	
#if MB_FUNC_READ_EXCEPTION_STATUS > 0
	//--------------------------------------------------------------------------
	// Read Exception status support
	//--------------------------------------------------------------------------		
	case MB_CMD_READ_EXCEPTION_STATUS: 	/* = 0x07 - Чтение статуса устройства 					*/
		
		if(lpDevData->lpExecuteReadExcStatus == NULL) {
			mb_slave_send_error( lpDevData, func, MB_EXC_ILLEGAL_FUNCTION );
			break;
		}
		
		result = (*lpDevData->lpExecuteReadExcStatus)( &lpRxBuf[2]);
		if(result != MB_EXC_NONE) {
			mb_slave_send_error( lpDevData, func, result );
			break;
		}
		
		// Отправляем данные
		txSize = 2 + 1 + 2;
					
		lpTxBuf[ MB_PACKET_ADDR_INDEX ] 		= lpDevData->address;
		lpTxBuf[ MB_PACKET_FUNC_CODE_INDEX ] 	= func;
		
		break;
#endif // MB_FUNC_READ_EXCEPTION_STATUS > 0
		
#if MB_FUNC_WRITE_MULTI_COIL > 0
	//--------------------------------------------------------------------------
	// Write Multi Coil support
	//--------------------------------------------------------------------------	
	case MB_CMD_WRITE_MULTI_COILS: 		/* = 0x0F - Установка множества выходов в ON или OFF 	*/
		startAddr = (uint16_t)((lpRxBuf[2] << 8) | lpRxBuf[3]);
		count = (uint16_t)((lpRxBuf[4] << 8) | lpRxBuf[5]);
		byteCount = lpRxBuf[6];
		
		if(lpDevData->lpExecuteWriteMultiCoils == NULL) {
			mb_slave_send_error( lpDevData, func, MB_EXC_ILLEGAL_FUNCTION );
			break;
		}
		
		uint16_t bytes = count / 8;
		if((count % 8) > 0)
			bytes++;
		
		if(bytes != byteCount) {
			mb_slave_send_error( lpDevData, func, MB_EXC_ILLEGAL_DATA_VALUE );
			break;
		}
		
		result = (*lpDevData->lpExecuteWriteMultiCoils)( startAddr, count, &lpRxBuf[7]);
		if(result != MB_EXC_NONE) {
			mb_slave_send_error( lpDevData, func, result );
			break;
		}
		
		// Отправляем данные
		txSize = 2 + 4 + 2;
					
		lpTxBuf[ MB_PACKET_ADDR_INDEX ] 		= lpDevData->address;
		lpTxBuf[ MB_PACKET_FUNC_CODE_INDEX ] 	= func;
		lpTxBuf[ 2 ] = lpRxBuf[ 2 ];
		lpTxBuf[ 3 ] = lpRxBuf[ 3 ];
		lpTxBuf[ 4 ] = lpRxBuf[ 4 ];
		lpTxBuf[ 5 ] = lpRxBuf[ 5 ];
		
		break;
#endif // MB_FUNC_WRITE_MULTI_COIL > 0

#if MB_FUNC_WRITE_MULTI_REGS > 0
	//--------------------------------------------------------------------------
	// Write Multi Register support
	//--------------------------------------------------------------------------		
	case MB_CMD_WRITE_MULTI_REGS: 		/* = 0x10 - Запись в несколько регистров 				*/
		startAddr = (uint16_t)((lpRxBuf[2] << 8) | lpRxBuf[3]);
		count = (uint16_t)((lpRxBuf[4] << 8) | lpRxBuf[5]);
		byteCount = lpRxBuf[6];
		
		if( lpDevData->lpExecuteWriteRegisters == NULL ) {
			mb_slave_send_error( lpDevData, func, MB_EXC_ILLEGAL_FUNCTION );
			break;
		}
		
		if( (count == 0) || (count > MODBUS_REGS_IOCOUNT_MAX) || ((count * 2) != byteCount ) ) {
			mb_slave_send_error( lpDevData, func, MB_EXC_ILLEGAL_DATA_VALUE );
			break;
		}
		
		result = (*lpDevData->lpExecuteWriteRegisters)( startAddr, count, &lpRxBuf[7]);
		if(result != MB_EXC_NONE) {
			mb_slave_send_error( lpDevData, func, result );
			break;
		}
		
		// Отправляем данные
		txSize = 2 + 4 + 2;
					
		lpTxBuf[ MB_PACKET_ADDR_INDEX ] 		= lpDevData->address;
		lpTxBuf[ MB_PACKET_FUNC_CODE_INDEX ] 	= func;
		lpTxBuf[ 2 ] = lpRxBuf[ 2 ];
		lpTxBuf[ 3 ] = lpRxBuf[ 3 ];
		lpTxBuf[ 4 ] = lpRxBuf[ 4 ];
		lpTxBuf[ 5 ] = lpRxBuf[ 5 ];
		
		break;
#endif // MB_FUNC_WRITE_MULTI_REGS > 0
		
#if MB_FUNC_READ_FILE_RECORD > 0
	//--------------------------------------------------------------------------
	// Read File Record support
	//--------------------------------------------------------------------------
	case MB_CMD_READ_FILE_RECORD: 		/* = 0x14 - Чтение файла 								*/
		#error "Modbus Fuction code 0x14 not supported"
		break;
#endif // MB_FUNC_READ_FILE_RECORD > 0
		
#if MB_FUNC_WRITE_FILE_RECORD > 0
	//--------------------------------------------------------------------------
	// Write File Record support
	//--------------------------------------------------------------------------
	case MB_CMD_WRITE_FILE_RECORD: 		/* = 0x15 - Запись файла 								*/
		#error "Modbus Fuction code 0x15 not supported"
		break;
#endif // MB_FUNC_WRITE_FILE_RECORD > 0
		
#if MB_FUNC_READ_DEVICE_ID > 0
	case MB_CMD_READ_DEVICE_ID: 		/* = 0x2B - Чтение идентификатора устройства 			*/
		#error "Modbus Fuction code 0x2B not supported"
		break;
#endif // MB_FUNC_READ_DEVICE_ID > 0
	default:
		mb_slave_send_error( lpDevData, func, MB_EXC_ILLEGAL_FUNCTION );
		txSize = 0;
		break;
	}
	
	if( (result != MB_EXC_NONE) || (txSize == 0) )
		return MB_ERR_NONE;
	
	// Считаем CRC
	WriteCrc16( lpTxBuf, txSize );
	
	// Запускаем передачу
	mb_slave_start_tx( lpDevData, txSize );
	
	return MB_ERR_NONE;
}

MbErrorCodes_t mb_slave_recive( MbDeviceData_t *lpDevData, uint8_t data ) {
	if(lpDevData == (MbDeviceData_t *)NULL)
		return MB_ERR_INVALID_DATA;
	//
	if( !lpDevData->isInit )
		return MB_ERR_NOINIT;
	
	if(lpDevData->status & (MB_SLAVE_STATUS_PARSED_START | MB_SLAVE_STATUS_PARSE_ENABLE) ) {
		// Текущий пакет еще не обработан
		return MB_ERR_NONE;
	}
	
	if( lpDevData->rxIndex >= lpDevData->rxBufSize ) {
		// Переполнение буфера
		lpDevData->rxIndex = 0;
		return MB_ERR_RX_BUF_OVF;
	}
	
	EnterCritSection();
	
	if( (lpDevData->status & MB_SLAVE_STATUS_TIMER_ENABLE) == 0 ) {
		(*lpDevData->lpTimerEnablePtr)();
		lpDevData->status |= MB_SLAVE_STATUS_TIMER_ENABLE;
	}
	else
		(*lpDevData->lpTimerResetPtr)( );
	
	// забираем данные
	*(lpDevData->lpRxBuf + lpDevData->rxIndex) = data;
	lpDevData->rxIndex++;
	
	ExitCritSection();
	
	(*lpDevData->lpTimerResetPtr)( );
	
	return MB_ERR_NONE;
}

MbErrorCodes_t mb_slave_timer_expired( MbDeviceData_t *lpDevData ) {
	if(lpDevData == (MbDeviceData_t *)NULL)
		return MB_ERR_INVALID_DATA;
	
	if( !lpDevData->isInit )
		return MB_ERR_NOINIT;
	
	if( (lpDevData->status & MB_SLAVE_STATUS_TIMER_ENABLE) == 0 ) {
		// Таймер не был включен - выходим
		return MB_ERR_NONE;
	}
	
	if( lpDevData->status & MB_SLAVE_STATUS_PARSED_START ) {
		// Обработка пакета запущена - выходим
		return MB_ERR_NONE;
	}
	
	EnterCritSection();
	{
		// Выключаем таймер
		(*lpDevData->lpTimerDisablePtr)(  );
		lpDevData->status &= ~MB_SLAVE_STATUS_TIMER_ENABLE;
		
		// Разрешаем обработку пакета
		lpDevData->status |= MB_SLAVE_STATUS_PARSE_ENABLE;
	}
	ExitCritSection();
	
	if( lpDevData->lpMbRiseEvent != NULL )
		(*lpDevData->lpMbRiseEvent)( MB_EVT_PARSE );
	
	return MB_ERR_NOINIT;
}

MbErrorCodes_t mb_slave_start_packet( MbDeviceData_t *lpDevData ) {
	if(lpDevData == (MbDeviceData_t *)NULL)
		return MB_ERR_INVALID_DATA;
	
	if( !lpDevData->isInit )
		return MB_ERR_NOINIT;
	
	EnterCritSection();
	{
		lpDevData->rxIndex = 0;
		lpDevData->status &= ~( MB_SLAVE_PARSE_COMPLETE_CLR_MASK );
		lpDevData->state = MB_SLAVE_STATE_IDLE;
	}
	ExitCritSection();
	
	if( lpDevData->lpSerialEnablePtr != NULL )
		(*lpDevData->lpSerialEnablePtr)( true, false );
	
	return MB_ERR_NONE;
}

MbErrorCodes_t mb_slave_start_tx( MbDeviceData_t *lpDevData, unsigned int size ) {
	if(lpDevData == (MbDeviceData_t *)NULL)
		return MB_ERR_INVALID_DATA;
	
	if( !lpDevData->isInit )
		return MB_ERR_NOINIT;
	
	EnterCritSection();
	{
		lpDevData->txCount = size;
		lpDevData->txIndex = 0;
		lpDevData->status |= MB_SLAVE_STATUS_TX_ENABLE;
		lpDevData->state = MB_SLAVE_STATE_TX;
	}
	ExitCritSection();
	
	if( lpDevData->lpSerialEnablePtr != NULL )
		(*lpDevData->lpSerialEnablePtr)( false, true );
	
	if( lpDevData->lpMbRiseEvent != NULL )
		(*lpDevData->lpMbRiseEvent)( MB_EVT_TX );
	
	return MB_ERR_NONE;
}

//------------------------------------------------------------------------------
// Private




void mb_slave_send_error( MbDeviceData_t *lpDevData, uint8_t func, uint8_t errCode ) {
	if(lpDevData == (MbDeviceData_t *)NULL)
		return;
	
	if( !lpDevData->isInit )
		return;
	
	uint8_t *ptr = lpDevData->lpTxBuf;
	uint32_t size = MB_ERROR_PACKET_SIZE;
	
	ptr[ MB_PACKET_ADDR_INDEX ] 		= lpDevData->address;
	ptr[ MB_PACKET_FUNC_CODE_INDEX ] 	= func | 0x80;
	ptr[ MB_PACKET_ERROR_CODE_INDEX ] 	= errCode;
	
	// Считаем CRC
	WriteCrc16( ptr, size );
	
	// Запускаем передачу
	mb_slave_start_tx( lpDevData, size );
}
