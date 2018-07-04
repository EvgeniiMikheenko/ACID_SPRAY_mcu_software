//==============================================================================
//
//==============================================================================

#ifndef _MB_SLAVE_H_
#define _MB_SLAVE_H_

#include <mb.h>

//#include <modbus_config.h>
#include <crc16.h>

typedef enum {
	  MB_SLAVE_STATE_IDLE = 0x00
	//, MB_SLAVE_STATE_PARSE
	, MB_SLAVE_STATE_TX
} MbSlaveState_t;

typedef struct {
	uint8_t address;
	uint8_t mode;
	
	bool isInit;
	uint32_t status;
	MbSlaveState_t state;
	
	uint32_t baudrate;
	uint8_t stopBits;
	uint8_t parity;
	
	uint8_t *lpRxBuf;
	unsigned int rxBufSize;
	unsigned int rxIndex;
	
	uint8_t *lpTxBuf;
	unsigned int txBufSize;
	unsigned int txIndex;
	unsigned int txCount;
	
	pxMBSerialPortInit lpSerialInitPtr;
	pxMBSerialPortEnable lpSerialEnablePtr;
	
	pxMBTimerInit lpTimerInitPtr;
	pxMBTimerEneble lpTimerEnablePtr;
	pxMBTimerDisable lpTimerDisablePtr;
	pxMBTimerReset lpTimerResetPtr;
	
	pxMbEvent lpMbRiseEvent;
	
	pxMbReadCoils 			lpExecuteReadCoils;
	pxMbReadDescreteInputs 	lpExecuteReadDescreteInputs;
	pxMbReadRegisters 		lpExecuteReadRegisters;
	pxMbReadInputRegisters 	lpExecuteReadInputRegisters;
	pxMbWriteRegisters 		lpExecuteWriteRegisters;
	pxMbReadExcStatus 		lpExecuteReadExcStatus;
	pxMbWriteMultiCoils 	lpExecuteWriteMultiCoils;
	pxMbReadFileRecord 		lpExecuteReadFileRecord;
	pxMbWriteFileRecord 	lpExecuteWriteFileRecord;		
} MbDeviceData_t;

#define MB_SLAVE_STATUS_PARSE_ENABLE			( 1 << 0 ) /* Разрешение обработки текущего пакета */
#define MB_SLAVE_STATUS_PARSED_START			( 1 << 1 ) /* Бит обработки текущего пакета */
#define MB_SLAVE_STATUS_TIMER_ENABLE			( 1 << 2 ) /* Бит, указывающий, что таймер включен  */
#define MB_SLAVE_STATUS_TX_ENABLE				( 1 << 3 ) /* Бит разрешения передачи */

#define MB_SLAVE_PARSE_COMPLETE_CLR_MASK		( MB_SLAVE_STATUS_TX_ENABLE | MB_SLAVE_STATUS_TIMER_ENABLE | MB_SLAVE_STATUS_PARSED_START | MB_SLAVE_STATUS_PARSE_ENABLE )  

#ifdef __cplusplus
	extern "C" {
#endif
		
MbErrorCodes_t mb_slave_init( MbDeviceData_t *lpDevData );

MbErrorCodes_t mb_slave_parse_packet( MbDeviceData_t *lpDevData );

MbErrorCodes_t mb_slave_recive( MbDeviceData_t *lpDevData, uint8_t data );

MbErrorCodes_t mb_slave_timer_expired( MbDeviceData_t *lpDevData );

MbErrorCodes_t mb_slave_start_packet( MbDeviceData_t *lpDevData );

MbErrorCodes_t mb_slave_start_tx( MbDeviceData_t *lpDevData, unsigned int size );

#ifdef __cplusplus
}
#endif

#endif // _MB_SLAVE_H_
