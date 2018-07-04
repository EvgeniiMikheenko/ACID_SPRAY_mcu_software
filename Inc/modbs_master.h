



#ifndef		MODBS_MASTER_H
#define		MODBS_MASTER_H



#include <stdint.h>
#include <stdbool.h>


#define TIMER3_INTERVAL_US	100
#define TIMER3_PRIORITY			8
#define BR		115200
//#define RE		LPC_GPIO0 -> FIOCLR |= 0x60000			//Rx Enable
//#define DE		LPC_GPIO0 -> FIOSET |= 0x60000			//Tx Enable
#define TxInt	0x10
#define	RxInt	0x01
#pragma anon_unions


typedef struct {
	uint16_t crc;
	uint32_t counter;
} Crc16Info_t, *lpCrc16Info_t;


typedef void (* SetBit)(bool value);
typedef void (* UartInit)(uint32_t baudrate);
typedef void (* UartSend)(uint8_t data);
typedef void (* TimerCmd)( void );
typedef uint8_t (* UartRecv)( void );
typedef uint8_t (* IRRType)( void );




typedef enum
{
	bps9600 	= 0,
	bps14400 	= 1,
	bps19200 	= 2,
	bps38400 	= 3,
	bps57600 	= 4,
	bps115200 	= 5
} usart_baudrates;




typedef struct {
	uint8_t 	address;						/* Адрес устройства */
	uint8_t		function;						/* Функция modbus */
	
	
	uint16_t 	num_regs;						/* Количество регистров */
	
	uint16_t reg_addr;						/* Адрес регистра для записи */
	uint16_t reg_data;						/* Данные для записи */
	
	uint8_t 	stopBits;						/* Количество стоп бит */
	uint32_t 	serialBaudrate;					/* Скорость порта */

	//
	uint8_t 	*rxBuf;							/* Указатель на буфер приема */
	uint16_t	*rxData;
	uint32_t	rxBufSize;						/* Размер буфера приема */
	uint32_t	rxCount;						/* Количество принятых данных */
	
	uint8_t 	*txBuf;							/* Указатель на буфер передачи */
	uint32_t	txBufSize;						/* Размер буфера передачи */
	uint32_t	txCount;						/* Количество байт для передачи */
	uint32_t	txIndex;						/* Номер переданного байта */
	//
	uint32_t	timeLeft;						/* Прошло времени с момента приема последнего байта */
	uint32_t	timeOut;						/* Значение таймаута после окончания тразакции  */
	uint32_t		timer_set;						/* Значение переода таймера в микро секундах (1) */
	//		
//	uint32_t	hostTimeOut;					/* Значение защитного таймаута связи с хостом */
//	uint32_t	hostTimeLeft;					/* Прошло времени с момента последней транзакции */	
	
	UartInit	lpUartInit;						/* Указатель на функцию инициализации порта */
	UartSend	lpUartSend;						/* Указатель на функцию отправки одного байта */
	UartRecv	lpUartRecv;						/* Указатель на функцию приема одного байта */
	//GetDeviceInfo lpGetDeviceInfo;				/* Указатель на функцию получения информации об устройстве */
	
	SetBit		lpSetTxEn;						/* Указатель на функцию вкл/выкл вывода TXEN RS485 */
	SetBit		lpSetRxEn;						/* Указатель на функцию вкл/выкл вывода RXEN RS485 */
	TimerCmd		lpTimerStart;					/* Указатель на функцию запуска таймера */
	TimerCmd		lpTimerReset;					/* Указатель на функцию сброса значения и остоновки таймера */
	IRRType		GetIntSourse;					/* Источник прервывания порта прием или передача   Tx -- 010   Rx -- 101 */
	
	
	int			passwordRegNum;					/* номер начального регистра для пароля */
	int			deviceInfoSizeRegNum;
	int			deviceInfoRegNum;				/* номер начального регистра для информации об устройстве */
	
	const char* password;
	int			passwordLen;
	

	
	uint32_t	errorCount;						/* количество ошибок протокола*/
	
	uint16_t	*lpRegisters;					/* Указатель на буфер регистров */
	uint16_t	registersCount;					/* количество регистров */
	
	
	 
	union {
		uint32_t value;
		struct {
			unsigned enableTimer 			: 1;	/* Флаг активности таймера */
			unsigned enableParseRxData 		: 1;	/* Флаг разрешения обработки принятого пакета */
			unsigned isParsedStart 			: 1;	/* Флаг, указывающий, что обработка текущего пакета начата */
			unsigned isRxDataMain 			: 1;	/* Флаг готовности принятых данных */
			unsigned isHostTimeout 			: 1;	/* Флаг таймаута связи с хостом */
			unsigned isConfigChangeLock 	: 1;	/* Флаг блокировки изменения конфигурации */
			unsigned eepromNeedUpdate 		: 1;	/* Флаг необходимости обновления энергонезависимых данных */
			unsigned isPasswordValid		: 1;   	/* Флаг правильного пароля */
			unsigned isPacket						: 1;		/*собран, готов к отправке */
			unsigned isBeginValid				: 1;		/* первые 2 байта принятого пакета верны */
			unsigned isCRCValid					: 1;
			unsigned isRecvPacket				: 1;
			unsigned isBusy							:	1;		/* Занят, попробуйте позже */	
			unsigned isEndOfPacket			:	1;		/*Конец таймаута приема, конец пакета*/
			unsigned bridge_delay_flag	: 1;
			
		};
	} Flags;
	
} MbParam_t, *lpMbParam_t;







bool  Slave_request(lpMbParam_t mbParam, uint8_t addr, uint8_t func, uint16_t reg_start, uint8_t reg_num);
bool Slave_write(lpMbParam_t mbParam, uint8_t addr, uint8_t func, uint16_t reg_addr, uint16_t data);
void mbInit(lpMbParam_t mbParam);
uint32_t mb_convert_baudrate( uint16_t value );
void mb_stop_tx(lpMbParam_t Param);
void mb_start_tx(lpMbParam_t lpParam, uint32_t size);
void mb_rx_func(lpMbParam_t lpParam, uint8_t data, bool frameError, bool overflowErroe);
void mb_rx_tx_interrupt( lpMbParam_t mbParam );
void mb_tx_reset( lpMbParam_t mbParam);
void mb_rx_reset( lpMbParam_t mbParam );
void mb_start_timer( lpMbParam_t mbParam);
void mb_reset_timer( lpMbParam_t mbParam);
void mb_Timer_Int( lpMbParam_t mbParam );
void mbParsePacket( lpMbParam_t mbParam );



#endif				//MODBS_MASTER_H
