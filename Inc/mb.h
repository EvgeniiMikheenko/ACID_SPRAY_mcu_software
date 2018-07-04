//==============================================================================
//
//==============================================================================

#ifndef _MB_H_
#define _MB_H_

#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
//#include "jornal_registers.h"
//#include <mb_slave.h>

//#include <modbus_config.h>

#define MAX_REPEATE_TRANSMIT_COUNT 			5
#define MODBUS_REGS_IOCOUNT_MAX				125
// 											(addr  	+ func  + startAddr + count + (MODBUS_REGS_IOCOUNT_MAX * 2))
#define MOBUS_MAX_PACKET_SIZE				(  1 	+   1 	+    2 		+   2 	+ (MODBUS_REGS_IOCOUNT_MAX * 2))

#define MB_DEVICE_ADDRESS_BROADCAST			0
#define MB_DEVICE_ADDRESS_MIN				1
#define MB_DEVICE_ADDRESS_MAX				247

#define MB_PACKET_SIZE_MIN					4

#define MB_PACKET_ADDR_INDEX				0
#define MB_PACKET_FUNC_CODE_INDEX			1
#define MB_PACKET_ERROR_CODE_INDEX			2

#define MB_ERROR_PACKET_SIZE				5

enum {
	  MB_CMD_READ_COILS 				= 0x01 // ������ ������� �������
	, MB_CMD_READ_DINPUTS 				= 0x02 // ������ ��������� ���������� ������
	, MB_CMD_READ_HOLDING_REGS 			= 0x03 // ������ ����������� ���������
	, MB_CMD_READ_INPUT_REGS 			= 0x04 // ������ ����������� ������� ���������
	, MB_CMD_WRITE_SINGLE_COIL 			= 0x05 // ��������� ���������� ������ � ON ��� OFF
	, MB_CMD_WRITE_SINGLE_REG 			= 0x06 // ������ � ��������� �������
	, MB_CMD_READ_EXCEPTION_STATUS		= 0x07 // ������ ������� ����������
	, MB_CMD_WRITE_MULTI_COILS 			= 0x0F // ��������� ��������� ������� � ON ��� OFF
	, MB_CMD_WRITE_MULTI_REGS 			= 0x10 // ������ � ��������� ���������
	, MB_CMD_READ_FILE_RECORD			= 0x14 // ������ �����
	, MB_CMD_WRITE_FILE_RECORD			= 0x15 // ������ �����
	, MB_CMD_READ_DEVICE_ID				= 0x2B // ������ �������������� ����������
};

typedef enum {
	  MB_EXC_NONE						= 0x00
	, MB_EXC_ILLEGAL_FUNCTION 			= 0x01
	, MB_EXC_ILLEGAL_DATA_ADDR			= 0x02
	, MB_EXC_ILLEGAL_DATA_VALUE			= 0x03
	, MB_EXC_SERVER_DEVICE_FAILURE		= 0x04
	, MB_EXC_ACK						= 0x05
	, MB_EXC_SERVER_DEVICE_BUSY			= 0x06
	, MB_EXC_MEMORY_PARITY_ERR			= 0x08
	, MB_EXC_GATEWAY_PATH_UNAVAILABLE	= 0x0A
	, MB_EXC_GATEWAY_TARGET_DEV_FAILED	= 0x0B
} MbExceptionCodes_t;

typedef enum {
	  MB_ERR_NONE
	, MB_ERR_INVALID_DATA
	, MB_ERR_NOINIT
	, MB_ERR_INVALID_FUNCTION_PTR
	, MB_ERR_SERIAL_INIT
	, MB_ERR_INVALID_DEVICE_ADDR
	, MB_ERR_TIMER_INIT
	, MB_ERR_PACKET_SIZE_MIN
	, MB_ERR_PACKET_CRC
	, MB_ERR_RX_BUF_OVF
} MbErrorCodes_t;

typedef enum {
	  MB_EVT_NONE
	, MB_EVT_PARSE
	, MB_EVT_TX
	, MB_EVT_ADDR_INVALID
} MBEvent_t;


#define USM_LOCATION_DESCRIPTION_STRING_SIZE			(64)
#define USM_CONFIG_DUMMY1_COUNT							(16)
#define USM_DUMMY1_COUNT								(10)
#define USM_DUMMY2_COUNT								(10)
#define USM_DUMMY3_COUNT								(10)
#define USM_DUMMY4_COUNT								(10)
#define USM_HW_VERSION_COUNT							(4)
#define USM_MD5_REGISTERS_COUNT							(8)


	
#pragma pack(push, 1)
//typedef struct {
//	
//	float temperature;
//	float humidity;
//	float pressure;
//	float wind_speed;
//	float wind_direction;
//	uint16_t year;
//	uint16_t mounth;
//	uint16_t date;
//	uint16_t hour;
//	uint16_t minute;
//	uint16_t second;
//	
//	uint16_t dummy_1[64];
//	
//	float temperature_average;
//	float humidity_average;
//	float pressure_average;
//	float wind_speed_average;
//	float wind_direction_average;
//	uint32_t time_of_averaging;
//	
//	uint16_t dummy_2[64];
//	
//	float time_first_avrg;
//	float time_first_left;
//	float time_second_avrg;
//	float time_second_left;
//	
//	uint16_t dummy_3[64];
//	
//	uint32_t v_bat;
//	uint32_t soft_version;
//	uint32_t hard_version[4];
//	
//	
//	uint16_t read_flag;				
//	////// cfg
//	uint16_t write_reg;		// 1 - ��������� ���������, 0 - ��������� ���������		3 - ���������� �� "0" �� �����, 4 - ��������� ������� �� ���������
//	uint16_t dev_addr;
//	uint16_t br_speed;
//	uint16_t stop_bit;
//	uint16_t time_of_avrg;
//	uint16_t nord_direction;
//	uint16_t first_length;
//	uint16_t second_length;
//	uint32_t gps_latitude;
//	uint32_t gps_longitude;
//	uint16_t time_error_first;
//	uint16_t time_error_second;
//	uint16_t crc;

//} mb_Regs;






typedef struct {
	uint16_t Year;
	uint16_t Month;
	uint16_t Day;
} DateRegisters_t, *lpDateRegisters_t;

typedef struct {
	uint16_t Hours;
	uint16_t Minutes;
	uint16_t Seconds;
} TimeRegisters_t, *lpTimeRegisters_t;

typedef struct {
	DateRegisters_t Date;
	TimeRegisters_t Time;
} DateTimeRegisters_t, *lpDateTimeRegisters_t;



typedef struct {
    float Red_DNG;
    float Green_DNG;
    float Blue_DNG;
    float White_DNG;
    float Ir_DNG;
    float Drop_DNG;
    float Temperature;  
    uint16_t  Flags; 
    uint16_t dummy1;
} DangerRegisters_t;

typedef struct  {
    float Red_time;
    float Green_time;
    float Blue_time;
    float White_time;
    float Ir_time;
    float Drop_time;
    
} DangerTimeRegs_t;



//typedef struct {
//	uint32_t ReadStartAddr;
//	uint32_t ReadSize;				//max 110
//	uint32_t ReadComplete;
//	uint16_t Flags;						//������� - ����� �������, ������� - ������ ���������, ringbuffer
//} JornalData_t, *lpJornalData_t;






typedef struct {
    uint16_t write_reg;		// time, drop/color, heater, save, reset time
    DateTimeRegisters_t DT;
    uint16_t Led_Pump_Flags;
    uint16_t Pump_LVL;
    uint16_t Red_LVL;
    uint16_t Green_LVL;
    uint16_t Blue_LVL;
    uint16_t White_LVL;
    uint16_t IR_LVL;
    uint16_t ColorSens_Flags;
    uint16_t Crc;
    uint16_t Red_freq;
    uint16_t Green_freq;
    uint16_t Blue_freq;
    uint16_t White_freq;
    uint16_t Ir_ADC;
    uint16_t Drop_ADC;
    uint16_t Temperature;
    uint16_t StepMotor;
    DangerRegisters_t Danger_LVL;
    DangerTimeRegs_t  Time_Set;
    DangerTimeRegs_t  Time_Val;
    uint16_t dummy;
    	
} mb_Regs;






#pragma pack(pop)


// ��������� �� ������� ������� ������
typedef void (*pxMbEvent)( MBEvent_t event );

// ��������� �� ������� ������ � UART
typedef bool (*pxMBSerialPortInit)( uint32_t baudrate, uint8_t stopBits, uint8_t parity );
typedef bool (*pxMBSerialPortEnable)( bool rxEn, bool txEn );

// ��������� �� ������� ������ � ��������
typedef bool (*pxMBTimerInit) ( uint32_t us );
typedef bool (*pxMBTimerEneble)( );
typedef bool (*pxMBTimerDisable)( );
typedef bool (*pxMBTimerReset)( );

// ��������� �� ������� ������ ������
typedef MbExceptionCodes_t (*pxMbReadCoils)( uint8_t *lpDstBuf, uint16_t startAddr, uint16_t count, uint16_t *lpResultByteCount );
typedef MbExceptionCodes_t (*pxMbReadDescreteInputs)( uint8_t *lpDstBuf, uint16_t startAddr, uint16_t count, uint16_t *lpResultByteCount );
typedef MbExceptionCodes_t (*pxMbReadRegisters)( uint8_t *lpDstBuf, uint16_t startAddr, uint16_t count );
typedef MbExceptionCodes_t (*pxMbReadInputRegisters)( uint8_t *lpDstBuf, uint16_t startAddr, uint16_t count );
typedef MbExceptionCodes_t (*pxMbWriteRegisters)( uint16_t startAddr, uint16_t count, uint8_t *lpSrcBuf );
typedef MbExceptionCodes_t (*pxMbReadExcStatus)( uint8_t *lpDstBuf );
typedef MbExceptionCodes_t (*pxMbWriteMultiCoils)( uint16_t startAddr, uint16_t count, uint8_t *lpSrcBuf );
typedef MbExceptionCodes_t (*pxMbReadFileRecord)( uint16_t fileNumber, uint16_t startAddr, uint16_t count, uint8_t *lpDstBuf );
typedef MbExceptionCodes_t (*pxMbWriteFileRecord)( uint16_t fileNumber, uint16_t startAddr, uint16_t count, uint8_t *lpSrcBuf );

#endif // _MB_H_
