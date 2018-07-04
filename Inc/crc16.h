//=============================================================================
//								crc16.h
//=============================================================================

#ifndef _CRC16_H_
#define _CRC16_H_

#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>

//typedef struct {
//	uint16_t crc;
//	uint32_t counter;
//} Crc16Info_t, *lpCrc16Info_t;

uint16_t CalcCrc16(uint8_t *lpBuf, uint32_t bufSize);

uint16_t CalcCrc16Next(uint16_t oldCrc, uint16_t data, bool isInit);

uint16_t WriteCrc16(uint8_t *lpBuf, uint32_t bufSize);




uint16_t CalcCrc16(uint8_t *nData, uint32_t wLength);

bool TestCrc16(uint8_t *lpBuf, uint32_t bufSize);


#endif // _CRC16_H_
