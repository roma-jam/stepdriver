/*
 * round_buf.h
 *
 *  Created on: 22 но€б. 2014 г.
 *      Author: Roma Jam
 */

#ifndef INC_ROUND_BUF_H_
#define INC_ROUND_BUF_H_

#include "stm32f10x.h"
#include <stddef.h> // for NULL

#define WIFI_RX_BUF_SZ      512    // 4 kBytes of Reception Len

class round_buf_t {
private:
	uint8_t CircBuf[WIFI_RX_BUF_SZ];
	uint32_t FilledCount, EmptyCount;
public:
	uint8_t *pToWrite, *pToRead;
	round_buf_t():
		FilledCount(0),
		EmptyCount(WIFI_RX_BUF_SZ),
	    pToWrite(CircBuf),
		pToRead(CircBuf)
	{}
	void Write(uint8_t Byte) {
		*pToWrite++ = Byte;
		if(pToWrite >= (CircBuf + WIFI_RX_BUF_SZ))
			pToWrite = CircBuf;
		FilledCount++;
	}
	char* GetNextLine();
	uint32_t GetFilledCount()  { return FilledCount; }
};

#endif /* INC_ROUND_BUF_H_ */
