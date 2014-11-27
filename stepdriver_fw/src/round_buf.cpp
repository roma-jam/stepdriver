/*
 * round_buf.cpp
 *
 *  Created on: 22 но€б. 2014 г.
 *      Author: Roma Jam
 */


#include "round_buf.h"
#include "wifi_driver.h"


char* round_buf_t::GetNextLine() {
	char *StrPtr = NULL;
	StrPtr = (char*)pToRead;
	uint32_t LineLength=0;
	while(*pToRead++ != WIFI_STR_CR) {
		LineLength++;
		Uart.Printf("%c", *pToRead);
		if(pToRead >= (CircBuf + WIFI_RX_BUF_SZ)) pToRead = CircBuf;
	}
	*pToRead = '\0';
	pToRead += LineLength;
	FilledCount -= LineLength;
	return StrPtr;
}
