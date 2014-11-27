/*
 * round_buf.cpp
 *
 *  Created on: 22 но€б. 2014 г.
 *      Author: Roma Jam
 */


#include "round_buf.h"
#include "wifi_driver.h"


Rslt_t round_buf_t::GetNextLine(char *Ptr, uint32_t *PLength) {
	uint32_t LineLength = 0; uint8_t C = 0;
	while(C != WIFI_STR_LF) {
	    C = ReadByte();
	    *Ptr++ = C;
	    LineLength++;
	}
	*Ptr = '\0';
	*PLength = LineLength;
	if(LineLength > 2) return OK;
	return FAILURE;
}
