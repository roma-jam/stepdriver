/*
 * server.h
 *
 *  Created on: 27 но€б. 2014 г.
 *      Author: Roma Jam
 */

#ifndef INC_SERVER_H_
#define INC_SERVER_H_

#include "kl_lib_f100.h"
#include <cstring>

#define SERVER_THD_SZ		2048
#define MAX_LINE_LENGTH		1024
#define SERVER_READ_TIMEOUT	9


#define LINE_DELIMITERS      "\r\n"

class server_t {
private:
	char LineBuf[MAX_LINE_LENGTH];
	char *PLine;
	Thread *PThd;
	InputQueue *PQueue;
	void NewLine() { PLine = LineBuf; LineBuf[LenToRead++] = 0; }
    char* GetNextLine() {
        char *RS = strtok(PLine, LINE_DELIMITERS);
        PLine = NULL;
        return RS;
    }
public:
	uint32_t LenToRead;
	void Init();
	void Task();
	void WakeUpI() {
		if(PThd != nullptr) chSchReadyI(PThd);
	}
	void Sleep() {
        chSysLock();
        chSchGoSleepS(THD_STATE_SUSPENDED);
        chSysUnlock();
	}
};

extern server_t HttpServer;

#endif /* INC_SERVER_H_ */
