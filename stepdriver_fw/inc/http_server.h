/*
 * http_server.h
 *
 *  Created on: 27 но€б. 2014 г.
 *      Author: r.leonov
 */

#ifndef HTTP_SERVER_H_
#define HTTP_SERVER_H_

#include "kl_lib_f100.h"

#define HTTP_SERVER_THD_SZ          512
#define HTTP_SERVER_MAX_LINE_SZ     401

class server_t {
private:
    Thread *PThd;
    char Line[HTTP_SERVER_MAX_LINE_SZ];
    uint32_t LineLength;
public:
    void Init();
    void Task();
    void LineHandle();

    void Sleep() {
        chSysLock();
        chSchGoSleepS(THD_STATE_SUSPENDED);
        chSysUnlock();
    }
    void Wakeup() { if(PThd != nullptr) chSchReadyI(PThd); }
};

extern server_t HttpServer;
#endif /* HTTP_SERVER_H_ */
