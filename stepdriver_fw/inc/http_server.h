/*
 * http_server.h
 *
 *  Created on: 27 но€б. 2014 г.
 *      Author: r.leonov
 */

#ifndef HTTP_SERVER_H_
#define HTTP_SERVER_H_

#include "kl_lib_f100.h"
#include <cstring>

#define HTTP_SERVER_THD_SZ          512
#define HTTP_SERVER_MAX_LINE_SZ     401

#define AT_COMMAND_DELIMETR         " :"
#define AT_WIND_CMD                 "+WIND"
#define AT_OK                       "OK"

class server_t {
private:
    Thread *PThd;
    char Line[HTTP_SERVER_MAX_LINE_SZ];
    char* pInnerLine;
    uint32_t LineLength;
public:
    void Init();
    void Task();
    void LineHandle();
    void WindCommand();
    void CommandSuccess();

    void Sleep() {
        chSysLock();
        chSchGoSleepS(THD_STATE_SUSPENDED);
        chSysUnlock();
    }
    void Wakeup() {
        if(PThd != nullptr) {
            if(PThd->p_state == THD_STATE_SUSPENDED) {
                PThd->p_u.rdymsg = RDY_OK;
                chSchReadyI(PThd);
            }
        }
    }
};

extern server_t HttpServer;
#endif /* HTTP_SERVER_H_ */
