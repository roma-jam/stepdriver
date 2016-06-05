/*
 * http_server.h
 *
 *  Created on: 27 но€б. 2014 г.
 *      Author: r.leonov
 */

#ifndef HTTP_SERVER_H_
#define HTTP_SERVER_H_

#include <cstring>
#include "kl_lib_f100.h"
#include "wind_cmd.h"
#include "config.h"

#define HTTP_SERVER_THD_SZ          1024
#define HTTP_SERVER_MAX_LINE_SZ     401
#define HTTP_REQUEST_SIZE			1000 // 1kbyte bye one transaction

#define AT_COMMAND_DELIMETR         ":\r\n"
#define AT_WIND_CMD                 "+WIND"
#define AT_OK                       "OK"
#define AT_GET                      "GET"
#define AT_EMPTYLINE				"\r\n"
class server_t {
private:
    Thread *PThd;
    char* pInnerLine;
    uint32_t LineLength;
    void GetWindID(uint8_t *P) {
        char *S = strtok(NULL, AT_COMMAND_DELIMETR);
        *P = strtoll(S, &S, 10);
    }
public:
    bool Started;
    char Line[HTTP_SERVER_MAX_LINE_SZ];
    char *CurrData;
    void Init();
    void Task();
    void LineHandle();
    void WindCommand();
    void HostCommand();
    void CommandSuccess();

    void ActionReply();
    void GetRequest();
    void OpenSocket();
    void CloseSocket();
    void SendHttpHeader(uint32_t ContentLength);

    void Sleep()
    {
        chSysLock();
        chSchGoSleepS(THD_STATE_SUSPENDED);
        chSysUnlock();
    }

    void Wakeup()
    {
        if(PThd != nullptr && PThd->p_state == THD_STATE_SUSPENDED)
        {
            PThd->p_u.rdymsg = RDY_OK;
            chSchReadyI(PThd);
        }
    }
};

extern server_t HttpServer;
#endif /* HTTP_SERVER_H_ */
