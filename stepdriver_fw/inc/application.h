/*
 * application.h
 *
 *  Created on: 19 февр. 2014 г.
 *      Author: r.leonov
 */

#ifndef APPLICATION_H_
#define APPLICATION_H_


#include "kl_lib_f100.h"
#include "vcp.h"
#include <string.h>
#include "events.h"

# if 1 // Uart Command Codes. See https://docs.google.com/document/d/14pGuFv6KsG5tB4OmI0qc9f37cWdUVqZpTvvds2y46VY/edit
#define CMD_PING            0x01
#define CMD_SET_ID          0x10
#define CMD_GET_ID          0x11
#define CMD_SET_DOSETOP     0x20
#define CMD_GET_DOSETOP     0x21
#define CMD_PILL_STATE      0x30
#define CMD_PILL_WRITE      0x31
#define CMD_PILL_READ       0x32
#define CMD_PILL_WRITEALL   0x33
#define CMD_DOSE_GET        0x60
#define CMD_DOSE_SET        0x61

#define CMD_SET_TIME        0x71
#define CMD_GET_MESH_INFO   0x72

#define RPL_ACK             0x90    // Acknowledge
#define RPL_GET_ID          0xA1
#define RPL_GET_DOSETOP     0xB1
#define RPL_PILL_STATE      0xC0
#define RPL_PILL_WRITE      0xC1
#define RPL_PILL_READ       0xC2
#define RPL_DOSE_GET        0xE0

#define RPL_SET_TIME        0xD1
#define RPL_GET_MESH_INFO   0xD2

#define UART_RPL_BUF_SZ     36
#endif

class App_t {
private:
    uint8_t UartRplBuf[UART_RPL_BUF_SZ];

public:
    Thread *PThd;
    uint32_t GlideTrackMaxStep;
    uint32_t StepSize, TimeDelay;
    void Init();

    // Events
    void OnWiFiCmd(char* Request);
    void OnUartCmd(Cmd_t *PCmd);
    void SendEvent(eventmask_t mask)  {
        chEvtSignal(PThd, mask);
    }
    void SendEventI(eventmask_t mask) {
        chSysLock();
        chEvtSignalI(PThd, mask);
        chSysUnlock();
    }
};

extern App_t App;

#endif /* APPLICATION_H_ */
