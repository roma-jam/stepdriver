/*
 * vcp.h
 *
 *  Created on: 03 дек. 2013 г.
 *      Author: r.leonov
 */

#ifndef VCP_H_
#define VCP_H_


#include "kl_lib_f100.h"
#include "usb_f103.h"
#include "stdarg.h"
#include "kl_sprintf.h"
#include <cstring>
#include "cmd_list.h"

/* ==== CDC Class-Specific Control Requests ====
 * Page 30
 */

#define SET_LINE_CODING         0x20
#define GET_LINE_CODING         0x21
#define SET_CONTROL_LINE_STATE  0x22

// Line Control bit definitions
#define LC_STOP_1                       0
#define LC_STOP_1P5                     1
#define LC_STOP_2                       2
#define LC_PARITY_NONE                  0
#define LC_PARITY_ODD                   1
#define LC_PARITY_EVEN                  2
#define LC_PARITY_MARK                  3
#define LC_PARITY_SPACE                 4

#define CDC_OUTQ_SZ     256
#define CDC_INQ_SZ      256

#define DELIMITERS      " ,"

#define VCP_CMDDATA_SZ     16 // payload bytes
#define VCP_CMD_BUF_SZ     36 // payload bytes

class Cmd_t {
private:
    char *InnerS;
    void Finalize() { InnerS = S; S[Cnt++] = 0; }
    void Backspace() { if(Cnt > 0) Cnt--; }
public:
    char S[VCP_CMD_BUF_SZ];
    uint32_t Cnt;
    void PutChar(char c) { if(Cnt < VCP_CMD_BUF_SZ-1) S[Cnt++] = c; }
    bool IsEmpty() { return (Cnt == 0); }
    char* GetNextToken() {
        char *RS = strtok(InnerS, DELIMITERS);
        InnerS = NULL;
        return RS;
    }
    friend class Vcp_t;
};



class Vcp_t {
private:
    uint8_t OutQBuf[CDC_OUTQ_SZ], InQBuf[CDC_INQ_SZ];
    InputQueue UsbOutQueue; // From chibios' point of view, OUT data is input
    OutputQueue UsbInQueue; // From chibios' point of view, IN data is output
public:
    Thread *PThread;
    void Init();
    void SendByte(uint8_t AByte) {
        chOQPutTimeout(&UsbInQueue, AByte, TIME_INFINITE);
        Usb.PEpBulkIn->WriteFromQueue(&UsbInQueue);
    }
    void SendBuf(uint8_t *PBuf, uint32_t Len) {
        chOQWriteTimeout(&UsbInQueue, PBuf, Len, TIME_INFINITE);
        Usb.PEpBulkIn->WriteFromQueue(&UsbInQueue);
    }
    uint8_t GetByte(uint8_t *PByte, systime_t Timeout = TIME_INFINITE) {
        msg_t r = chIQGetTimeout(&UsbOutQueue, Timeout);
        if(r >= 0) {
            *PByte = (uint8_t)r;
            return OK;
        }
        else return FAILURE;
    }
    void IOutTask();
    void Printf(const char *format, ...);

    Cmd_t ICmd[2], *PCmdWrite = &ICmd[0], *PCmdRead = &ICmd[1];
    void CompleteCmd();
    void Ack(int Result = OK)  {
        if(Result == OK) Printf("#Ack\r\n");
        else Printf("#Err %d\r\n", Result);
    }
    void Rpl(const char* Str, uint32_t Val) {
        Printf(Str, Val);
    }
};

extern Vcp_t Vcp;
#endif /* VCP_H_ */
