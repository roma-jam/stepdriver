/*
 * vcp.h
 *
 *  Created on: 03 ���. 2013 �.
 *      Author: r.leonov
 */

#ifndef VCP_H_
#define VCP_H_


#include <cstring>
#include "kl_lib_f100.h"
#include "usb_f103.h"
#include "stdarg.h"
#include "kl_sprintf.h"
#include "cmd_list.h"
#include "config.h"

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
#define END_OF_COMMAND  "\n\r"

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

    uint32_t GetValue(char* pData)
    {
        return strtoll(pData, &pData, APP_MOTOR_CMD_CONVERSATION_SYSTEM);
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
    uint8_t BytesToRead;
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
    uint8_t ReadBytes(uint8_t *ToPtr, uint32_t *PSize) {
        msg_t r = chIQReadTimeout(&UsbOutQueue, ToPtr, 20, TIME_INFINITE);
        if(r > 0) {
            Uart.Printf("Recieve %u\r", (uint8_t)r);
            *PSize = (uint32_t)r;
            return OK;
        }
        else return FAILURE;
    }
    void IOutTask();
    void Printf(const char *format, ...);

    Cmd_t ICmd[2], *PCmdWrite = &ICmd[0], *PCmdRead = &ICmd[1];
    void CompleteCmd();

    void CmdRpl(uint8_t ErrCode);
    void CmdRplData(uint32_t Data);
};

extern Vcp_t Vcp;
#endif /* VCP_H_ */
