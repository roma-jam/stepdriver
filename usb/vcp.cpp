/*
 * vcp.cpp
 *
 *  Created on: 03 дек. 2013 г.
 *      Author: r.leonov
 */



#include "vcp.h"
#include "usb_f103.h"
#include "led.h"
#include "application.h"

Vcp_t Vcp;
// ==== Line Coding structure ====
struct Linecoding_t {
  uint8_t dwDTERate[4];
  uint8_t bCharFormat;
  uint8_t bParityType;
  uint8_t bDataBits;
} __attribute__((__packed__));

static Linecoding_t LineCoding = {
        {0x00, 0xC2, 0x01, 0x00},
        LC_STOP_1, LC_PARITY_NONE, 8
};

static WORKING_AREA(waVcpThread, 128);
__attribute__ ((__noreturn__))
static void VcpThread(void *arg) {
    chRegSetThreadName("Vcp");
    while(1) {
        chSysLock();
        chSchGoSleepS(THD_STATE_SUSPENDED);
        chSysUnlock();
        Vcp.IOutTask();
    }
}


void Vcp_t::IOutTask() {
	uint8_t Byte;
	if(GetByte(&Byte, 20) == OK) IProcessByte(Byte);
}

static inline uint8_t TryConvertToDigit(uint8_t b, uint8_t *p) {
    if((b >= '0') and (b <= '9')) {
        *p = b - '0';
        return OK;
    }
    else if((b >= 'A') and (b <= 'F')) {
        *p = 0x0A + b - 'A';
        return OK;
    }
    else return FAILURE;
}
static inline bool IsDelimiter(uint8_t b) { return (b == ','); }
static inline bool IsEnd(uint8_t b) { return (b == '\r') or (b == '\n'); }

void Vcp_t::IProcessByte(uint8_t b) {
    uint8_t digit=0;
    if(b == '#') RxState = rsCmdCode1; // If # is received anywhere, start again
    else switch(RxState) {
        case rsCmdCode1:
            if(TryConvertToDigit(b, &digit) == OK) {
                CmdCode = digit << 4;
                RxState = rsCmdCode2;
            }
            else IResetCmd();
            break;

        case rsCmdCode2:
            if(TryConvertToDigit(b, &digit) == OK) {
                CmdCode |= digit;
                RxState = rsData1;
            }
            else IResetCmd();
            break;

        case rsData1:
            if(TryConvertToDigit(b, &digit) == OK) {
                *PCmdWrite = digit << 4;
                RxState = rsData2;
            }
            else if(IsDelimiter(b)) return; // skip delimiters
            else if(IsEnd(b)) {
                App.OnUartCmd(CmdCode, CmdData, (PCmdWrite - CmdData));
                IResetCmd();
            }
            else IResetCmd();
            break;

        case rsData2:
            if(TryConvertToDigit(b, &digit) == OK) {
                *PCmdWrite |= digit;
                RxState = rsData1;  // Prepare to rx next byte
                if(PCmdWrite < (CmdData + (VCP_CMDDATA_SZ-1))) PCmdWrite++;
            }
            else IResetCmd(); // Delimiters and End symbols are not allowed in the middle of byte
            break;

        default: break;
    } // switch
}



#if 1 // ================== USB events =================
static void OnUsbReady() {
    Uart.Printf("Ready\r");
    Usb.PEpBulkOut->StartOutTransaction();
}

static void SetLineCoding() {
//    Uart.Printf("SetLineCoding\r");
}
static void SetCtrlLineState() {
//    uint16_t w = Usb.SetupReq.wValue;
//    if(w & 0x0001) Uart.Printf("DTR 1\r");
//    else Uart.Printf("DTR 0\r");
//    if(w & 0x0002) Uart.Printf("RTS 1\r");
//    else Uart.Printf("RTS 0\r");
}
#endif

EpState_t NonStandardControlRequestHandler(uint8_t **PPtr, uint32_t *PLen) {
//    Uart.Printf("NonStandard Request\r");
    switch(Usb.SetupReq.bRequest) {
        case SET_LINE_CODING:
//            Uart.Printf("SET_LINE_CODING\r");
            *PPtr = (uint8_t*)&LineCoding;  // Do not use length in setup pkt
            *PLen = sizeof(LineCoding);
            Usb.Events.OnTransactionEnd[0] = SetLineCoding;
            return esOutData;
            break;

        case GET_LINE_CODING:
//            Uart.Printf("GET_LINE_CODING\r");
            *PPtr = (uint8_t*)&LineCoding;
            *PLen = sizeof(LineCoding);
            return esInData;
            break;

        case SET_CONTROL_LINE_STATE:
//            Uart.Printf("SET_CTRL_LINE_STATE\r");
            Led.Blink();
            SetCtrlLineState();
            return esOutStatus;
            break;

        default:
            Uart.Printf("Error\r");
            break;
    } // switch
    return esError;
}

void Vcp_t::Init() {
    Usb.Events.OnCtrlPkt = NonStandardControlRequestHandler;
    Usb.Events.OnReady = OnUsbReady;

    // Queues
    chIQInit(&UsbOutQueue, OutQBuf, CDC_OUTQ_SZ, NULL, NULL);
    Usb.PEpBulkOut->POutQueue = &UsbOutQueue;
    chOQInit(&UsbInQueue, InQBuf, CDC_INQ_SZ, NULL, NULL);
    // Start reception
//    Usb.PEpBulkOut->StartOutTransaction();
    PThread = chThdCreateStatic(waVcpThread, sizeof(waVcpThread), NORMALPRIO, (tfunc_t)VcpThread, NULL);
}

void Vcp_t::Printf(const char *format, ...) {
    va_list args;
    va_start(args, format);
    PrintToQueue(&UsbInQueue, format, args);
    va_end(args);
    // Start transmission
    Usb.PEpBulkIn->WriteFromQueue(&UsbInQueue);
}


