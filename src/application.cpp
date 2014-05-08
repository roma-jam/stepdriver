/*
 * application.cpp
 *
 *  Created on: 19 февр. 2014 г.
 *      Author: r.leonov
 */

#include "application.h"
#include "vcp.h"

#if 1 // ==== Init's ====
App_t App;

void AppTask();

static WORKING_AREA(waAppThread, 128);
__attribute__ ((__noreturn__))
static void AppThread(void *arg) {
    chRegSetThreadName("Motor");
    while(1) AppTask();
}
#endif


void AppTask() {
    chThdSleepMilliseconds(999);
}

#if 1 // ======================= Command processing ============================
void App_t::OnUartCmd(Cmd_t *PCmd) {
    Uart.Printf("%S\r", PCmd->S);
    char *S;
    int32_t d;
    uint8_t b;

    S = PCmd->GetNextToken();
    if(S == NULL) return;
//    Uart.Printf("%S\r", S);
    if(strcasecmp(S, "#ping") == 0) Vcp.Ack();

    else if(strcasecmp(S, "#test") == 0) {
        while((S = PCmd->GetNextToken()) != NULL) {        // Next token exists
            if(Convert::TryStrToNumber(S, &d) == OK) {  // Next token is number
                Vcp.Printf("%d ", d);
            }
        }
        Vcp.Ack();
    }


#if 0 // ==== ID ====
    else if(strcasecmp(S, "#SetID") == 0) {
        if((S = PCmd->GetNextToken()) != NULL) {        // Next token exists
            if(Convert::TryStrToNumber(S, &d) == OK) {  // Next token is number
                b = ISetID(d);
                Vcp.Ack(b);
            }
            else Vcp.Ack(CMD_ERROR);
        }
        else Vcp.Ack(CMD_ERROR);
    }

    else if(strcasecmp(S, "#GetID") == 0) Uart.Printf("#ID %u\r", ID);
#endif
//    uint8_t b, b2;
//    uint32_t *p32 __attribute__((unused));  // May be unused in some cofigurations

//    switch(CmdCode) {
//        case CMD_PING: Vcp.Ack(OK); break;

#if 0 // ==== ID ====
        case CMD_SET_ID:
            break;
        case CMD_GET_ID:
            break;
#endif

#if 0 // ==== Pills ====
        case CMD_PILL_STATE:
            b = PData[0];   // Pill address
            if(b <= 7) {
                UartRplBuf[0] = b;
                UartRplBuf[1] = PillMgr.CheckIfConnected(PILL_I2C_ADDR);
                Vcp.Cmd(RPL_PILL_STATE, UartRplBuf, 2);
            }
            else Vcp.Ack(CMD_ERROR);
            break;
        case CMD_PILL_WRITE:
            b = PData[0];   // Pill address
            if(b <= 7) {
                UartRplBuf[0] = b;    // Pill address
                UartRplBuf[1] = PillMgr.Write(PILL_I2C_ADDR, &PData[1], Length-1); // Write result
                Vcp.Cmd(RPL_PILL_WRITE, UartRplBuf, 2);
            }
            else Vcp.Ack(CMD_ERROR);
            break;
        case CMD_PILL_READ:
            b = PData[0];           // Pill address
            b2 = PData[1];          // Data size to read
            if(b2 > (UART_RPL_BUF_SZ-2)) b2 = (UART_RPL_BUF_SZ-2);  // Check data size
            if(b <= 7) {
                UartRplBuf[0] = b;                                  // Pill address
                UartRplBuf[1] = PillMgr.Read(PILL_I2C_ADDR, &UartRplBuf[2], b2);    // Read result
                if(UartRplBuf[1] == OK) Vcp.Cmd(RPL_PILL_READ, UartRplBuf, b2+2);
                else Vcp.Cmd(RPL_PILL_READ, UartRplBuf, 2);
            }
            else Vcp.Ack(CMD_ERROR);
            break;
#endif

//        case CMD_SET_TIME:
//            break;
//
//        case CMD_GET_MESH_INFO:
//            break;
//
//        default: Vcp.Ack(CMD_ERROR); break;
//    } // switch
}
#endif


#if 1 // ============================ App class ================================
void App_t::Init() {
//    chVTSet(&ITmr, MS2ST(999), MeasureTmrCallback, nullptr);
    PThd = chThdCreateStatic(waAppThread, sizeof(waAppThread), NORMALPRIO, (tfunc_t)AppThread, NULL);
}
#endif
