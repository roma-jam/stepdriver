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
}
#endif


#if 1 // ============================ App class ================================
void App_t::Init() {
//    chVTSet(&ITmr, MS2ST(999), MeasureTmrCallback, nullptr);
    PThd = chThdCreateStatic(waAppThread, sizeof(waAppThread), NORMALPRIO, (tfunc_t)AppThread, NULL);
}
#endif
