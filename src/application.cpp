/*
 * application.cpp
 *
 *  Created on: 19 февр. 2014 г.
 *      Author: r.leonov
 */

#include "application.h"
#include "vcp.h"
#include "motor_ctrl.h"

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
//    int32_t d;
    S = PCmd->GetNextToken();
    if(S == NULL) return;
    if(strcasecmp(S, VCP_PING) == 0) Vcp.Ack();
    else if(strcasecmp(S, VCP_DRIVER_INIT) == 0) {
        uint8_t Rslt = Driver.Init();
        Vcp.Ack(Rslt);
    }
    else if(Driver.IsInit()) {
        if(strcasecmp(S, VCP_SET_PARAM) == 0) { //writes Value in Param
            Uart.Printf("VCP_SET_PARAM\r");
            uint8_t Rslt = VCP_RPL_OK;
            uint8_t Param;
            uint32_t Value;
            S = PCmd->GetNextToken();
            if((Param = strtoll(S, &S, 16)) != 0) {
                Uart.Printf("Param: %X\r", Param);
                S = PCmd->GetNextToken();
                if((Value = strtoll(S, &S, 16)) != 0) {
                    Uart.Printf("Value: %X\r", Value);
                    Driver.Motor[DEFAULT_ID].SetParam(Param, Value);
                } // if Value
                else Rslt = VCP_RPL_CMD_ERROR;
            } // if Param
            else Rslt = VCP_RPL_CMD_ERROR;
            Vcp.Ack(Rslt);
        }
        else if(strcasecmp(S, VCP_GET_PARAM) == 0) {
            Uart.Printf("VCP_GET_PARAM\r");
            uint32_t Param;
            uint32_t Rpl;
            S = PCmd->GetNextToken();
            if((Param = strtoll(S, &S, 16)) != 0) {
                Uart.Printf("Param: %X\r", Param);
                Driver.Motor[DEFAULT_ID].GetParams(Param, &Rpl);
                Uart.Printf("Rpl: %X\r", Rpl);
                Vcp.Rpl("#GetParam %X\n\r", Rpl);
            } // if Param
            else Vcp.Ack(VCP_RPL_CMD_ERROR);

        }
        else if(strcasecmp(S, VCP_MOVE) == 0) {
            Uart.Printf("VCP_MOVE\r");
            uint8_t Rslt = VCP_RPL_OK;
            uint8_t Dir;
            uint32_t Step;
            S = PCmd->GetNextToken();
            Dir = strtoll(S, &S, 16);
            if(Dir == 0 || Dir == 1) {
                Uart.Printf("Dir: %X\r", Dir);
                S = PCmd->GetNextToken();
                if((Step = strtoll(S, &S, 16)) != 0) {
                    Uart.Printf("Step: %X\r", Step);
                    Driver.Motor[DEFAULT_ID].Move(Dir, Step);
                    Vcp.Printf("#Move %X %X\n\r", Dir, Step);
                } // if Value
                else Rslt = VCP_RPL_CMD_ERROR;
            } // if Param
            else Rslt = VCP_RPL_CMD_ERROR;
            if(Rslt != VCP_RPL_OK) Vcp.Ack(Rslt);
        }

        else if(strcasecmp(S, VCP_GOTO) == 0) {
            Uart.Printf("VCP_GOTO\r");
            uint32_t Pos;
            S = PCmd->GetNextToken();
            if((Pos = strtoll(S, &S, 16)) != 0) {
                Driver.Motor[DEFAULT_ID].GoTo(Pos);
                Vcp.Printf("#GoTo %X\n\r", Pos);
            } else Vcp.Ack(VCP_RPL_CMD_ERROR);
        }
        else if(strcasecmp(S, VCP_GOTODIR) == 0) {
            Uart.Printf("VCP_GOTODIR\r");
            uint8_t Rslt = VCP_RPL_OK;
            uint8_t Dir;
            uint32_t Position;
            S = PCmd->GetNextToken();
            Dir = strtoll(S, &S, 16);
            if(Dir == 0 || Dir == 1) {
                Uart.Printf("Dir: %X\r", Dir);
                S = PCmd->GetNextToken();
                if((Position = strtoll(S, &S, 16)) != 0) {
                    Uart.Printf("Step: %X\r", Position);
                    Driver.Motor[DEFAULT_ID].GoTo_Dir(Dir, Position);
                    Vcp.Printf("#GoToDir %X %X\n\r", Dir, Position);
                } // if Value
                else Rslt = VCP_RPL_CMD_ERROR;
            } // if Param
            else Rslt = VCP_RPL_CMD_ERROR;
            if(Rslt != VCP_RPL_OK) Vcp.Ack(Rslt);
        }
        else if(strcasecmp(S, VCP_GOUNTIL) == 0) {
            Uart.Printf("VCP_GOUNTIL\r");
            uint8_t Rslt = VCP_RPL_OK;
            uint8_t Act, Dir;
            uint32_t Position;
            S = PCmd->GetNextToken();
            Act = strtoll(S, &S, 16);
            S = PCmd->GetNextToken();
            Dir = strtoll(S, &S, 16);
            if(Act == 0 || Act == 1 || Dir == 0 || Act == 1) {
                S = PCmd->GetNextToken();
                if((Position = strtoll(S, &S, 16)) != 0) {
                    Driver.Motor[DEFAULT_ID].GoUntil(Act, Dir, Position);
                    Vcp.Printf("#GoUntil %u %u\n\r", Act, Dir);
                } else Rslt = VCP_RPL_CMD_ERROR;
            } else Rslt = VCP_RPL_CMD_ERROR;
                Vcp.Ack(Rslt);

        }
        else if(strcasecmp(S, VCP_RELEASE) == 0) {
            Uart.Printf("VCP_RELEASE\r");
            uint8_t Act, Dir;
            S = PCmd->GetNextToken();
            Act = strtoll(S, &S, 16);
            S = PCmd->GetNextToken();
            Dir = strtoll(S, &S, 16);
            if(Act == 0 || Act == 1 || Dir == 0 || Act == 1) {
                Driver.Motor[DEFAULT_ID].ReleaseSW(Act, Dir);
                Vcp.Printf("#ReleaseSW %u %u\n\r", Act, Dir);
            }
            else Vcp.Ack(VCP_RPL_CMD_ERROR);
        }
        else if(strcasecmp(S, VCP_GO_HOME) == 0) {
            Uart.Printf("VCP_GO_HOME\r");
            Driver.Motor[DEFAULT_ID].GoHome();
            Vcp.Ack();
        }
        else if(strcasecmp(S, VCP_GO_MARK) == 0) {
            Uart.Printf("VCP_GO_MARK\r");
            Driver.Motor[DEFAULT_ID].GoMark();
            Vcp.Ack();
        }

        else if(strcasecmp(S, VCP_RESET_POS) == 0) {
            Uart.Printf("VCP_RESET_POS\r");
            Driver.Motor[DEFAULT_ID].ResetPos();
            Vcp.Ack();
        }

        else if(strcasecmp(S, VCP_SOFT_HiZ) == 0) {
            Uart.Printf("VCP_SOFT_HiZ\r");
            Driver.Motor[DEFAULT_ID].SoftHiZ();
            Vcp.Ack();
        }

        else if(strcasecmp(S, VCP_HARD_HiZ) == 0) {
            Uart.Printf("VCP_HARD_HiZ\r");
            Driver.Motor[DEFAULT_ID].HardHiZ();
            Vcp.Ack();
        }

        else if(strcasecmp(S, VCP_RESET_DEVICE) == 0) {
            Uart.Printf("VCP_RESET_DEVICE\r");
            Driver.Motor[DEFAULT_ID].ResetDevice();
            Vcp.Ack();
        }

        else if(strcasecmp(S, VCP_RUN) == 0) {
            Uart.Printf("VCP_RUN\r");
            uint8_t Rslt = VCP_RPL_OK;
            uint8_t Dir;
            uint32_t Speed;
            S = PCmd->GetNextToken();
            Dir = strtoll(S, &S, 16);
            if(Dir == 0 || Dir == 1) {
                Uart.Printf("Dir: %X\r", Dir);
                S = PCmd->GetNextToken();
                if((Speed = strtoll(S, &S, 16)) != 0) {
                    Uart.Printf("Speed: %X\r", Speed);
                    Driver.Motor[DEFAULT_ID].Run(Dir, Speed);
                    Vcp.Printf("#Run %u %X\n\r", Dir, Speed);
                } // if Value
                else Rslt = VCP_RPL_CMD_ERROR;
            } // if Dir
            else Rslt = LENGTH_ERROR;
            if(Rslt != VCP_RPL_OK) Vcp.Ack(Rslt);
        }

        else if(strcasecmp(S, VCP_STOP) == 0) {
            Uart.Printf("VCP_STOP\r");
            Driver.Motor[DEFAULT_ID].Run(0,0);
            Vcp.Ack();
        }

        else if(strcasecmp(S, VCP_STEP_CLOCK) == 0) {
            Uart.Printf("VCP_STEP_CLOCK\r");
            uint8_t Dir;
            S = PCmd->GetNextToken();
            Dir = strtoll(S, &S, 16);
            if(Dir == 0 || Dir == 1) {
                Uart.Printf("Dir: %X\r", Dir);
                Driver.Motor[DEFAULT_ID].StepClock(Dir);
                Vcp.Printf("#StepClock %u\n\r", Dir);
            } else Vcp.Ack(VCP_RPL_CMD_ERROR);
        }

        else if(strcasecmp(S, VCP_SOFT_STOP) == 0) {
            Uart.Printf("VCP_SOFT_STOP\r");
            Driver.Motor[DEFAULT_ID].SoftStop();
            Vcp.Ack();
        }
        else if(strcasecmp(S, VCP_HARD_STOP) == 0) {
            Uart.Printf("VCP_HARD_STOP\r");
            Driver.Motor[DEFAULT_ID].HardStop();
            Vcp.Ack();
        }
        else if(strcasecmp(S, VCP_GET_STATUS) == 0) {
            Uart.Printf("VCP_GET_STATUS\r");
            uint32_t Status;
            Driver.Motor[DEFAULT_ID].GetStatus(&Status);
            Vcp.Printf("#GetStatus %X\n\r", Status);
        }

        #if 0 // test
        else if(strcasecmp(S, "#test") == 0) {
            while((S = PCmd->GetNextToken()) != NULL) {        // Next token exists
                if(Convert::TryStrToNumber(S, &d) == OK) {  // Next token is number
                    Vcp.Printf("%d ", d);
                }
            }
            Vcp.Ack();
        }
    #endif

        else Vcp.Ack(VCP_RPL_CMD_UNKNOWN);
    } // if Dirver init
    else {
        Vcp.Printf("Driver not init, do #DriverInit\n\r");
    }
}
#endif


#if 1 // ============================ App class ================================
void App_t::Init() {
//    chVTSet(&ITmr, MS2ST(999), MeasureTmrCallback, nullptr);
    PThd = chThdCreateStatic(waAppThread, sizeof(waAppThread), NORMALPRIO, (tfunc_t)AppThread, NULL);
}
#endif
