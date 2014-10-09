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
//    Uart.Printf("%S\r", PCmd->S);
    char *S;
    S = PCmd->GetNextToken();
    if(S == NULL) return;

    if(strcasecmp(S, VCP_PING) == 0) Vcp.CmdRpl(VCP_RPL_OK);

    else if(strcasecmp(S, VCP_DRIVER_INIT) == 0) {
        uint8_t Rslt = Driver.Init();
        Vcp.CmdRpl(Rslt);
    }

    else if(Driver.IsInit()) {
#if 1 // service command
        if(strcasecmp(S, VCP_SET_PARAM) == 0) { //writes Value in Param
            Uart.Printf("VCP_SET_PARAM\r");
            uint8_t Param;
            uint32_t Value;
            S = PCmd->GetNextToken();
            if((Param = strtoll(S, &S, 16)) != 0) {
                S = PCmd->GetNextToken();
                Value = strtoll(S, &S, 16);
                Driver.Motor[DEFAULT_ID].SetParam(Param, Value);
                Vcp.CmdRpl(VCP_RPL_OK);
            } // if Param
            else Vcp.CmdRpl(VCP_RPL_CMD_ERROR);

        }
        else if(strcasecmp(S, VCP_GET_PARAM) == 0) {
            Uart.Printf("VCP_GET_PARAM\r");
            uint32_t Param;
            uint32_t Rpl;
            S = PCmd->GetNextToken();
            if((Param = strtoll(S, &S, 16)) != 0) {
                Driver.Motor[DEFAULT_ID].GetParams(Param, &Rpl);
                Uart.Printf("%X\n\r", Rpl);
                Vcp.CmdRpl(VCP_RPL_OK, 1, Rpl);
            } // if Param
            else Vcp.CmdRpl(VCP_RPL_CMD_ERROR);

        }
        else if(strcasecmp(S, VCP_MOVE) == 0) {
            Uart.Printf("VCP_MOVE\r");
            uint8_t Rslt = VCP_RPL_OK;
            uint8_t Dir;
            uint32_t Step;
            S = PCmd->GetNextToken();
            Dir = strtoll(S, &S, 16);
            if(Dir == 0 || Dir == 1) {
                S = PCmd->GetNextToken();
                if((Step = strtoll(S, &S, 16)) != 0) {
                    Driver.Motor[DEFAULT_ID].Move(Dir, Step);
                    Uart.Printf("%X %X\r", Dir, Step);
                    Vcp.CmdRpl(VCP_RPL_OK);
                } // if Value
                else Rslt = VCP_RPL_CMD_ERROR;
            } // if Param
            else Rslt = VCP_RPL_CMD_ERROR;
            if(Rslt != VCP_RPL_OK) Vcp.CmdRpl(Rslt);
        }

        else if(strcasecmp(S, VCP_GOTO) == 0) {
            Uart.Printf("VCP_GOTO\r");
            uint32_t Pos;
            S = PCmd->GetNextToken();
            if((Pos = strtoll(S, &S, 16)) != 0) {
                Driver.Motor[DEFAULT_ID].GoTo(Pos);
                Uart.Printf("%X\n\r", Pos);
                Vcp.CmdRpl(VCP_RPL_OK);
            } else Vcp.CmdRpl(VCP_RPL_CMD_ERROR);
        }
        else if(strcasecmp(S, VCP_GOTODIR) == 0) {
            Uart.Printf("VCP_GOTODIR\r");
            uint8_t Rslt = VCP_RPL_OK;
            uint8_t Dir;
            uint32_t Position;
            S = PCmd->GetNextToken();
            Dir = strtoll(S, &S, 16);
            if(Dir == 0 || Dir == 1) {
                S = PCmd->GetNextToken();
                if((Position = strtoll(S, &S, 16)) != 0) {
                    Driver.Motor[DEFAULT_ID].GoTo_Dir(Dir, Position);
                    Uart.Printf("%X %X\r", Dir, Position);
                    Vcp.CmdRpl(VCP_RPL_OK);
                } // if Value
                else Rslt = VCP_RPL_CMD_ERROR;
            } // if Param
            else Rslt = VCP_RPL_CMD_ERROR;
            if(Rslt != VCP_RPL_OK) Vcp.CmdRpl(Rslt);
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
                    Uart.Printf("%X %X\r", Act, Dir);
                    Vcp.CmdRpl(VCP_RPL_OK);
                } else Rslt = VCP_RPL_CMD_ERROR;
            } else Rslt = VCP_RPL_CMD_ERROR;
            Vcp.CmdRpl(Rslt);
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
                Uart.Printf("%X %X\r", Act, Dir);
                Vcp.CmdRpl(VCP_RPL_OK);
            }
            else Vcp.CmdRpl(VCP_RPL_CMD_ERROR);
        }
        else if(strcasecmp(S, VCP_GO_HOME) == 0) {
            Uart.Printf("VCP_GO_HOME\r");
            Driver.Motor[DEFAULT_ID].GoHome();
            Vcp.CmdRpl(VCP_RPL_OK);
        }
        else if(strcasecmp(S, VCP_GO_MARK) == 0) {
            Uart.Printf("VCP_GO_MARK\r");
            Driver.Motor[DEFAULT_ID].GoMark();
            Vcp.CmdRpl(VCP_RPL_OK);
        }

        else if(strcasecmp(S, VCP_RESET_POS) == 0) {
            Uart.Printf("VCP_RESET_POS\r");
            Driver.Motor[DEFAULT_ID].ResetPos();
            Vcp.CmdRpl(VCP_RPL_OK);
        }

        else if(strcasecmp(S, VCP_SOFT_HiZ) == 0) {
            Uart.Printf("VCP_SOFT_HiZ\r");
            Driver.Motor[DEFAULT_ID].SoftHiZ();
            Vcp.CmdRpl(VCP_RPL_OK);
        }

        else if(strcasecmp(S, VCP_HARD_HiZ) == 0) {
            Uart.Printf("VCP_HARD_HiZ\r");
            Driver.Motor[DEFAULT_ID].HardHiZ();
            Vcp.CmdRpl(VCP_RPL_OK);
        }

        else if(strcasecmp(S, VCP_RESET_DEVICE) == 0) {
            Uart.Printf("VCP_RESET_DEVICE\r");
            Driver.Motor[DEFAULT_ID].ResetDevice();
            Vcp.CmdRpl(VCP_RPL_OK);
        }

        else if(strcasecmp(S, VCP_RUN) == 0) {
            Uart.Printf("VCP_RUN\r");
            uint8_t Rslt = VCP_RPL_OK;
            uint8_t Dir;
            uint32_t Speed;
            S = PCmd->GetNextToken();
            Dir = strtoll(S, &S, 16);
            if(Dir == 0 || Dir == 1) {
                S = PCmd->GetNextToken();
                if((Speed = strtoll(S, &S, 16)) != 0) {
                    Driver.Motor[DEFAULT_ID].Run(Dir, Speed);
                    Uart.Printf("%X %X\n\r", Dir, Speed);
                    Vcp.CmdRpl(VCP_RPL_OK);
                } // if Value
                else Rslt = VCP_RPL_CMD_ERROR;
            } // if Dir
            else Rslt = LENGTH_ERROR;
            if(Rslt != VCP_RPL_OK) Vcp.CmdRpl(Rslt);
        }

        else if(strcasecmp(S, VCP_STOP) == 0) {
            Uart.Printf("VCP_STOP\r");
            Driver.Motor[DEFAULT_ID].Run(0,0);
            Vcp.CmdRpl(VCP_RPL_OK);
        }

        else if(strcasecmp(S, VCP_STEP_CLOCK) == 0) {
            Uart.Printf("VCP_STEP_CLOCK\r");
            uint8_t Dir;
            S = PCmd->GetNextToken();
            Dir = strtoll(S, &S, 16);
            if(Dir == 0 || Dir == 1) {
                Driver.Motor[DEFAULT_ID].StepClock(Dir);
                Uart.Printf("%X\r", Dir);
                Vcp.CmdRpl(VCP_RPL_OK);
            } else Vcp.CmdRpl(VCP_RPL_CMD_ERROR);
        }

        else if(strcasecmp(S, VCP_SOFT_STOP) == 0) {
            Uart.Printf("VCP_SOFT_STOP\r");
            Driver.Motor[DEFAULT_ID].SoftStop();
            Vcp.CmdRpl(VCP_RPL_OK);
        }
        else if(strcasecmp(S, VCP_HARD_STOP) == 0) {
            Uart.Printf("VCP_HARD_STOP\r");
            Driver.Motor[DEFAULT_ID].HardStop();
            Vcp.CmdRpl(VCP_RPL_OK);
        }
        else if(strcasecmp(S, VCP_GET_STATUS) == 0) {
            Uart.Printf("VCP_GET_STATUS\r");
            uint32_t Status;
            Driver.Motor[DEFAULT_ID].GetStatus(&Status);
            Uart.Printf("%X\r", Status);
            Vcp.CmdRpl(VCP_RPL_OK, 1, Status);
        }
#endif

#if 1 // system command
        else if(strcasecmp(S, VCP_UPDATE_PARAM) == 0) {
            Uart.Printf("VCP_UPDATE_PARAM\r");
            Driver.Motor[DEFAULT_ID].UpdatePrm();
        }

        else if(strcasecmp(S, VCP_TIMELAPSE_PARAM) == 0) {
            Uart.Printf("VCP_TIMELAPSE_PARAM\r");
            uint32_t TimePeriod = 0;
            uint32_t ShutterCount = 0;
            S = PCmd->GetNextToken();
            if((TimePeriod = strtoll(S, &S, 16)) != 0) {
                TimePeriod *= 60;
                ShutterCount = TimePeriod*30; // 30 fps
                TimePeriod *= 60;
                uint32_t TimeD = TimePeriod/ShutterCount;
                TimePeriod *= 1000; // now in msec
                Uart.Printf("TimePeriod: %u msec\r", TimePeriod);
                Uart.Printf("ShutterCount: %u\r", ShutterCount);
                Uart.Printf("TimeD: %u\r", TimeD);
                TimeDelay = TimeD;
                StepSize = (GlideTrackMaxStep - 1) / ShutterCount;
                Uart.Printf("StepSize: %u\r", StepSize);
                Vcp.CmdRpl(VCP_RPL_OK);
            } else Vcp.CmdRpl(VCP_RPL_CMD_ERROR);
        }

        else if(strcasecmp(S, VCP_TIMELAPSE_START) == 0) {
            Uart.Printf("VCP_TIMELAPSE_START\r");
            Driver.Motor[DEFAULT_ID].SetState(msTimeLapse);
            Vcp.CmdRpl(VCP_RPL_OK);
        }

        else if(strcasecmp(S, VCP_TIMELAPSE_STOP) == 0) {
            Uart.Printf("VCP_TIMELAPSE_STOP\r");
            Driver.Motor[DEFAULT_ID].SetState(msSleep);
            Vcp.CmdRpl(VCP_RPL_OK);
        }

        else if(strcasecmp(S, VCP_GLIDETRACK_SIZE_SET) == 0) {
            Uart.Printf("VCP_GLIDETRACK_SIZE_SET\r");
            Driver.Motor[DEFAULT_ID].GetParams(ADDR_ABS_POS, &GlideTrackMaxStep);
            Uart.Printf("Max size %X\r", GlideTrackMaxStep);
        }

#endif

        else Vcp.CmdRpl(VCP_RPL_CMD_UNKNOWN);
    } // if Dirver init

    else {
        Vcp.CmdRpl(VCP_RPL_NOT_INIT);
    }
}
#endif

void App_t::Init() {
//    chVTSet(&ITmr, MS2ST(999), MeasureTmrCallback, nullptr);
    PThd = chThdCreateStatic(waAppThread, sizeof(waAppThread), NORMALPRIO, (tfunc_t)AppThread, NULL);
}
