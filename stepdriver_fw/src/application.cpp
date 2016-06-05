/*
 * application.cpp
 *
 *  Created on: 19 февр. 2014 г.
 *      Author: r.leonov
 */

#include "application.h"
#include "vcp.h"
#include "motor_ctrl.h"
#include "http_server.h"
#include "wifi_driver.h"
#include "beeper.h"

#if 1 // ==== Init's ====
App_t App;

void AppTask();

static WORKING_AREA(waAppThread, 1024);
__attribute__ ((__noreturn__))
static void AppThread(void *arg)
{
    chRegSetThreadName("Motor");
    while(1)
        AppTask();
}
#endif

void AppTask()
{
    uint32_t EvtMsk = chEvtWaitAny(ALL_EVENTS);

#if 1 // =========================== WIFI =====================================
    if(EvtMsk & EVTMSK_WIFI_READY)
    {
        HttpServer.OpenSocket();
    } // WiFi Module Ready

    if(EvtMsk & EVTMSK_WIFI_STARTED)
    {
        Beeper.Sequence(BEEPER_WIFI_RDY_SEQ);
    }

    if(EvtMsk & EVTMSK_WIFI_HTTP_GET_REQUEST)
    {
        HttpServer.GetRequest();
    }

    if(EvtMsk & EVTMSK_WIFI_HTTP_ACTION)
    {
#if(APP_HTTP_SERVER_DEBUG)
        Uart.Printf("Action: %s\r", HttpServer.CurrData);
#endif
        App.OnWiFiCmd(HttpServer.CurrData);
        HttpServer.ActionReply();
    }
#endif

#if 1 // =========================== MOTOR DRIVER =============================
    if(EvtMsk & EVTMSK_WIFI_STARTED)
    {
        Driver.EndStop();
    }
#endif

}

#if 1 // ======================= Command processing ============================
void App_t::OnWiFiCmd(char* Request)
{
    char *cmd = &Request[7];
    if(strcasecmp(cmd, WIFI_CMD_START) == 0)
    {
        Uart.Printf("WIFI Start\r");
    }
    else if(strcasecmp(cmd, WIFI_CMD_STOP) == 0)
    {
        Uart.Printf("WIFI Stop\r");
    }
    else if(strcasecmp(cmd, WIFI_CMD_CALIBRATE) == 0)
    {
        Uart.Printf("WIFI Calibrate\r");
    }
}

void App_t::OnUartCmd(Cmd_t *PCmd)
{
//    Uart.Printf("%S\r", PCmd->S);
    char *S;
    uint32_t Param = 0;
    uint32_t Value = 0;
    uint32_t Step = 0;
    uint8_t Rslt = VCP_RPL_OK;
    uint8_t Act, Dir;
    uint32_t Position;
    uint32_t Speed;
    uint32_t TimePeriod = 0;
    uint32_t ShutterCount = 0;

    S = PCmd->GetNextToken();
    if(S == NULL) return;

    if(strcasecmp(S, VCP_PING) == 0)
    {
        Vcp.CmdRpl(VCP_RPL_OK);
    }

    else if(strcasecmp(S, VCP_DRIVER_INIT) == 0)
    {
        uint8_t Rslt = Driver.Init();
        Vcp.CmdRpl(Rslt);
    }

    else if(Driver.isInit())
    {
#if 1 // service command
        //writes Value in Param
        cmdType cmd_type = Driver.get_cmd_type(S);
        switch (cmd_type) {

            case cmdType::SetParam:
                Uart.Printf("VCP_SET_PARAM\r");
                S = PCmd->GetNextToken();

                if((Param = strtoll(S, &S, 16)) != 0)
                {
                    S = PCmd->GetNextToken();
                    Value = strtoll(S, &S, 16);
                    Driver.Motor[DEFAULT_ID].SetParam(Param, Value);
                    Vcp.CmdRpl(VCP_RPL_OK);
                } // if Param
                else
                {
                    Vcp.CmdRpl(VCP_RPL_CMD_ERROR);
                }
                break;

            case cmdType::GetParam:
                Uart.Printf("VCP_GET_PARAM\r");
                uint32_t Rpl;
                S = PCmd->GetNextToken();
                if((Param = strtoll(S, &S, 16)) != 0)
                {
                    Driver.Motor[DEFAULT_ID].GetParams(Param, &Rpl);
                    Uart.Printf("%X\n\r", Rpl);
                    Vcp.CmdRpl(VCP_RPL_OK, 1, &Rpl);
                } // if Param
                else Vcp.CmdRpl(VCP_RPL_CMD_ERROR);
                break;

            case cmdType::Move:
                Uart.Printf("VCP_MOVE\r");
                S = PCmd->GetNextToken();
                Dir = strtoll(S, &S, 16);
                if(Dir == 0 || Dir == 1)
                {
                    S = PCmd->GetNextToken();
                    if((Step = strtoll(S, &S, 16)) != 0)
                    {
                        Driver.Motor[DEFAULT_ID].Move(Dir, Step);
                        Uart.Printf("%X %X\r", Dir, Step);
                        Vcp.CmdRpl(VCP_RPL_OK);
                    } // if Value
                    else
                        Rslt = VCP_RPL_CMD_ERROR;
                } // if Param
                else
                    Rslt = VCP_RPL_CMD_ERROR;

                if(Rslt != VCP_RPL_OK)
                    Vcp.CmdRpl(Rslt);
                break;

            case cmdType::cmdGoTo:
                Uart.Printf("VCP_GOTO\r");
                uint32_t Pos;
                S = PCmd->GetNextToken();
                if((Pos = strtoll(S, &S, 16)) != 0)
                {
                    Driver.Motor[DEFAULT_ID].GoTo(Pos);
                    Uart.Printf("%X\n\r", Pos);
                    Vcp.CmdRpl(VCP_RPL_OK);
                }
                else
                    Vcp.CmdRpl(VCP_RPL_CMD_ERROR);
                            break;

            case cmdType::cmdGoToDir:
                Uart.Printf("VCP_GOTODIR\r");
                S = PCmd->GetNextToken();
                Dir = strtoll(S, &S, 16);
                if(Dir == 0 || Dir == 1)
                {
                    S = PCmd->GetNextToken();
                    if((Position = strtoll(S, &S, 16)) != 0)
                    {
                        Driver.Motor[DEFAULT_ID].GoTo_Dir(Dir, Position);
                        Uart.Printf("%X %X\r", Dir, Position);
                        Vcp.CmdRpl(VCP_RPL_OK);
                    } // if Value
                    else
                        Rslt = VCP_RPL_CMD_ERROR;
                } // if Param
                else
                    Rslt = VCP_RPL_CMD_ERROR;

                if(Rslt != VCP_RPL_OK)
                    Vcp.CmdRpl(Rslt);
                            break;

            case cmdType::GoUntil:
                Uart.Printf("VCP_GOUNTIL\r");
                S = PCmd->GetNextToken();
                Act = strtoll(S, &S, 16);
                S = PCmd->GetNextToken();
                Dir = strtoll(S, &S, 16);
                if(Act == 0 || Act == 1 || Dir == 0 || Act == 1)
                {
                    S = PCmd->GetNextToken();
                    if((Position = strtoll(S, &S, 16)) != 0)
                    {
                        Driver.Motor[DEFAULT_ID].GoUntil(Act, Dir, Position);
                        Uart.Printf("%X %X\r", Act, Dir);
                        Vcp.CmdRpl(VCP_RPL_OK);
                    }
                    else
                        Rslt = VCP_RPL_CMD_ERROR;
                }
                else
                    Rslt = VCP_RPL_CMD_ERROR;
                Vcp.CmdRpl(Rslt);
                            break;

            case cmdType::ReleaseSW:
                Uart.Printf("VCP_RELEASE\r");
                S = PCmd->GetNextToken();
                Act = strtoll(S, &S, 16);
                S = PCmd->GetNextToken();
                Dir = strtoll(S, &S, 16);
                if(Act == 0 || Act == 1 || Dir == 0 || Act == 1)
                {
                    Driver.Motor[DEFAULT_ID].ReleaseSW(Act, Dir);
                    Uart.Printf("%X %X\r", Act, Dir);
                    Vcp.CmdRpl(VCP_RPL_OK);
                }
                else
                    Vcp.CmdRpl(VCP_RPL_CMD_ERROR);
                            break;

            case cmdType::GoHome:
                Uart.Printf("VCP_GO_HOME\r");
                Driver.Motor[DEFAULT_ID].GoHome();
                Vcp.CmdRpl(VCP_RPL_OK);
                            break;

            case cmdType::GoMark:
                Uart.Printf("VCP_GO_MARK\r");
                Driver.Motor[DEFAULT_ID].GoMark();
                Vcp.CmdRpl(VCP_RPL_OK);
                            break;

            case cmdType::ResetPos:
                Uart.Printf("VCP_RESET_POS\r");
                Driver.Motor[DEFAULT_ID].ResetPos();
                Vcp.CmdRpl(VCP_RPL_OK);
                            break;

            case cmdType::SoftHiZ:
                Uart.Printf("VCP_SOFT_HiZ\r");
                Driver.Motor[DEFAULT_ID].SoftHiZ();
                Vcp.CmdRpl(VCP_RPL_OK);
                            break;

            case cmdType::HardHiZ:
                Uart.Printf("VCP_HARD_HiZ\r");
                Driver.Motor[DEFAULT_ID].HardHiZ();
                Vcp.CmdRpl(VCP_RPL_OK);
                            break;

            case cmdType::ResetDevice:
                Uart.Printf("VCP_RESET_DEVICE\r");
                Driver.Motor[DEFAULT_ID].ResetDevice();
                Vcp.CmdRpl(VCP_RPL_OK);
                            break;

            case cmdType::Run:
                Uart.Printf("VCP_RUN\r");
                S = PCmd->GetNextToken();
                Dir = strtoll(S, &S, 16);
                if(Dir == 0 || Dir == 1) {
                    S = PCmd->GetNextToken();
                    if((Speed = strtoll(S, &S, 16)) != 0)
                    {
                        Driver.Motor[DEFAULT_ID].Run(Dir, Speed);
                        Uart.Printf("%X %X\n\r", Dir, Speed);
                        Vcp.CmdRpl(VCP_RPL_OK);
                    } // if Value
                    else
                        Rslt = VCP_RPL_CMD_ERROR;
                } // if Dir
                else
                    Rslt = LENGTH_ERROR;

                if(Rslt != VCP_RPL_OK)
                    Vcp.CmdRpl(Rslt);
                            break;

            case cmdType::Stop:
                Uart.Printf("VCP_STOP\r");
                Driver.Motor[DEFAULT_ID].Run(0,0);
                Vcp.CmdRpl(VCP_RPL_OK);
                            break;

            case cmdType::StepClock:
                Uart.Printf("VCP_STEP_CLOCK\r");
                S = PCmd->GetNextToken();
                Dir = strtoll(S, &S, 16);
                if(Dir == 0 || Dir == 1)
                {
                    Driver.Motor[DEFAULT_ID].StepClock(Dir);
                    Uart.Printf("%X\r", Dir);
                    Vcp.CmdRpl(VCP_RPL_OK);
                }
                else
                    Vcp.CmdRpl(VCP_RPL_CMD_ERROR);
                            break;

            case cmdType::SoftStop:
                Uart.Printf("VCP_SOFT_STOP\r");
                Driver.Motor[DEFAULT_ID].SoftStop();
                Vcp.CmdRpl(VCP_RPL_OK);
                            break;

            case cmdType::HardStop:
                Uart.Printf("VCP_HARD_STOP\r");
                Driver.Motor[DEFAULT_ID].HardStop();
                Vcp.CmdRpl(VCP_RPL_OK);
                            break;

            case cmdType::GetStaus:
                Uart.Printf("VCP_GET_STATUS\r");
                uint32_t Status;
                Driver.Motor[DEFAULT_ID].GetStatus(&Status);
                Uart.Printf("%X\r", Status);
                Vcp.CmdRpl(VCP_RPL_OK, 1, &Status);
                            break;

            case cmdType::UpdateParam:
                Uart.Printf("VCP_UPDATE_PARAM\r");
                Driver.Motor[DEFAULT_ID].UpdatePrm();
                            break;

            case cmdType::TLParam:
                Uart.Printf("VCP_TIMELAPSE_PARAM\r");
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
                            break;

            case cmdType::TLStart:
                Uart.Printf("VCP_TIMELAPSE_START\r");
                Driver.Motor[DEFAULT_ID].SetState(msTimeLapse);
                Vcp.CmdRpl(VCP_RPL_OK);
                            break;

            case cmdType::TLStop:
                Uart.Printf("VCP_TIMELAPSE_STOP\r");
                Driver.Motor[DEFAULT_ID].SetState(msSleep);
                Vcp.CmdRpl(VCP_RPL_OK);
                            break;

            case cmdType::SetSize:
                Uart.Printf("VCP_GLIDETRACK_SIZE_SET\r");
                Driver.Motor[DEFAULT_ID].GetParams(ADDR_ABS_POS, &GlideTrackMaxStep);
                Uart.Printf("Max size %X\r", GlideTrackMaxStep);
                Vcp.CmdRpl(VCP_RPL_OK, 1, &GlideTrackMaxStep);
                            break;

            default:
                Vcp.CmdRpl(VCP_RPL_CMD_UNKNOWN);
        }
#endif
    } // Driver init
    // if Dirver not init
    else
    {
        Vcp.CmdRpl(VCP_RPL_NOT_INIT);
    }
}
#endif

void App_t::Init()
{
//    chVTSet(&ITmr, MS2ST(999), MeasureTmrCallback, nullptr);
    PThd = chThdCreateStatic(waAppThread, sizeof(waAppThread), NORMALPRIO, (tfunc_t)AppThread, NULL);
}
