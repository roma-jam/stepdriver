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
    if(EvtMsk & EVTMSK_MOTOR_ENDSTOP)
    {
        Beeper.Sequence(BEEPER_ENDPOINT_SEQ);
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
//        Driver.Motor[0].Run(0, 5000);
    }
    else if(strcasecmp(cmd, WIFI_CMD_STOP) == 0)
    {
        Uart.Printf("WIFI Stop\r");
//        Driver.Motor[0].Stop();
    }
    else if(strcasecmp(cmd, WIFI_CMD_CALIBRATE) == 0)
    {
        Uart.Printf("WIFI Calibrate\r");
    }
}

void App_t::OnUartCmd(Cmd_t *PCmd)
{
//    Uart.Printf("%S\r", PCmd->S);
    char *Cmd, *pData;

    uint32_t Addr = 0;
    uint32_t Value = 0;

    uint32_t Step = 0;
    uint8_t Rslt = VCP_RPL_OK;
    uint8_t Act, Dir;
    uint32_t Position;
    uint32_t Speed;

//    uint32_t TimePeriod = 0;
//    uint32_t ShutterCount = 0;

    Cmd = PCmd->GetNextToken();

    if(Cmd == NULL)
        return;

    if(strcasecmp(Cmd, VCP_PING) == 0)
    {
        Vcp.CmdRpl(VCP_RPL_OK);
    }

    else if(strcasecmp(Cmd, VCP_POWER_ON) == 0)
    {
        Vcp.CmdRpl(Driver.Init());
    }

    else if(Driver.isInit())
    {
#if 1 // service command
        //writes Value in Param
        switch (Driver.get_cmd_type(Cmd)) {

            case SetParam:
#if (APP_MOTOR_DEBUG_IO)
                Uart.Printf("VCP_SET_PARAM\r");
#endif
                pData = PCmd->GetNextToken();

                if((Addr = PCmd->GetValue(pData)) != 0)
                {
                    pData = PCmd->GetNextToken();
                    Value = PCmd->GetValue(pData);
                    Driver.Motor.SetParam(Addr, Value);
                    Vcp.CmdRpl(VCP_RPL_OK);
                } // if Param
                else
                    Vcp.CmdRpl(VCP_RPL_CMD_ERROR);
                break;

            case cmdType::GetParam:
#if (APP_MOTOR_DEBUG_IO)
                Uart.Printf("VCP_GET_PARAM\r");
#endif
                pData = PCmd->GetNextToken();
                if((Addr = PCmd->GetValue(pData)) != 0)
                {
                    Value = Driver.Motor.GetParam(Addr);
                    Vcp.CmdRplData(Value);
                } // if Param
                else
                    Vcp.CmdRpl(VCP_RPL_CMD_ERROR);
                break;

            case cmdType::Move:
#if (APP_MOTOR_DEBUG_IO)
                Uart.Printf("VCP_MOVE\r");
#endif
                pData = PCmd->GetNextToken();
                Dir = PCmd->GetValue(pData);
                if(Dir == 0 || Dir == 1)
                {
                    pData = PCmd->GetNextToken();
                    if((Step = PCmd->GetValue(pData)) != 0)
                    {
                        Driver.Motor.Move(Dir, Step);
#if (APP_MOTOR_DEBUG_IO)
                        Uart.Printf("Dir %X, Step %X\r", Dir, Step);
#endif
                        Vcp.CmdRpl(VCP_RPL_OK);
                    } // if Value
                    else
                        Vcp.CmdRpl(VCP_RPL_CMD_ERROR);
                } // if Param
                else
                    Vcp.CmdRpl(VCP_RPL_CMD_ERROR);

                break;

            case cmdType::cmdGoTo:
#if (APP_MOTOR_DEBUG_IO)
                Uart.Printf("VCP_GOTO\r");
#endif
                pData = PCmd->GetNextToken();
                if((Value = PCmd->GetValue(pData)) != 0)
                {
                    Driver.Motor.GoTo(Value);
#if (APP_MOTOR_DEBUG_IO)
                    Uart.Printf("Pos %X\n\r", Value);
#endif
                    Vcp.CmdRpl(VCP_RPL_OK);
                }
                else
                    Vcp.CmdRpl(VCP_RPL_CMD_ERROR);
                break;

            case cmdType::cmdGoToDir:
#if (APP_MOTOR_DEBUG_IO)
                Uart.Printf("VCP_GOTODIR\r");
#endif
                pData = PCmd->GetNextToken();
                Dir = PCmd->GetValue(pData);
                if(Dir == 0 || Dir == 1)
                {
                    pData = PCmd->GetNextToken();
                    if((Position = PCmd->GetValue(pData)) != 0)
                    {
                        Driver.Motor.GoTo_Dir(Dir, Position);
#if (APP_MOTOR_DEBUG_IO)
                        Uart.Printf("Dir %X, Pos %X\r", Dir, Position);
#endif
                        Vcp.CmdRpl(VCP_RPL_OK);
                    } // if Value
                    else
                        Vcp.CmdRpl(VCP_RPL_CMD_ERROR);
                } // if Param
                else
                    Vcp.CmdRpl(VCP_RPL_CMD_ERROR);

                break;

            case cmdType::GoUntil:
#if (APP_MOTOR_DEBUG_IO)
                Uart.Printf("VCP_GOUNTIL\r");
#endif
                pData = PCmd->GetNextToken();
                Act = PCmd->GetValue(pData);
                pData = PCmd->GetNextToken();
                Dir = PCmd->GetValue(pData);
                if(Act == 0 || Act == 1 || Dir == 0 || Act == 1)
                {
                    pData = PCmd->GetNextToken();
                    if((Position = PCmd->GetValue(pData)) != 0)
                    {
                        Driver.Motor.GoUntil(Act, Dir, Position);
                        Uart.Printf("Act %X, Dir %X, Pos %X\r", Act, Dir, Position);
                        Vcp.CmdRpl(VCP_RPL_OK);
                    }
                    else
                        Vcp.CmdRpl(VCP_RPL_CMD_ERROR);
                }
                else
                    Vcp.CmdRpl(VCP_RPL_CMD_ERROR);

                break;

            case cmdType::ReleaseSW:
#if (APP_MOTOR_DEBUG_IO)
                Uart.Printf("VCP_RELEASE\r");
#endif
                pData = PCmd->GetNextToken();
                Act = PCmd->GetValue(pData);
                pData = PCmd->GetNextToken();
                Dir = PCmd->GetValue(pData);
                if(Act == 0 || Act == 1 || Dir == 0 || Act == 1)
                {
                    Driver.Motor.ReleaseSW(Act, Dir);
#if (APP_MOTOR_DEBUG_IO)
                    Uart.Printf("%Act X, Dir %X\r", Act, Dir);
#endif
                    Vcp.CmdRpl(VCP_RPL_OK);
                }
                else
                    Vcp.CmdRpl(VCP_RPL_CMD_ERROR);
                break;

            case cmdType::GoHome:
#if (APP_MOTOR_DEBUG_IO)
                Uart.Printf("VCP_GO_HOME\r");
#endif
                Driver.Motor.GoHome();
                Vcp.CmdRpl(VCP_RPL_OK);
                break;

            case cmdType::GoMark:
#if (APP_MOTOR_DEBUG_IO)
                Uart.Printf("VCP_GO_MARK\r");
#endif
                Driver.Motor.GoMark();
                Vcp.CmdRpl(VCP_RPL_OK);
                break;

            case cmdType::ResetPos:
#if (APP_MOTOR_DEBUG_IO)
                Uart.Printf("VCP_RESET_POS\r");
#endif
                Driver.Motor.ResetPos();
                Vcp.CmdRpl(VCP_RPL_OK);
                break;

            case cmdType::SoftHiZ:
#if (APP_MOTOR_DEBUG_IO)
                Uart.Printf("VCP_SOFT_HiZ\r");
#endif
                Driver.Motor.SoftHiZ();
                Vcp.CmdRpl(VCP_RPL_OK);
                break;

            case cmdType::HardHiZ:
#if (APP_MOTOR_DEBUG_IO)
                Uart.Printf("VCP_HARD_HiZ\r");
#endif
                Driver.Motor.HardHiZ();
                Vcp.CmdRpl(VCP_RPL_OK);
                break;

            case cmdType::ResetDevice:
#if (APP_MOTOR_DEBUG_IO)
                Uart.Printf("VCP_RESET_DEVICE\r");
#endif
                Driver.Motor.ResetDevice();
                Vcp.CmdRpl(VCP_RPL_OK);
                break;

            case cmdType::Run:
#if (APP_MOTOR_DEBUG_IO)
                Uart.Printf("VCP_RUN\r");
#endif
                pData = PCmd->GetNextToken();
                Dir =  PCmd->GetValue(pData);
                if(Dir == 0 || Dir == 1) {
                    pData = PCmd->GetNextToken();
                    if((Speed =  PCmd->GetValue(pData)) != 0)
                    {
                        Driver.Motor.Run(Dir, Speed);
#if (APP_MOTOR_DEBUG_IO)
                        Uart.Printf("Dir %X, Speed %X\n\r", Dir, Speed);
#endif
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
#if (APP_MOTOR_DEBUG_IO)
                Uart.Printf("VCP_STOP\r");
#endif
                Driver.Motor.Stop();
                Vcp.CmdRpl(VCP_RPL_OK);
                break;

            case cmdType::StepClock:
#if (APP_MOTOR_DEBUG_IO)
                Uart.Printf("VCP_STEP_CLOCK\r");
#endif
                pData = PCmd->GetNextToken();
                Dir = PCmd->GetValue(pData);
                if(Dir == 0 || Dir == 1)
                {
                    Driver.Motor.StepClock(Dir);
#if (APP_MOTOR_DEBUG_IO)
                    Uart.Printf("Dir %X\r", Dir);
#endif
                    Vcp.CmdRpl(VCP_RPL_OK);
                }
                else
                    Vcp.CmdRpl(VCP_RPL_CMD_ERROR);

                break;

            case cmdType::SoftStop:
#if (APP_MOTOR_DEBUG_IO)
                Uart.Printf("VCP_SOFT_STOP\r");
#endif
                Driver.Motor.SoftStop();
                Vcp.CmdRpl(VCP_RPL_OK);
                break;

            case cmdType::HardStop:
#if (APP_MOTOR_DEBUG_IO)
                Uart.Printf("VCP_HARD_STOP\r");
#endif
                Driver.Motor.HardStop();
                Vcp.CmdRpl(VCP_RPL_OK);
                break;

            case cmdType::GetStaus:
#if (APP_MOTOR_DEBUG_IO)
                Uart.Printf("VCP_GET_STATUS\r");
#endif
                Value = Driver.Motor.GetStatus();
#if (APP_MOTOR_DEBUG_IO)
                Uart.Printf("Status %X\r", Value);
#endif
                Vcp.CmdRplData(Value);
                break;

            case cmdType::UpdateParam:
#if (APP_MOTOR_DEBUG_IO)
                Uart.Printf("VCP_UPDATE_PARAM\r");
#endif
//                Driver.Motor[DEFAULT_ID].UpdatePrm();
                break;

            case cmdType::Calibrate:
#if (APP_MOTOR_DEBUG_IO)
                Uart.Printf("VCP_CALIBRATE\r");
#endif
//                Driver.Motor[DEFAULT_ID].SetState(msCalibrate);
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
    PThd = chThdCreateStatic(waAppThread, sizeof(waAppThread), NORMALPRIO, (tfunc_t)AppThread, NULL);
}
