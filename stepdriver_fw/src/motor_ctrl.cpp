/*
 * motor_ctrl.cpp
 *
 *  Created on: 19 дек. 2013 г.
 *      Author: r.leonov
 */



#include "motor_ctrl.h"
#include "vcp.h"
#include "application.h"

// Macros
#define PRINT_MSG(S)       { Uart.Printf(S); }

Driver_t Driver;

static WORKING_AREA(waDriverThread, 256);
__attribute__ ((__noreturn__))
static void DriverThread(void *arg)
{
    chRegSetThreadName("Motor Driver");
    while(1)
        Driver.Task();
}

cmdType Driver_t::get_cmd_type(char *S)
{
    if(strcasecmp(S, VCP_SET_PARAM_STRING) == 0)              return cmdType::SetParam;
    if(strcasecmp(S, VCP_GET_PARAM_STRING) == 0)              return cmdType::GetParam;
    if(strcasecmp(S, VCP_MOVE_STRING) == 0)                   return cmdType::Move;
    if(strcasecmp(S, VCP_GOTO_STRING) == 0)                   return cmdType::cmdGoTo;
    if(strcasecmp(S, VCP_GOTODIR_STRING) == 0)                return cmdType::cmdGoToDir;
    if(strcasecmp(S, VCP_GOUNTIL_STRING) == 0)                return cmdType::GoUntil;
    if(strcasecmp(S, VCP_RELEASE_STRING) == 0)                return cmdType::ReleaseSW;
    if(strcasecmp(S, VCP_GO_HOME_STRING) == 0)                return cmdType::GoHome;
    if(strcasecmp(S, VCP_GO_MARK_STRING) == 0)                return cmdType::GoMark;
    if(strcasecmp(S, VCP_RESET_POS_STRING) == 0)              return cmdType::ResetPos;
    if(strcasecmp(S, VCP_SOFT_HiZ_STRING) == 0)               return cmdType::SoftHiZ;
    if(strcasecmp(S, VCP_HARD_HiZ_STRING) == 0)               return cmdType::HardHiZ;
    if(strcasecmp(S, VCP_RESET_DEVICE_STRING) == 0)           return cmdType::ResetDevice;
    if(strcasecmp(S, VCP_RUN_STRING) == 0)                    return cmdType::Run;
    if(strcasecmp(S, VCP_STOP_STRING) == 0)                   return cmdType::Stop;
    if(strcasecmp(S, VCP_STEP_CLOCK_STRING) == 0)             return cmdType::StepClock;
    if(strcasecmp(S, VCP_SOFT_STOP_STRING) == 0)              return cmdType::SoftStop;
    if(strcasecmp(S, VCP_HARD_STOP_STRING) == 0)              return cmdType::HardStop;
    if(strcasecmp(S, VCP_GET_STATUS_STRING) == 0)             return cmdType::GetStaus;
    if(strcasecmp(S, VCP_UPDATE_PARAM_STRING) == 0)           return cmdType::UpdateParam;
    if(strcasecmp(S, VCP_TIMELAPSE_PARAM_STRING) == 0)        return cmdType::TLParam;
    if(strcasecmp(S, VCP_TIMELAPSE_START_STRING) == 0)        return cmdType::TLStart;
    if(strcasecmp(S, VCP_TIMELAPSE_STOP_STRING) == 0)         return cmdType::TLStop;
    if(strcasecmp(S, VCP_GLIDETRACK_SIZE_SET_STRING) == 0)    return cmdType::SetSize;
    if(strcasecmp(S, VCP_CALIBRATE) == 0)    return cmdType::Calibrate;
    return cmdType::Err;
}

Rslt_t Driver_t::Init()
{
    if(Motor.isPowered())
        return VCP_RPL_OK;

    Spi.Init();
    Motor.SetState(msOff);

    if(PThread == nullptr)
        PThread = chThdCreateStatic(waDriverThread, sizeof(waDriverThread), NORMALPRIO, (tfunc_t)DriverThread, NULL);

    PCmdBuf = Cmd;
    PAckBuf = Ack.Buf;
    CmdLength = 0;

    Motor.SetState(msPowerUp);

    // wait until we powered on correctly
    uint32_t Timeout = 0;
    do
    {
        chThdSleepMilliseconds(100);
        if (Motor.isPowered())
            return VCP_RPL_OK;
    } while (Timeout++ < DRIVER_INIT_TIMEOUT);

    return VCP_RPL_INIT_TIMEOUT;
}

#if 1 // ========================= MOTOR DRIVER ================================
void Driver_t::Task()
{
    if(Motor.State == Motor.NewState)
    {
        chThdSleepMilliseconds(99);
        return;
    }

    Motor.State = Motor.NewState;

    switch (Motor.State)
    {
        case msIdle:
        case msOff:
            chThdSleepMilliseconds(999);
            break;

        case msPowerUp:
#if (APP_MOTOR_DRIVER_DEBUG)
            Uart.Printf("MC: preparing\r");
#endif
            Motor.Init();
            Motor.UpdatePrm();
            if(Motor.param.max_speed != 0)
            {
#if (APP_MOTOR_DRIVER_DEBUG)
                Uart.Printf("MC: powered up\r");
#endif
                Motor.SetState(msIdle);
                Motor.PoweredOn();
                break;
            }

#if (APP_MOTOR_DRIVER_DEBUG)
            Uart.Printf("MC: powered failure\r");
#endif
            Motor.SetState(msOff);
            break;

//        case msTimeLapse:
//            Motor[i].Move(1, App.StepSize);
//            chThdSleepMilliseconds(App.TimeDelay);
//            if(Motor[i].GetPosition() >= App.GlideTrackMaxStep)
//            {
//#if (APP_MOTOR_DRIVER_DEBUG)
//                Uart.Printf("MC: Timelapse end\r");
//#endif
//                Motor[i].Stop();
//                Motor[i].SetGoHome();
//            }
//            break;

//        case msGoHome:
//#if (APP_MOTOR_DRIVER_DEBUG)
//            Uart.Printf("MC: goHome\r");
//#endif
//            Motor[i].GoHome();
//            Motor[i].SetIdle();
//            break;
//
//        case msCalibrate:
//#if (APP_MOTOR_DRIVER_DEBUG)
//            Uart.Printf("MC: Calibrate\r");
//#endif
//            // run
//            Motor[i].Run(0, 9000);
//            Motor[i].SetState(msCalibrate1);

    }
}
#endif

void Driver_t::EndStop()
{
#if (APP_MOTOR_DRIVER_DEBUG)
    Uart.Printf("MC: End Stop\r");
#endif
//    Motor[DEFAULT_ID].Stop();
//    if(Motor[DEFAULT_ID].State == msCalibrate1)
//    {
//        Motor[DEFAULT_ID].ResetPos();
//        Motor[DEFAULT_ID].forward = 1;
//        Motor[DEFAULT_ID].backward = 0;
//        Motor[DEFAULT_ID].SetState(msCalibrate2);
//        Motor[DEFAULT_ID].Run(Motor[DEFAULT_ID].forward, 9000);
//        return;
//    }
//
//    if(Motor[DEFAULT_ID].State == msCalibrate2)
//    {
//        uint32_t MaxStep;
//        Motor[DEFAULT_ID].GetParams(ADDR_ABS_POS, &MaxStep);
//        Uart.Printf("MC: Max Size: %u steps\r", MaxStep);
//        Motor[DEFAULT_ID].GoHome();
//    }

}


#if 1 // =========================== MOTOR LOW LEVEL ==========================
void Motor_t::Init()
{
    ResetDevice();
}

void Motor_t::UpdatePrm()
{
    param.curr_pos = GetParam(ADDR_ABS_POS);
    param.el_pos = GetParam(ADDR_EL_POS);
    param.mark_pos = GetParam(ADDR_MARK);
    param.speed = GetParam(ADDR_SPEED);
    param.acc = GetParam(ADDR_ACC);
    param.dec = GetParam(ADDR_DEC);
    param.max_speed = GetParam(ADDR_MAX_SPEED);
    param.min_speed = GetParam(ADDR_MIN_SPEED);
    param.adc = GetParam(ADDR_ADC_OUT);

//    Vcp.Printf("#AbsPos %X\n\r", Prm.curr_pos);
//    Vcp.Printf("#ElPos %X\n\r", Prm.el_pos);
//    Vcp.Printf("#MarkPos %X\n\r", Prm.mark_pos);
//    Vcp.Printf("#Speed %X\n\r", Prm.speed);
//    Vcp.Printf("#Acc %X\n\r", Prm.acc);
//    Vcp.Printf("#Dec %X\n\r", Prm.dec);
//    Vcp.Printf("#MaxSpeed %X\n\r", Prm.max_speed);
//    Vcp.Printf("#MinSpeed %X\n\r", Prm.min_speed);
//    Vcp.Printf("#Adc %X\n\r", Prm.adc);

#if (APP_MOTOR_DEBUG_INFO)
    Uart.Printf("AbsPos %X\n\r", param.curr_pos);
    Uart.Printf("ElPos %X\n\r", param.el_pos);
    Uart.Printf("MarkPos %X\n\r", param.mark_pos);
    Uart.Printf("Speed %X\n\r", param.speed);
    Uart.Printf("Acc %X\n\r", param.acc);
    Uart.Printf("Dec %X\n\r", param.dec);
    Uart.Printf("MaxSpeed %X\n\r", param.max_speed);
    Uart.Printf("MinSpeed %X\n\r", param.min_speed);
    Uart.Printf("Adc %X\n\r", param.adc);
#endif
}

#endif


#if 1 // ==== CMDs ====
uint8_t Motor_t::NOP()
{
    return Spi.WriteReadByte(0x00);
}

void Motor_t::SetParamBuf(uint8_t Addr, uint8_t *PBuf, uint8_t ALength)
{
//    Spi.DaisyTxRxData(id, PBuf, ALength, PRxBuf);
#if (APP_MOTOR_DEBUG_IO)
//    Uart.Printf("MC: set param %A\r", PTxBuf, 4, ' ');
#endif

}

void Motor_t::SetParam(uint8_t Addr, uint32_t Value)
{
    switch(Addr) {
        // 3 bytes Value
        case ADDR_ABS_POS:
        case ADDR_MARK:
        case ADDR_SPEED:
//            TxBuf[0] = Addr;
//            TxBuf[1] = (uint8_t)((0x00FF0000 & Value) >> 16);
//            TxBuf[2] = (uint8_t)((0x0000FF00 & Value) >> 8);
//            TxBuf[3] = (uint8_t)(0x000000FF & Value);
//            TxSize = 4;
            break;
        // 2 bytes Value
        case ADDR_EL_POS:
        case ADDR_ACC:
        case ADDR_DEC:
        case ADDR_MAX_SPEED:
        case ADDR_MIN_SPEED:
        case ADDR_FS_SPD:
        case ADDR_INT_SPEED:
        case ADDR_CONFIG:
        case ADDR_STATUS:
//            TxBuf[0] = Addr;
//            TxBuf[1] = (uint8_t)((0x0000FF00 & Value) >> 8);
//            TxBuf[2] = (uint8_t)(0x000000FF & Value);
//            TxSize = 3;
            break;

        // 1 bytes Value
        default:
//            TxBuf[0] = Addr;
//            TxBuf[1] = (uint8_t)(0x000000FF & Value);
//            TxSize = 2;
            break;
    } // switch
#if (APP_MOTOR_DEBUG_IO)
//    Uart.Printf("MC: tx %A\r", PTxBuf, TxSize, ' ');
#endif
//    Spi.DaisyTxRxData(id, PTxBuf, TxSize, PRxBuf);
}

uint32_t Motor_t::GetParam(uint8_t Addr)
{
    uint32_t param = 0;

    Spi.WriteReadByte(GET_PARAM | (0x1F & Addr));
    // TODO: Need to Fix answer by 1000 e.g.
    switch(Addr) {
        // 3 bytes Value
        case ADDR_ABS_POS:
        case ADDR_MARK:
        case ADDR_SPEED:
            param = Spi.ReadByte();
            param <<= 8;
            param |= Spi.ReadByte();
            param <<= 8;
            param |= Spi.ReadByte();
            break;
        // 2 bytes Value
        case ADDR_EL_POS:
        case ADDR_ACC:
        case ADDR_DEC:
        case ADDR_MAX_SPEED:
        case ADDR_MIN_SPEED:
        case ADDR_FS_SPD:
        case ADDR_INT_SPEED:
        case ADDR_CONFIG:
        case ADDR_STATUS:
            param = Spi.ReadByte();
            param <<= 8;
            param |= Spi.ReadByte();
            break;

        // 1 bytes Value
        default:
            param = Spi.ReadByte();
            break;
    } // switch
#if (APP_MOTOR_DEBUG_IO)
    Uart.Printf("MC: Addr %X, param %X\r", Addr, param);
#endif
    return param;
}

void Motor_t::Run(uint8_t Dir, uint32_t Speed)
{
//    uint8_t tmp;
//    tmp = RUN | (0x01 & Dir);
//    TxBuf[0] = tmp;
//    TxBuf[1] = ((0x00FF0000 & Speed) >> 16);
//    TxBuf[2] = ((0x0000FF00 & Speed) >> 8);
//    TxBuf[3] = (0x000000FF & Speed);

//    Spi.DaisyTxRxData(id, PTxBuf, 4, PRxBuf);
}

void Motor_t::StepClock(uint8_t Dir)
{
	Spi.WriteReadByte(STEP_CLOCK | (0x01 & Dir));
}

void Motor_t::Move(uint8_t Dir, uint32_t Step)
{
//	TxBuf[0] = MOVE | (0x01 & Dir);
//	TxBuf[1] = ((0x003F0000 & Step) >> 16);
//	TxBuf[2] = ((0x0000FF00 & Step) >> 8);
//	TxBuf[3] =  (0x000000FF & Step);
//	Spi.DaisyTxRxData(id, PTxBuf, 4, PRxBuf);
}

void Motor_t::GoTo(uint32_t Position)
{
//	TxBuf[0] = GOTO;
//	TxBuf[1] = ((0x003F0000 & Position) >> 16);
//	TxBuf[2] = ((0x0000FF00 & Position) >> 8);
//	TxBuf[3] =  (0x000000FF & Position);

//	Spi.DaisyTxRxData(id, PTxBuf, 4, PRxBuf);
}

void Motor_t::GoTo_Dir(uint8_t Dir, uint32_t Position)
{
//	TxBuf[0] = GOTODIR | (0x01 & Dir);
//	TxBuf[1] = ((0x003F0000 & Position) >> 16);
//	TxBuf[2] = ((0x0000FF00 & Position) >> 8);
//	TxBuf[3] =  (0x000000FF & Position);

//	Spi.DaisyTxRxData(id, PTxBuf, 4, PRxBuf);
}

void Motor_t::GoUntil(uint8_t Act, uint8_t Dir, uint32_t Speed)
{
//    TxBuf[0] = GOUNTIL | (0x08 & Act) | (0x01 & Dir);
//    TxBuf[1] = ((0x0F0000 & Speed) >> 16);
//    TxBuf[2] = ((0x00FF00 & Speed) >> 8);
//    TxBuf[3] =  (0x0000FF & Speed);
}

void Motor_t::ReleaseSW(uint8_t Act, uint8_t Dir)
{
    Spi.WriteReadByte(RELEASE | (0x08 & Act) | (0x01 & Dir));
}

void Motor_t::GoHome()
{
    Spi.WriteReadByte(GO_HOME);
}

void Motor_t::GoMark()
{
    Spi.WriteReadByte(GO_MARK);
}

void Motor_t::ResetPos()
{
	Spi.WriteReadByte(RESET_POS);
}

void Motor_t::ResetDevice()
{
    Spi.WriteReadByte(RESET_DEVICE);
}

void Motor_t::SoftStop()
{
    Spi.WriteReadByte(SOFT_STOP);
}

void Motor_t::HardStop()
{
    Spi.WriteReadByte(HARD_STOP);
}

void Motor_t::SoftHiZ()
{
	Spi.WriteReadByte(SOFT_HiZ);
}

void Motor_t::HardHiZ()
{
	Spi.WriteReadByte(HARD_HiZ);
}

void Motor_t::GetStatus(uint32_t *PValue)
{
//	TxBuf[0] = GET_STATUS;
//    TxBuf[1] = TxBuf[2] = TxBuf[3] = 0;
////    Spi.DaisyTxRxData(id, PTxBuf, 4, PRxBuf);
//    *PValue = RxBuf[0];
//    if(RxBuf[1] != 0) { *PValue <<= 8; *PValue |= RxBuf[1]; }
//    if(RxBuf[2] != 0) { *PValue <<= 8; *PValue |= RxBuf[2]; }
//    if(RxBuf[3] != 0) { *PValue <<= 8; *PValue |= RxBuf[3]; }

#if (APP_MOTOR_DEBUG_IO)
//    Uart.Printf("MC: status %A\r", PRxBuf, 4, ' ');
#endif
}
#endif

