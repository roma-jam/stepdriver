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
    if(strcasecmp(S, VCP_SET_PARAM_STRING) == 0)            return SetParam;
    if(strcasecmp(S, VCP_GET_PARAM_STRING) == 0)            return GetParam;
    if(strcasecmp(S, VCP_MOVE_STRING) == 0)                 return Move;
    if(strcasecmp(S, VCP_GOTO_STRING) == 0)                 return cmdGoTo;
    if(strcasecmp(S, VCP_GOTODIR_STRING) == 0)              return cmdGoToDir;
    if(strcasecmp(S, VCP_GOUNTIL_STRING) == 0)              return GoUntil;
    if(strcasecmp(S, VCP_RELEASE_STRING) == 0)              return ReleaseSW;
    if(strcasecmp(S, VCP_GO_HOME_STRING) == 0)              return GoHome;
    if(strcasecmp(S, VCP_GO_MARK_STRING) == 0)              return GoMark;
    if(strcasecmp(S, VCP_RESET_POS_STRING) == 0)            return ResetPos;
    if(strcasecmp(S, VCP_SOFT_HiZ_STRING) == 0)             return SoftHiZ;
    if(strcasecmp(S, VCP_HARD_HiZ_STRING) == 0)             return HardHiZ;
    if(strcasecmp(S, VCP_RESET_DEVICE_STRING) == 0)         return ResetDevice;
    if(strcasecmp(S, VCP_RUN_STRING) == 0)                  return Run;
    if(strcasecmp(S, VCP_STOP_STRING) == 0)                 return Stop;
    if(strcasecmp(S, VCP_STEP_CLOCK_STRING) == 0)           return StepClock;
    if(strcasecmp(S, VCP_SOFT_STOP_STRING) == 0)            return SoftStop;
    if(strcasecmp(S, VCP_HARD_STOP_STRING) == 0)            return HardStop;
    if(strcasecmp(S, VCP_GET_STATUS_STRING) == 0)           return GetStaus;
    if(strcasecmp(S, VCP_UPDATE_PARAM_STRING) == 0)         return UpdateParam;
    if(strcasecmp(S, VCP_CALIBRATE) == 0)                   return Calibrate;
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
        chThdSleepMilliseconds(999);
        if (Motor.isPowered())
        {
#if (APP_MOTOR_DRIVER_DEBUG)
            Uart.Printf("MC: powered up\r");
#endif
            return VCP_RPL_OK;
        }
    } while (Timeout++ < APP_MOTOR_DRIVER_INIT_TIMEOUT_S);

#if (APP_MOTOR_DRIVER_DEBUG)
    Uart.Printf("MC: powered failure\r");
#endif

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
        case msCalibrate:
        {
            chThdSleepMilliseconds(999);
        }
            break;

        case msPowerUp:
        {
#if (APP_MOTOR_DRIVER_DEBUG)
            Uart.Printf("MC: preparing\r");
#endif
            Motor.Init();
            Motor.UpdatePrm();
            if(Motor.param.max_speed != 0)
            {
                Motor.SetState(msIdle);
                Motor.PoweredOn();
                break;
            }

            Motor.SetState(msOff);
        }
            break;

        case msBusy:
        {
            if((Motor.GetStatus() & STATUS_FLAG_IDLE) == STATUS_FLAG_IDLE)
            {
                Uart.Printf("MC: Idle Flag\r");
                Motor.SetState(msIdle);
            }
            Uart.Printf("MC: Busy\r");
            chThdSleepMilliseconds(APP_MOTOR_BUSY_STATE_CHECK_MS);
        }
            break;
    }
}
#endif

void Driver_t::EndStop()
{
#if (APP_MOTOR_DRIVER_DEBUG)
    Uart.Printf("MC: End Stop\r");
#endif
}


#if 1 // =========================== MOTOR LOW LEVEL ==========================
void Motor_t::Init()
{
    ResetDevice();
}

void Motor_t::UpdatePrm()
{
    param.curr_pos = GetParam(L6470_ADDR_ABS_POS);
    param.el_pos = GetParam(L6470_ADDR_EL_POS);
    param.mark_pos = GetParam(L6470_ADDR_MARK);
    param.speed = GetParam(L6470_ADDR_SPEED);
    param.acc = GetParam(L6470_ADDR_ACC);
    param.dec = GetParam(L6470_ADDR_DEC);
    param.max_speed = GetParam(L6470_ADDR_MAX_SPEED);
    param.min_speed = GetParam(L6470_ADDR_MIN_SPEED);
    param.adc = GetParam(L6470_ADDR_ADC_OUT);

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

void Motor_t::SetParam(uint8_t Addr, uint32_t Value)
{
    switch(Addr) {
        // 3 bytes Value
        case L6470_ADDR_ABS_POS:
        case L6470_ADDR_MARK:
        case L6470_ADDR_SPEED:
            Spi.WriteReadByte(Addr);
            Spi.WriteReadByte((uint8_t)((0x00FF0000 & Value) >> 16));
            Spi.WriteReadByte((uint8_t)((0x0000FF00 & Value) >> 8));
            Spi.WriteReadByte((uint8_t)(0x000000FF & Value));
            break;
        // 2 bytes Value
        case L6470_ADDR_EL_POS:
        case L6470_ADDR_ACC:
        case L6470_ADDR_DEC:
        case L6470_ADDR_MAX_SPEED:
        case L6470_ADDR_MIN_SPEED:
        case L6470_ADDR_FS_SPD:
        case L6470_ADDR_INT_SPEED:
        case L6470_ADDR_CONFIG:
        case L6470_ADDR_STATUS:
            Spi.WriteReadByte(Addr);
            Spi.WriteReadByte((uint8_t)((0x0000FF00 & Value) >> 8));
            Spi.WriteReadByte((uint8_t)(0x000000FF & Value));
            break;

        // 1 bytes Value
        default:
            Spi.WriteReadByte(Addr);
            Spi.WriteReadByte((uint8_t)(0x000000FF & Value));
            break;
    } // switch
#if (APP_MOTOR_DEBUG_IO)
    Uart.Printf("MC: Addr %X, SetParam %X\r", Addr, Value);
#endif
}

uint32_t Motor_t::GetParam(uint8_t Addr)
{
    uint32_t param = 0;

    Spi.WriteReadByte(L6470_CMD_SET_PARAM | (0x1F & Addr));
    // TODO: Need to Fix answer by 1000 e.g.
    switch(Addr) {
        // 3 bytes Value
        case L6470_ADDR_ABS_POS:
        case L6470_ADDR_MARK:
        case L6470_ADDR_SPEED:
            param = Spi.ReadByte();
            param <<= 8;
            param |= Spi.ReadByte();
            param <<= 8;
            param |= Spi.ReadByte();
            break;
        // 2 bytes Value
        case L6470_ADDR_EL_POS:
        case L6470_ADDR_ACC:
        case L6470_ADDR_DEC:
        case L6470_ADDR_MAX_SPEED:
        case L6470_ADDR_MIN_SPEED:
        case L6470_ADDR_FS_SPD:
        case L6470_ADDR_INT_SPEED:
        case L6470_ADDR_CONFIG:
        case L6470_ADDR_STATUS:
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
    Spi.WriteReadByte(L6470_CMD_RUN | (0x01 & Dir));
    Spi.WriteReadByte((0x00FF0000 & Speed) >> 16);
    Spi.WriteReadByte((0x0000FF00 & Speed) >> 8);
    Spi.WriteReadByte(0x000000FF & Speed);
}

void Motor_t::StepClock(uint8_t Dir)
{
	Spi.WriteReadByte(L6470_CMD_STEP_CLOCK | (0x01 & Dir));
}

void Motor_t::Move(uint8_t Dir, uint32_t Step)
{
    Spi.WriteReadByte(L6470_CMD_MOVE | (0x01 & Dir));
    Spi.WriteReadByte((0x00FF0000 & Step) >> 16);
    Spi.WriteReadByte((0x0000FF00 & Step) >> 8);
    Spi.WriteReadByte(0x000000FF & Step);
}

void Motor_t::GoTo(uint32_t Position)
{
    Spi.WriteReadByte(L6470_CMD_GOTO);
    Spi.WriteReadByte((0x003F0000 & Position) >> 16);
    Spi.WriteReadByte((0x0000FF00 & Position) >> 8);
    Spi.WriteReadByte(0x000000FF & Position);
}

void Motor_t::GoTo_Dir(uint8_t Dir, uint32_t Position)
{
    Spi.WriteReadByte(L6470_CMD_GOTODIR | (0x01 & Dir));
    Spi.WriteReadByte((0x003F0000 & Position) >> 16);
    Spi.WriteReadByte((0x0000FF00 & Position) >> 8);
    Spi.WriteReadByte(0x000000FF & Position);
}

void Motor_t::GoUntil(uint8_t Act, uint8_t Dir, uint32_t Speed)
{
    Spi.WriteReadByte(L6470_CMD_GOUNTIL | (0x08 & Act) | (0x01 & Dir));
    Spi.WriteReadByte((0x000F0000 & Speed) >> 16);
    Spi.WriteReadByte((0x0000FF00 & Speed) >> 8);
    Spi.WriteReadByte(0x000000FF & Speed);
}

void Motor_t::ReleaseSW(uint8_t Act, uint8_t Dir)
{
    Spi.WriteReadByte(L6470_CMD_RELEASE | (0x08 & Act) | (0x01 & Dir));
}

void Motor_t::GoHome()
{
    Spi.WriteReadByte(L6470_CMD_GO_HOME);
}

void Motor_t::GoMark()
{
    Spi.WriteReadByte(L6470_CMD_GO_MARK);
}

void Motor_t::ResetPos()
{
	Spi.WriteReadByte(L6470_CMD_RESET_POS);
}

void Motor_t::ResetDevice()
{
    Spi.WriteReadByte(L6470_CMD_RESET_DEVICE);
}

void Motor_t::SoftStop()
{
    Spi.WriteReadByte(L6470_CMD_SOFT_STOP);
}

void Motor_t::HardStop()
{
    Spi.WriteReadByte(L6470_CMD_HARD_STOP);
}

void Motor_t::SoftHiZ()
{
	Spi.WriteReadByte(L6470_CMD_SOFT_HiZ);
}

void Motor_t::HardHiZ()
{
	Spi.WriteReadByte(L6470_CMD_HARD_HiZ);
}

uint32_t Motor_t::GetStatus()
{
    uint32_t Status;
    Spi.WriteReadByte(L6470_CMD_GET_STATUS);
    Status = Spi.ReadByte();
    Status <<= 8;
    Status = Spi.ReadByte();

#if (APP_MOTOR_DEBUG_IO)
    Uart.Printf("MC: Status %X\r", Status);
#endif
    return Status;
}
#endif

