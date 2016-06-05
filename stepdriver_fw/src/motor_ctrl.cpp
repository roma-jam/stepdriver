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
    return cmdType::Err;
}

Rslt_t Driver_t::Init()
{
    if(_IsInit)
        return VCP_RPL_OK;

    Spi.Init();
    NumberOfMotors = SPI_SLAVE_CNT;
    if(NumberOfMotors == 0)
    {
#if (APP_MOTOR_DRIVER_DEBUG)
        Uart.Printf("MC: Wrong SPI_SLAVE_CNT value\n\rDriver Not Init\n\r");
#endif
        return VCP_RPL_FAILURE;
    }

    for(uint8_t i=0; i<NumberOfMotors; i++)
        Motor[i].SetSleep();

    if(PThread == nullptr)
        PThread = chThdCreateStatic(waDriverThread, sizeof(waDriverThread), NORMALPRIO, (tfunc_t)DriverThread, NULL);

    PCmdBuf = Cmd;
    PAckBuf = Ack.Buf;
    CmdLength = 0;

    for(uint8_t i=0; i<NumberOfMotors; i++)
        Motor[i].SetReset();

    // wait until we powered on correctly
    uint32_t Timeout = 0;
    do
    {
        chThdSleepMilliseconds(100);
        Timeout++;
        if (Motor[DEFAULT_ID].isPowered())
        {
            _IsInit = true;
            return VCP_RPL_OK;
        }
    } while (Timeout < DRIVER_INIT_TIMEOUT);

    return VCP_RPL_INIT_TIMEOUT;
}

#if 1 // ========================= MOTOR DRIVER ================================
void Driver_t::Task()
{
    for(uint8_t i = 0; i < NumberOfMotors; i++)
    {
        if(Motor[i].State != Motor[i].NewState) Motor[i].State = Motor[i].NewState;
        switch (Motor[i].State)
        {
            case msOff:
                chThdSleepMilliseconds(999);
                break;

            case msReset:
#if (APP_MOTOR_DRIVER_DEBUG)
                Uart.Printf("MC: init\r");
#endif
                Motor[i].SetInit();
                break;

            case msInit:
                Motor[i].Init(i);
                Motor[i].UpdatePrm();
                Motor[i].SetIdle();
                if(Motor[i].Prm.max_speed != 0)
                {
#if (APP_MOTOR_DRIVER_DEBUG)
                    Uart.Printf("MC: driver init %u\r", i);
#endif
                    Motor[i].PoweredOn();
                }
                else
                {
#if (APP_MOTOR_DRIVER_DEBUG)
                    Uart.Printf("MC: powered off %u\r", i);
#endif
                }
                break;

            case msIdle:
                chThdSleepMilliseconds(999);
                break;

            case msTimeLapse:
                Motor[i].Move(1, App.StepSize);
                chThdSleepMilliseconds(App.TimeDelay);
                if(Motor[i].GetPosition() >= App.GlideTrackMaxStep)
                {
#if (APP_MOTOR_DRIVER_DEBUG)
                    Uart.Printf("MC: Timelapse end\r");
#endif
                    Motor[i].Stop();
                    Motor[i].SetGoHome();
                }
                break;

            case msSleep:
                chThdSleepMilliseconds(999);
                break;

            case msGoHome:
#if (APP_MOTOR_DRIVER_DEBUG)
                Uart.Printf("MC: goHome\r");
#endif
                chThdSleepMilliseconds(99);
                Motor[i].GoHome();
                do
                { // Wait
                    chThdSleepMilliseconds(501);
                } while(Motor[i].GetPosition() != 0);
#if (APP_MOTOR_DRIVER_DEBUG)
                Uart.Printf("MC: sleep\r");
#endif
                Motor[i].SetSleep();
                break;
        }
    }
}
#endif

void Driver_t::EndStop()
{
#if (APP_MOTOR_DRIVER_DEBUG)
    Uart.Printf("MC: End Stop\r");
#endif
    for(uint8_t i=0; i<NumberOfMotors; i++)
        Motor[i].Stop();
}


#if 1 // =========================== MOTOR LOW LEVEL ==========================
void Motor_t::Init(uint8_t AssignId)
{
    id = AssignId;
    ResetDevice();
    PTxBuf = TxBuf;
    PRxBuf = RxBuf;
}

void Motor_t::UpdatePrm()
{
    GetParams(ADDR_ABS_POS, &Prm.curr_pos);
//    Vcp.Printf("#AbsPos %X\n\r", Prm.curr_pos);
    GetParams(ADDR_EL_POS, &Prm.el_pos);
//    Vcp.Printf("#ElPos %X\n\r", Prm.el_pos);
    GetParams(ADDR_MARK, &Prm.mark_pos);
//    Vcp.Printf("#MarkPos %X\n\r", Prm.mark_pos);
    GetParams(ADDR_SPEED, &Prm.speed);
//    Vcp.Printf("#Speed %X\n\r", Prm.speed);
    GetParams(ADDR_ACC, &Prm.acc);
//    Vcp.Printf("#Acc %X\n\r", Prm.acc);
    GetParams(ADDR_DEC, &Prm.dec);
//    Vcp.Printf("#Dec %X\n\r", Prm.dec);
    GetParams(ADDR_MAX_SPEED, &Prm.max_speed);
//    Vcp.Printf("#MaxSpeed %X\n\r", Prm.max_speed);
    GetParams(ADDR_MIN_SPEED, &Prm.min_speed);
//    Vcp.Printf("#MinSpeed %X\n\r", Prm.min_speed);
    GetParams(ADDR_ADC_OUT, &Prm.adc);
//    Vcp.Printf("#Adc %X\n\r", Prm.adc);

#if (APP_MOTOR_DEBUG_INFO)
    Uart.Printf("AbsPos %X\n\r", Prm.curr_pos);
    Uart.Printf("ElPos %X\n\r", Prm.el_pos);
    Uart.Printf("MarkPos %X\n\r", Prm.mark_pos);
    Uart.Printf("Speed %X\n\r", Prm.speed);
    Uart.Printf("Acc %X\n\r", Prm.acc);
    Uart.Printf("Dec %X\n\r", Prm.dec);
    Uart.Printf("MaxSpeed %X\n\r", Prm.max_speed);
    Uart.Printf("MinSpeed %X\n\r", Prm.min_speed);
    Uart.Printf("Adc %X\n\r", Prm.adc);
#endif
}

#endif


#if 1 // ==== CMDs ====
uint8_t Motor_t::NOP()
{
    return Spi.DaisyTxRxByte(id, 0x00);
}

void Motor_t::SetParamBuf(uint8_t Addr, uint8_t *PBuf, uint8_t ALength)
{
    Spi.DaisyTxRxData(id, PBuf, ALength, PRxBuf);
#if (APP_MOTOR_DEBUG_IO)
    Uart.Printf("MC: set param %A\r", PTxBuf, 4, ' ');
#endif

}

void Motor_t::SetParam(uint8_t Addr, uint32_t Value)
{
    uint8_t TxSize = 1;
    switch(Addr) {
        // 3 bytes Value
        case ADDR_ABS_POS:
        case ADDR_MARK:
        case ADDR_SPEED:
            TxBuf[0] = Addr;
            TxBuf[1] = (uint8_t)((0x00FF0000 & Value) >> 16);
            TxBuf[2] = (uint8_t)((0x0000FF00 & Value) >> 8);
            TxBuf[3] = (uint8_t)(0x000000FF & Value);
            TxSize = 4;
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
            TxBuf[0] = Addr;
            TxBuf[1] = (uint8_t)((0x0000FF00 & Value) >> 8);
            TxBuf[2] = (uint8_t)(0x000000FF & Value);
            TxSize = 3;
            break;

        // 1 bytes Value
        default:
            TxBuf[0] = Addr;
            TxBuf[1] = (uint8_t)(0x000000FF & Value);
            TxSize = 2;
            break;
    } // switch
#if (APP_MOTOR_DEBUG_IO)
    Uart.Printf("MC: tx %A\r", PTxBuf, TxSize);
#endif
    Spi.DaisyTxRxData(id, PTxBuf, TxSize, PRxBuf);
}

void Motor_t::GetParams(uint8_t Addr, uint32_t* PValue)
{
    bool FirstSymbol = false;
    TxBuf[0] = GET_PARAM | (0x1F & Addr);
    TxBuf[1] = TxBuf[2] = TxBuf[3] = 0;
    Spi.DaisyTxRxData(id, PTxBuf, 4, PRxBuf);
    *PValue = RxBuf[0];
    // TODO: Need to Fix answer by 1000 e.g.
    switch(Addr) {
        // 3 bytes Value
        case ADDR_ABS_POS:
        case ADDR_MARK:
        case ADDR_SPEED:
            if(RxBuf[1] != 0) { *PValue <<= 8; *PValue |= RxBuf[1]; FirstSymbol = true; }
            if((RxBuf[2] != 0) || FirstSymbol) { *PValue <<= 8; *PValue |= RxBuf[2]; FirstSymbol = true; }
            if((RxBuf[3] != 0) || FirstSymbol) { *PValue <<= 8; *PValue |= RxBuf[3]; }
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
            if(RxBuf[1] != 0) { *PValue <<= 8; *PValue |= RxBuf[1]; FirstSymbol = true; }
            if((RxBuf[2] != 0) || FirstSymbol) { *PValue <<= 8; *PValue |= RxBuf[2]; }
            break;

        // 1 bytes Value
        default:
            if(RxBuf[1] != 0) { *PValue <<= 8; *PValue |= RxBuf[1]; }
            break;
    } // switch
#if (APP_MOTOR_DEBUG_IO)
    Uart.Printf("MC: param %A\r", PRxBuf, 4);
#endif
}

void Motor_t::Run(uint8_t Dir, uint32_t Speed)
{
    uint8_t tmp;
    tmp = RUN | (0x01 & Dir);
    TxBuf[0] = tmp;
    TxBuf[1] = ((0x00FF0000 & Speed) >> 16);
    TxBuf[2] = ((0x0000FF00 & Speed) >> 8);
    TxBuf[3] = (0x000000FF & Speed);

    Spi.DaisyTxRxData(id, PTxBuf, 4, PRxBuf);
}

void Motor_t::StepClock(uint8_t Dir)
{
	Spi.DaisyTxRxByte(id, (STEP_CLOCK | (0x01 & Dir)));
}

void Motor_t::Move(uint8_t Dir, uint32_t Step)
{
	TxBuf[0] = MOVE | (0x01 & Dir);
	TxBuf[1] = ((0x003F0000 & Step) >> 16);
	TxBuf[2] = ((0x0000FF00 & Step) >> 8);
	TxBuf[3] =  (0x000000FF & Step);
	Spi.DaisyTxRxData(id, PTxBuf, 4, PRxBuf);
}

void Motor_t::GoTo(uint32_t Position)
{
	TxBuf[0] = GOTO;
	TxBuf[1] = ((0x003F0000 & Position) >> 16);
	TxBuf[2] = ((0x0000FF00 & Position) >> 8);
	TxBuf[3] =  (0x000000FF & Position);

	Spi.DaisyTxRxData(id, PTxBuf, 4, PRxBuf);
}

void Motor_t::GoTo_Dir(uint8_t Dir, uint32_t Position)
{
	TxBuf[0] = GOTODIR | (0x01 & Dir);
	TxBuf[1] = ((0x003F0000 & Position) >> 16);
	TxBuf[2] = ((0x0000FF00 & Position) >> 8);
	TxBuf[3] =  (0x000000FF & Position);

	Spi.DaisyTxRxData(id, PTxBuf, 4, PRxBuf);
}

void Motor_t::GoUntil(uint8_t Act, uint8_t Dir, uint32_t Speed)
{
    TxBuf[0] = GOUNTIL | (0x08 & Act) | (0x01 & Dir);
    TxBuf[1] = ((0x0F0000 & Speed) >> 16);
    TxBuf[2] = ((0x00FF00 & Speed) >> 8);
    TxBuf[3] =  (0x0000FF & Speed);
}

void Motor_t::ReleaseSW(uint8_t Act, uint8_t Dir)
{
    Spi.DaisyTxRxByte(id, (RELEASE | (0x08 & Act) | (0x01 & Dir)));
}

void Motor_t::GoHome()
{
    Spi.DaisyTxRxByte(id, GO_HOME);
}

void Motor_t::GoMark()
{
    Spi.DaisyTxRxByte(id, GO_MARK);
}

void Motor_t::ResetPos()
{
	Spi.DaisyTxRxByte(id, RESET_POS);
}

void Motor_t::ResetDevice()
{
    Spi.DaisyTxRxByte(id, RESET_DEVICE);
}

void Motor_t::SoftStop()
{
    Spi.DaisyTxRxByte(id, SOFT_STOP);
}

void Motor_t::HardStop()
{
    Spi.DaisyTxRxByte(id, HARD_STOP);
}

void Motor_t::SoftHiZ()
{
	Spi.DaisyTxRxByte(id, SOFT_HiZ);
}

void Motor_t::HardHiZ()
{
	Spi.DaisyTxRxByte(id, HARD_HiZ);
}

void Motor_t::GetStatus(uint32_t *PValue)
{
	TxBuf[0] = GET_STATUS;
    TxBuf[1] = TxBuf[2] = TxBuf[3] = 0;
    Spi.DaisyTxRxData(id, PTxBuf, 4, PRxBuf);
    *PValue = RxBuf[0];
    if(RxBuf[1] != 0) { *PValue <<= 8; *PValue |= RxBuf[1]; }
    if(RxBuf[2] != 0) { *PValue <<= 8; *PValue |= RxBuf[2]; }
    if(RxBuf[3] != 0) { *PValue <<= 8; *PValue |= RxBuf[3]; }

#if (APP_MOTOR_DEBUG_IO)
    Uart.Printf("MC: status %A\r", PRxBuf, 4, ' ');
#endif
}
#endif

