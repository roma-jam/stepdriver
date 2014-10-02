/*
 * motor_ctrl.cpp
 *
 *  Created on: 19 дек. 2013 г.
 *      Author: r.leonov
 */



#include "motor_ctrl.h"
#include "vcp.h"


// Macros
#define PRINT_MSG(S)       { Uart.Printf(S); }

Driver_t Driver;

static WORKING_AREA(waDriverThread, 128);
__attribute__ ((__noreturn__))
static void DriverThread(void *arg) {
    chRegSetThreadName("Motor");
    while(1) Driver.Task();
}

Rslt_t Driver_t::Init() {
    Spi.Init();
    PThread = chThdCreateStatic(waDriverThread, sizeof(waDriverThread), NORMALPRIO, (tfunc_t)DriverThread, NULL);
    NumberOfMotors = SPI_SLAVE_CNT;
    PCmdBuf = Cmd;
    PAckBuf = Ack.Buf;
    CmdLength = 0;
    if(NumberOfMotors != 0) {
        for(uint8_t i=0; i<NumberOfMotors; i++) Motor[i].SetState(msOff);
        Uart.Printf("DriverInit\n\r");
        _IsInit = true;
        return VCP_RPL_OK;
    }
    Vcp.Printf("Wrong SPI_SLAVE_CNT value\n\rDriver Not Init\n\r");
    return VCP_RPL_FAILURE;
}

#if 1 // ==== Task ====
void Driver_t::Task() {
    for(uint8_t i = 0; i < NumberOfMotors; i++) {
        if(Motor[i].State != Motor[i].NewState) Motor[i].State = Motor[i].NewState;
        switch (Motor[i].State) {
            case msOff:
                Motor[i].SetState(msInit);
                Uart.Printf("msInit\r");
                break;

            case msInit:
                Vcp.Printf("#DriverInit\n\r");
                Motor[i].Init(i);
                Motor[i].UpdatePrm();
                Motor[i].SetState(msIdle);
                break;

            case msIdle:
//                Motor[i].Move(1, 100);
                chThdSleepMilliseconds(999);
                break;
            case msSleep:
                chThdSleepMilliseconds(999);
                break;
        }
    }
}
#endif

#if 0 // old
            case SET_PARAM:
                Ack.CmdID++;
                if(CmdLength < 1) Ack.Len = 0xFF;
                else {
                    uint32_t NewValue=0;
                    uint8_t Addr=0, tmp=0;
                    Uart.Printf("SET_PARAM\r");
                    do {
                        if(!TryConvertToDigit(*Ptr, &tmp)) { Uart.Printf("Addr Error\r"); break; }
                        Addr <<= 4;
                        Addr |= tmp;
                        Ptr++;
                        CmdLength--;
                    } while(*Ptr != ',');
                    Ptr++;
                    CmdLength--;
                    do {
                        if(!TryConvertToDigit(*Ptr++, &tmp)) { Uart.Printf("Value Error\r"); break; }
                        NewValue <<= 4;
                        NewValue |= tmp;
                        CmdLength--;
                    } while(CmdLength != 0);
                    Uart.Printf("Addr=%X, Value=%X\r", Addr, NewValue);
                    Motor[Ack.MtrID].SetParam(Addr, NewValue);
                    Ack.Len = 0x00;
                }
                Vcp.Printf("&%u,%X,%X\r\n", Ack.MtrID, Ack.CmdID, Ack.Len);
                break;

            case GET_PARAM:
                Ack.CmdID++;
                if(CmdLength < 1) {
                    Ack.Len = 0xFF;
                    Vcp.Printf("&%u,%X,%X\r\n", Ack.MtrID, Ack.CmdID, Ack.Len);
                }
                else {
                    uint32_t Rep;
                    uint8_t Addr=0, tmp=0;
                    Uart.Printf("GET_PARAM\r");
                    do {
                        if(!TryConvertToDigit(*Ptr++, &tmp)) { Uart.Printf("Step Not Number\r"); break; }
                        Addr <<= 4;
                        Addr |= tmp;
                        CmdLength--;
                    } while(CmdLength > 0);
                    Motor[Ack.MtrID].GetParams(Addr, &Rep);
                    Vcp.Printf("&%u,%X,%X,%X\r\n", Ack.MtrID, Ack.CmdID, Addr, Rep);
                }
                break;

            case GOTO:
                Ack.CmdID++;
                if(CmdLength < 1) Ack.Len = 0xFF;
                else {
//                    Motor[Ack.MtrID].GoTo();
                }
                Uart.Printf("GOTO\r");
                break;

            case GOTODIR:
                Uart.Printf("GOTODIR\r");
                break;

            case GOUNTIL:
                Uart.Printf("GOUNTIL\r");
                break;

            case RELEASE:
                Ack.CmdID++;
                if(CmdLength < 1) Ack.Len = 0xFF;
                else {
//                    Motor[Ack.MtrID].ReleaseSW();
                    Uart.Printf("RELEASE\r");
                }

                break;

            case GO_HOME:
                Ack.CmdID++;
                Motor[Ack.MtrID].GoHome();
                Ack.Len = 0x00;
                Uart.Printf("GO_HOME\r");
                Vcp.Printf("&%u,%X,%X\r\n", Ack.MtrID, Ack.CmdID, Ack.Len);
                break;

            case GO_MARK:
                Ack.CmdID++;
                Motor[Ack.MtrID].GoMark();
                Ack.Len = 0x00;
                Uart.Printf("GO_MARK\r");
                Vcp.Printf("&%u,%X,%X\r\n", Ack.MtrID, Ack.CmdID, Ack.Len);
                break;

            case RESET_POS:
                Ack.CmdID++;
                Motor[Ack.MtrID].ResetPos();
                Ack.Len = 0x00;
                Uart.Printf("RESET_POS\r");
                Vcp.Printf("&%u,%X,%X\r\n", Ack.MtrID, Ack.CmdID, Ack.Len);
                break;

            case SOFT_HiZ:
                Ack.CmdID++;
                Motor[Ack.MtrID].SoftHiZ();
                Ack.Len = 0x00;
                Uart.Printf("SOFT_HiZ\r");
                Vcp.Printf("&%u,%X,%X\r\n", Ack.MtrID, Ack.CmdID, Ack.Len);
                break;

            case HARD_HiZ:
                Ack.CmdID++;
                Motor[Ack.MtrID].HardHiZ();
                Ack.Len = 0x00;
                Uart.Printf("HARD_HiZ\r");
                Vcp.Printf("&%u,%X,%X\r\n", Ack.MtrID, Ack.CmdID, Ack.Len);
                break;

            case RESET_DEVICE:
                Ack.CmdID++;
                Motor[Ack.MtrID].ResetDevice();
                Ack.Len = 0x00;
                Uart.Printf("RESET_DEVICE\r");
                Vcp.Printf("&%u,%X,%X\r\n", Ack.MtrID, Ack.CmdID, Ack.Len);
                break;

            case RUN:
                Ack.CmdID++;
                if(CmdLength < 1) Ack.Len = 0xFF;
                else {
                    uint32_t Speed=0;
                    uint8_t Dir=0, tmp=0;
                    Uart.Printf("RUN\r");
                    TryConvertToDigit(*Ptr++, &Dir);
                    CmdLength--;
                    if(Dir > 1) { Uart.Printf("Error Dir\r"); break; }
                    if(*Ptr++ != ',') { Uart.Printf("Wromg Dir delimeter\r"); break; }
                    CmdLength--;
                    do {
                        if(!TryConvertToDigit(*Ptr++, &tmp)) { Uart.Printf("Speed Not Number\r"); break; }
                        Speed <<= 4;
                        Speed |= tmp;
                        CmdLength--;
                    } while(CmdLength > 0);
                    Uart.Printf("dir=%u, speed=%X\r", Dir, Speed);
                    Motor[Ack.MtrID].Run(Dir, Speed);
                    Ack.Len = 0x00;
                }
                Vcp.Printf("&%u,%X,%X\r\n", Ack.MtrID, Ack.CmdID, Ack.Len);
                break;

            case STEP_CLOCK:
                Uart.Printf("STEP_CLOCK\r");
                break;

            case SOFT_STOP:
                Ack.CmdID++;
                Motor[Ack.MtrID].SoftStop();
                Ack.Len = 0x00;
                Uart.Printf("SOFT_STOP\r");
                Vcp.Printf("&%u,%X,%X\r\n", Ack.MtrID, Ack.CmdID, Ack.Len);
                break;

            case MOVE:
                Ack.CmdID++;
                if(CmdLength < 1) Ack.Len = 0xFF;
                else {
                    uint32_t Steps=0;
                    uint8_t Step=0;
                    uint8_t Dir;
                    Uart.Printf("MOVE\r");

                    TryConvertToDigit(*Ptr++, &Dir);
                    CmdLength--;
                    if(Dir > 1) { Uart.Printf("Error Dir\r"); break; }
                    if(*Ptr++ != ',') { Uart.Printf("Wromg Dir delimeter\r"); break; }
                    CmdLength--;
                    do {
                        if(!TryConvertToDigit(*Ptr++, &Step)) { Uart.Printf("Step Not Number\r"); break; }
//                        Steps = (Steps*10) + Step;
                        Steps <<= 4;
                        Steps |= Step;
                        CmdLength--;
                    } while(CmdLength > 0);
                    Uart.Printf("dir=%u, step=%X\r", Dir, Steps);
                    Motor[Ack.MtrID].Move(Dir, Steps);
                    Ack.Len = 0x00;
                }
                Vcp.Printf("&%u,%X,%X\r\n", Ack.MtrID, Ack.CmdID, Ack.Len);
                break;

            case HARD_STOP:
                Ack.CmdID++;
                Motor[Ack.MtrID].HardStop();
                Ack.Len = 0x00;
                Uart.Printf("HARD_STOP\r");
                Vcp.Printf("&%u,%X,%X\r\n", Ack.MtrID, Ack.CmdID, Ack.Len);
                break;

            case GET_STATUS:
                Ack.CmdID++;
                uint32_t Status;
                Motor[Ack.MtrID].GetStatus(&Status);
                Ack.Len = 0x00;
                Uart.Printf("GET_STATUS\r");
                Vcp.Printf("&%u,%X,%X\r\n", Ack.MtrID, Ack.CmdID, Status);
                break;

            default:
                Uart.Printf("CmdID Error\r");
                break;
        } // switch
#endif

#if 1 // ==== Motors ====
void Motor_t::Init(uint8_t AssignId) {
    id = AssignId;
    ResetDevice();
    PTxBuf = TxBuf;
    PRxBuf = RxBuf;
}

void Motor_t::UpdatePrm() {
    GetParams(ADDR_ABS_POS, &Prm.curr_pos);
//    Uart.Printf("%u, abs_pos = %X\r", id, Prm.curr_pos);
    GetParams(ADDR_EL_POS, &Prm.el_pos);
//    Uart.Printf("%u, el_pos = %X\r", id, Prm.el_pos);
    GetParams(ADDR_MARK, &Prm.mark_pos);
//    Uart.Printf("%u, mark_pos = %X\r", id, Prm.mark_pos);
    GetParams(ADDR_SPEED, &Prm.speed);
//    Uart.Printf("%u, speed = %X\r", id, Prm.speed);
    GetParams(ADDR_ACC, &Prm.acc);
//    Uart.Printf("%u, acc = %X\r", id, Prm.acc);
    GetParams(ADDR_DEC, &Prm.dec);
//    Uart.Printf("%u, dec = %X\r", id, Prm.dec);
    GetParams(ADDR_MAX_SPEED, &Prm.max_speed);
    Vcp.Printf("#Max_speed %X\r", Prm.max_speed);
    GetParams(ADDR_MIN_SPEED, &Prm.min_speed);
//    Uart.Printf("%u, min_speed = %X\r", id, Prm.min_speed);
    GetParams(ADDR_ADC_OUT, &Prm.adc);
//    Uart.Printf("%u, adc = %X\r", id, Prm.adc);
}
#endif


#if 1 // ==== CMDs ====
uint8_t Motor_t::NOP() {
    return Spi.DaisyTxRxByte(id, 0x00);
}

void Motor_t::SetParamBuf(uint8_t Addr, uint8_t *PBuf, uint8_t ALength) {
    Spi.DaisyTxRxData(id, PBuf, ALength, PRxBuf);
    Uart.Printf("Tx: %A\r", PTxBuf, 4, ' ');
}

void Motor_t::SetParam(uint8_t Addr, uint32_t Value) {
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
//    Uart.Printf("Tx: %A\r", PTxBuf, TxSize);
    Spi.DaisyTxRxData(id, PTxBuf, TxSize, PRxBuf);
}

void Motor_t::GetParams(uint8_t Addr, uint32_t* PValue) {
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
//    Uart.Printf("rx: %A\r", PRxBuf, 4);
}

void Motor_t::Run(uint8_t Dir, uint32_t Speed) {
    uint8_t tmp;
//    uint8_t SizeTx = 2, i = 0;
    tmp = RUN | (0x01 & Dir);
    TxBuf[0] = tmp;
    TxBuf[1] = ((0x00FF0000 & Speed) >> 16);
//    if(TxBuf[i] != 0) { SizeTx++, i++; }
    TxBuf[2] = ((0x0000FF00 & Speed) >> 8);
//    if(TxBuf[i] != 0) { SizeTx++, i++; }
    TxBuf[3] = (0x000000FF & Speed);

    Spi.DaisyTxRxData(id, PTxBuf, 4, PRxBuf);
}

void Motor_t::StepClock(uint8_t Dir) {
	Spi.DaisyTxRxByte(id, (STEP_CLOCK | (0x01 & Dir)));
}

void Motor_t::Move(uint8_t Dir, uint32_t Step) {
	TxBuf[0] = MOVE | (0x01 & Dir);
	TxBuf[1] = ((0x003F0000 & Step) >> 16);
	TxBuf[2] = ((0x0000FF00 & Step) >> 8);
	TxBuf[3] =  (0x000000FF & Step);
	Uart.Printf("%A\r", TxBuf, 4, ' ');
	Spi.DaisyTxRxData(id, PTxBuf, 4, PRxBuf);
}

void Motor_t::GoTo(uint32_t Position) {
	TxBuf[0] = GOTO;
	TxBuf[1] = ((0x003F0000 & Position) >> 16);
	TxBuf[2] = ((0x0000FF00 & Position) >> 8);
	TxBuf[3] =  (0x000000FF & Position);

	Spi.DaisyTxRxData(id, PTxBuf, 4, PRxBuf);
}

void Motor_t::GoTo_Dir(uint8_t Dir, uint32_t Position) {
	TxBuf[0] = GOTODIR | (0x01 & Dir);
	TxBuf[1] = ((0x003F0000 & Position) >> 16);
	TxBuf[2] = ((0x0000FF00 & Position) >> 8);
	TxBuf[3] =  (0x000000FF & Position);

	Spi.DaisyTxRxData(id, PTxBuf, 4, PRxBuf);
}

void Motor_t::GoUntil(uint8_t Act, uint8_t Dir, uint32_t Speed) {
    TxBuf[0] = GOUNTIL | (0x08 & Act) | (0x01 & Dir);
    TxBuf[1] = ((0x0F0000 & Speed) >> 16);
    TxBuf[2] = ((0x00FF00 & Speed) >> 8);
    TxBuf[3] =  (0x0000FF & Speed);
}

void Motor_t::ReleaseSW(uint8_t Act, uint8_t Dir) {
    Spi.DaisyTxRxByte(id, (RELEASE | (0x08 & Act) | (0x01 & Dir)));
}

void Motor_t::GoHome() {
    Spi.DaisyTxRxByte(id, GO_HOME);
}

void Motor_t::GoMark() {
    Spi.DaisyTxRxByte(id, GO_MARK);
}

void Motor_t::ResetPos() {
	Spi.DaisyTxRxByte(id, RESET_POS);
}

void Motor_t::ResetDevice() {
    Spi.DaisyTxRxByte(id, RESET_DEVICE);
}

void Motor_t::SoftStop() {
    Spi.DaisyTxRxByte(id, SOFT_STOP);
}

void Motor_t::HardStop() {
    Spi.DaisyTxRxByte(id, HARD_STOP);
}

void Motor_t::SoftHiZ() {
	Spi.DaisyTxRxByte(id, SOFT_HiZ);
}

void Motor_t::HardHiZ() {
	Spi.DaisyTxRxByte(id, HARD_HiZ);
}

void Motor_t::GetStatus(uint32_t *PValue) {
	TxBuf[0] = GET_STATUS;
    TxBuf[1] = TxBuf[2] = TxBuf[3] = 0;
    Spi.DaisyTxRxData(id, PTxBuf, 4, PRxBuf);
    *PValue = RxBuf[0];
    if(RxBuf[1] != 0) { *PValue <<= 8; *PValue |= RxBuf[1]; }
    if(RxBuf[2] != 0) { *PValue <<= 8; *PValue |= RxBuf[2]; }
    if(RxBuf[3] != 0) { *PValue <<= 8; *PValue |= RxBuf[3]; }
    Uart.Printf("status: %A\r", PRxBuf, 4, ' ');
}
#endif

