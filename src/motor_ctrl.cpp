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
    if(_IsInit) return VCP_RPL_OK;
    Spi.Init();
    NumberOfMotors = SPI_SLAVE_CNT;
    if(NumberOfMotors == 0) { Uart.Printf("Wrong SPI_SLAVE_CNT value\n\rDriver Not Init\n\r"); return VCP_RPL_FAILURE; }
    for(uint8_t i=0; i<NumberOfMotors; i++) Motor[i].SetState(msSleep);
    if(PThread == nullptr) PThread = chThdCreateStatic(waDriverThread, sizeof(waDriverThread), NORMALPRIO, (tfunc_t)DriverThread, NULL);
    PCmdBuf = Cmd;
    PAckBuf = Ack.Buf;
    CmdLength = 0;
    for(uint8_t i=0; i<NumberOfMotors; i++) Motor[i].SetState(msOff);
    // wait until we powered on correctly
    uint32_t Timeout = 0;
    do {
        chThdSleepMilliseconds(100);
        Timeout++;
        if (Motor[DEFAULT_ID].isPowered()) {
            _IsInit = true;
            return VCP_RPL_OK;
        }
    } while (Timeout < DRIVER_INIT_TIMEOUT);
    return VCP_RPL_INIT_TIMEOUT;
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
                Motor[i].Init(i);
                Motor[i].UpdatePrm();
                Motor[i].SetState(msIdle);
                if(Motor[i].Prm.max_speed != 0) {
                    Motor[i].PoweredOn();
                    Uart.Printf("#DriverInit\r");
                }
                else Uart.Printf("#PoweredOff\r");
                break;

            case msIdle:
//                Motor[i].Run(1, 0x9508);
//                chThdSleepMilliseconds(1999);
//                Motor[i].Run(0, 0x9508);
                chThdSleepMilliseconds(1999);
                break;
            case msSleep:
                chThdSleepMilliseconds(999);
                break;
        }
    }
}
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

