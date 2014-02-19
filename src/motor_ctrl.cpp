/*
 * motor_ctrl.cpp
 *
 *  Created on: 19 дек. 2013 г.
 *      Author: r.leonov
 */



#include "motor_ctrl.h"
#include "vcp.h"

#define UPDATE_PRM_PRINT

#define NEXT_BYTE(A,B)  {A++;B--;}

Driver_t Driver;

static WORKING_AREA(waDriverThread, 128);
__attribute__ ((__noreturn__))
static void DriverThread(void *arg) {
    chRegSetThreadName("Motor");
    while(1) Driver.Task();
}

static inline bool TryConvertToDigit(uint8_t b, uint8_t *p) {
    if((b >= '0') and (b <= '9')) {
        *p = b - '0';
        return true;
    }
    else if((b >= 'A') and (b <= 'F')) {
        *p = 0x0A + b - 'A';
        return true;
    }
    else return false;
}

static inline bool IsDelimeter(uint8_t b) {
    return (b == ',');
}

void Driver_t::Init() {
    Spi.Init();
    PThread = chThdCreateStatic(waDriverThread, sizeof(waDriverThread), NORMALPRIO, (tfunc_t)DriverThread, NULL);
    NumberOfMotors = SPI_SLAVE_CNT;
    PCmdBuf = Cmd;
    PAckBuf = &Ack;
    CmdLength = 0;
    if(NumberOfMotors != 0) {
        for(uint8_t i=0; i<NumberOfMotors; i++) Motor[i].SetState(msOff);
        Uart.Printf("DriverInit\r");
    }
    else {
        Uart.Printf("Wrong SPI_SLAVE_CNT value\rDriver Not Init\r");
    }

}

#if 1 // ==== Task ====
void Driver_t::Task() {
    for(uint8_t i = 0; i < NumberOfMotors; i++) {
        if(Motor[i].State != Motor[i].NewState) Motor[i].State = Motor[i].NewState;
        switch (Motor[i].State) {
            case msOff:
//                Motor[i].SetState(msInit);
//                Uart.Printf("msInit\r");
                break;

            case msInit:
                Uart.Printf("%u msInit\r", i);
                Motor[i].Init(i);
                Motor[i].UpdatePrm();
                Motor[i].SetState(msIdle);
                Uart.Printf("%u msIdle\r", i);
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

uint8_t Driver_t::CmdHandle() {
    uint8_t Rslt = FAILURE;
    uint8_t *P = Cmd;
    uint8_t Len = CmdLength;
//    Uart.Printf("%A\r", P, Len, ' ');
    switch(*P) {
        case '#':
            NEXT_BYTE(P,Len)
//            Uart.Printf("Cmd %A\r", P, Len, ' ');
            Rslt = ICmdExecute(P, Len);
            break;
        case '$':
            NEXT_BYTE(P,Len)
//            Uart.Printf("Srv %A\r", P, Len, ' ');
            Rslt = ISrvExecute(P, Len);
            break;
        default:
            Uart.Printf("Start symbol error\r");
            break;
    }
    IResetBuf();
    return Rslt;
}

uint8_t Driver_t::ISrvExecute(uint8_t *Ptr, uint8_t ALen) {
    if(ALen > MAX_SRV_LEN) return LENGTH_ERROR;
    IResetCmdValues();
    uint8_t *P = (uint8_t*)&CmdValues;
    for(uint8_t i=0; i<ALen; i++, Ptr++) {
        if(IsDelimeter(*Ptr)) P++;
        else {
            if(TryConvertToDigit(*Ptr, Ptr)) {
                *P <<= 4;
                *P |= *Ptr;
            }
            else return CMD_ERROR;
        } // not delimeter
    } // for
    Uart.Printf("SrvID=%X, Value=%X\r", CmdValues.SrvID, CmdValues.SrvValue);
    // Prepare reply
    Ack.MtrID = 0x0F;
    Ack.CmdID = CmdValues.SrvID+1;
    Ack.Err = OK;
    return OK;
}

uint8_t Driver_t::ICmdExecute(uint8_t *Ptr, uint8_t ALen) {
    if(ALen > MAX_CMD_LEN) return LENGTH_ERROR;
    IResetCmdValues();
    uint8_t *P = (uint8_t*)&CmdValues;
    uint8_t DelCnt = 0;
    for(uint8_t i=0; i<ALen; i++, Ptr++) {
        if(IsDelimeter(*Ptr)) { P++; DelCnt++; }
        else {
            if(DelCnt < 3) {
                if(TryConvertToDigit(*Ptr, Ptr)) {
                    *P <<= 4;
                    *P |= *Ptr;
                } // 8 bit
                else return CMD_ERROR;
            } // 32 bit
            else {
                if(TryConvertToDigit(*Ptr, Ptr)) {
                    *((uint32_t*)P) <<= 4;
                    *((uint32_t*)P) |= (*Ptr & 0x0F);
                } // 32 bit
                else return CMD_ERROR;
            } // 32 bit
        } // not delimeter
    }// for
    Uart.Printf("MtrID=%X, CmdID=%X, Addr=%X, Value=%X\r", CmdValues.MtrID, CmdValues.CmdID, CmdValues.Addr, CmdValues.Value);
    Ack.MtrID = CmdValues.MtrID;
    Ack.CmdID = CmdValues.CmdID+1;
    Ack.Err = NO_ERROR;
#if 1 //==== Cmd Execute ====
    switch (CmdValues.CmdID) {
        case SET_PARAM:
            Uart.Printf("SET_PARAM\r");
//            Motor[Ack.MtrID].SetParam(CmdValues.Addr, CmdValues.Value);
//            Uart.Printf("&%u,%X,%X\r\n", Ack.MtrID, Ack.CmdID, Ack.Err);
            break;

        case GET_PARAM:
            Uart.Printf("GET_PARAM\r");
//            uint32_t Rep;
//            Motor[Ack.MtrID].GetParams(CmdValues.Addr, &Rep);
//            Uart.Printf("&%u,%X,%X,%X\r\n", Ack.MtrID, Ack.CmdID, CmdValues.Addr, Rep);
            break;

        case GOTO:
            Uart.Printf("GOTO\r");
            break;

        case GOTODIR:
            Uart.Printf("GOTODIR\r");
            break;

        case GOUNTIL:
            Uart.Printf("GOUNTIL\r");
            break;

        case RELEASE:
//            Motor[Ack.MtrID].ReleaseSW();
            Uart.Printf("RELEASE\r");
            break;

        case GO_HOME:
            Uart.Printf("GO_HOME\r");
//            Motor[Ack.MtrID].GoHome();
//            Uart.Printf("&%u,%X,%X\r\n", Ack.MtrID, Ack.CmdID, Ack.Len);
            break;

        case GO_MARK:
            Uart.Printf("GO_MARK\r");
//            Motor[Ack.MtrID].GoMark();
//            Uart.Printf("&%u,%X,%X\r\n", Ack.MtrID, Ack.CmdID, Ack.Len);
            break;

        case RESET_POS:
            Uart.Printf("RESET_POS\r");
//            Motor[Ack.MtrID].ResetPos();
//            Uart.Printf("&%u,%X,%X\r\n", Ack.MtrID, Ack.CmdID, Ack.Len);
            break;

        case SOFT_HiZ:
            Uart.Printf("SOFT_HiZ\r");
//            Motor[Ack.MtrID].SoftHiZ();
//            Uart.Printf("&%u,%X,%X\r\n", Ack.MtrID, Ack.CmdID, Ack.Len);
            break;

        case HARD_HiZ:
            Uart.Printf("HARD_HiZ\r");
//            Motor[Ack.MtrID].HardHiZ();
//            Uart.Printf("&%u,%X,%X\r\n", Ack.MtrID, Ack.CmdID, Ack.Len);
            break;

        case RESET_DEVICE:
            Uart.Printf("RESET_DEVICE\r");
//            Motor[Ack.MtrID].ResetDevice();
//            Uart.Printf("&%u,%X,%X\r\n", Ack.MtrID, Ack.CmdID, Ack.Len);
            break;

        case RUN:
            Uart.Printf("RUN\r");
//            Motor[Ack.MtrID].Run(Dir, Speed);
//            Uart.Printf("&%u,%X,%X\r\n", Ack.MtrID, Ack.CmdID, Ack.Len);
            break;

        case STEP_CLOCK:
            Uart.Printf("STEP_CLOCK\r");
            break;

        case SOFT_STOP:
            Uart.Printf("SOFT_STOP\r");
//            Motor[Ack.MtrID].SoftStop();
//            Uart.Printf("&%u,%X,%X\r\n", Ack.MtrID, Ack.CmdID, Ack.Len);
            break;

        case MOVE:
            Uart.Printf("MOVE\r");
//            Motor[Ack.MtrID].Move(Dir, Steps);
//            Uart.Printf("&%u,%X,%X\r\n", Ack.MtrID, Ack.CmdID, Ack.Len);
            break;

        case HARD_STOP:
            Uart.Printf("HARD_STOP\r");
//            Motor[Ack.MtrID].HardStop();
//            Uart.Printf("&%u,%X,%X\r\n", Ack.MtrID, Ack.CmdID, Ack.Len);
            break;

        case GET_STATUS:
            Uart.Printf("GET_STATUS\r");
//            Motor[Ack.MtrID].GetStatus(&Status);
//            Uart.Printf("&%u,%X,%X\r\n", Ack.MtrID, Ack.CmdID, Status);
            break;

        default:
            Uart.Printf("CmdID Error\r");
            break;
    } // switch
#endif
    return OK;
}

bool Driver_t::ISetNewMotorsNumber(uint8_t NewNumber){
    if(NewNumber == NumberOfMotors) return false;
    else {
        for(uint8_t i=0; i<NumberOfMotors; i++) Motor[i].SetState(msOff);
        NumberOfMotors = NewNumber;
        Uart.Printf("NumberOfMotors=%u\r", NumberOfMotors);
        for(uint8_t i=0; i<NumberOfMotors; i++) Motor[i].SetState(msInit);
    }
    return true;
}


#if 1 // ==== Motors ====
void Motor_t::Init(uint8_t AssignId) {
    id = AssignId;
    ResetDevice();
    PTxBuf = TxBuf;
    PRxBuf = RxBuf;
}

void Motor_t::UpdatePrm() {
    GetParams(ADDR_ABS_POS, &Prm.curr_pos);
    GetParams(ADDR_EL_POS, &Prm.el_pos);
    GetParams(ADDR_MARK, &Prm.mark_pos);
    GetParams(ADDR_SPEED, &Prm.speed);
    GetParams(ADDR_ACC, &Prm.acc);
    GetParams(ADDR_DEC, &Prm.dec);
    GetParams(ADDR_MAX_SPEED, &Prm.max_speed);
    GetParams(ADDR_MIN_SPEED, &Prm.min_speed);
    GetParams(ADDR_ADC_OUT, &Prm.adc);
#ifdef UPDATE_PRM_PRINT
//    Uart.Printf("%u, abs_pos = %X\r", id, Prm.curr_pos);
//    Uart.Printf("%u, el_pos = %X\r", id, Prm.el_pos);
//    Uart.Printf("%u, mark_pos = %X\r", id, Prm.mark_pos);
//    Uart.Printf("%u, speed = %X\r", id, Prm.speed);
//    Uart.Printf("%u, acc = %X\r", id, Prm.acc);
//    Uart.Printf("%u, dec = %X\r", id, Prm.dec);
//    Uart.Printf("%u, min_speed = %X\r", id, Prm.min_speed);
    Uart.Printf("%u, max_speed = %X\r", id, Prm.max_speed);
//    Uart.Printf("%u, adc = %X\r", id, Prm.adc);
#endif
}
#endif


#if 1 // ==== CMDs ====
uint8_t Motor_t::NOP() {
    return Spi.DaisyTxRxByte(id, 0x00);
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
    TxBuf[0] = GET_PARAM | (0x1F & Addr);
    TxBuf[1] = TxBuf[2] = TxBuf[3] = 0;
    Spi.DaisyTxRxData(id, PTxBuf, 4, PRxBuf);
    *PValue = RxBuf[0];
    if(RxBuf[1] != 0) { *PValue <<= 8; *PValue |= RxBuf[1]; }
    if(RxBuf[2] != 0) { *PValue <<= 8; *PValue |= RxBuf[2]; }
    if(RxBuf[3] != 0) { *PValue <<= 8; *PValue |= RxBuf[3]; }
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
    Uart.Printf("status: %A\r", PRxBuf, 4);
}
#endif

