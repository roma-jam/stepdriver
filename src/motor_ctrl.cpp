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
            Uart.Printf("Cmd %A\r", P, Len, ' ');
            Rslt = ICmdExecute(P, Len);
            break;
        case '$':
            NEXT_BYTE(P,Len)
            Uart.Printf("Srv %A\r", P, Len, ' ');
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
    if(ALen > 5) return LENGTH_ERROR;
    uint8_t *P = (uint8_t*)&CmdValues;
    for(uint8_t i=0; i<ALen; i++, Ptr++) {
        if(IsDelimeter(*Ptr)) P++;
        else {
            if(TryConvertToDigit(*Ptr, Ptr)) {
                (uint8_t&)*P <<= 4;
                (uint8_t&)*P = *Ptr;
            }
            else return CMD_ERROR;
        }
    }
    Uart.Printf("SrvID=%X, Value=%u\r", CmdValues.SrvID, CmdValues.SrvValue);
    Ack.MtrID = 0x0F;
    Ack.CmdID = CmdValues.SrvID+1;
    Ack.Err = OK;
    return OK;
}

uint8_t Driver_t::ICmdExecute(uint8_t *Ptr, uint8_t ALen) {
    return OK;

}

/*
void Driver_t::CmdHandle() {
    uint8_t *Ptr = Cmd;
    bool CmdExecute = false;
    uint32_t ClearLen = CmdLength;
//    Uart.Printf("%A\r", Ptr, CmdLength);

#if 1 // ==== UniCode ====
    if(CmdLength == 0) Uart.Printf("Wromg Cmd\r");
    else {
        if(*Ptr != '#') Uart.Printf("Error # symbol\r");
        else {
            Ptr++;
            CmdLength--;
            if(!TryConvertToDigit(*Ptr++, &Ack.MtrID)) Uart.Printf("MotorID Error\r");
            else {
                if(Ack.MtrID >= NumberOfMotors) Uart.Printf("MotorID=%u doesn't exist\r", Ack.MtrID);
                else {
                    CmdLength--;
                    if(*Ptr++ != ',') Uart.Printf("Wrong delimeter\r");
                    else {
                        CmdLength--;
                        TryConvertToDigit(*Ptr, Ptr);
                        Ack.CmdID = *Ptr++;
                        CmdLength--;
                        if(*Ptr != ',') {
                            Ack.CmdID <<= 4;
                            TryConvertToDigit(*Ptr, Ptr);
                            Ack.CmdID |= *Ptr++;
                            CmdLength--;
                        }
                        if(CmdLength == 0x00) {
                            CmdExecute = true;
                        }
                        else { // Next Symbols are present
                            if(*Ptr++ != ',') Uart.Printf("Wrong CmdID or delimeter\r");
                            else {
                                CmdLength--;
                                CmdExecute = true;
                            }  // long cmd
                        } // short cmd
                    } // correct 2delimetr
                } // correct number of motors
            } // correct motorID
        } // correct start symbol
        if(*Ptr != '$') Uart.Printf("Error $ symbol\r");
        else {
            uint8_t Value;
            Uart.Printf("ServiceMsg\r");
            Ptr++;
            CmdLength--;
            TryConvertToDigit(*Ptr, Ptr);
            Ack.CmdID = *Ptr++;
            CmdLength--;
            if(CmdLength != 0) {
                if(*Ptr != ',') {
                    Ack.CmdID <<= 4;
                    TryConvertToDigit(*Ptr, Ptr);
                    Ack.CmdID |= *Ptr;
                }
                else {
                    Ptr++;
                    CmdLength--;
                    TryConvertToDigit(*Ptr, &Value);
                }
            }
            if(CmdLength != 0) {
                if(*Ptr++ != ',') Uart.Printf("Wrong CmdID or delimeter\r");
                CmdLength--;
            }
            switch (Ack.CmdID) {
                case NEW_NUMBER:
                    Uart.Printf("NewNumber=%u\r", Value);
                    break;
                default:
                    Uart.Printf("Error Service number ID\r");
                    break;
            }
        } // Service Msg
    } // cmd len != 0
#endif
    if(CmdExecute) {
#if 1 //==== Cmd Execute ====
        switch (Ack.CmdID) {

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
    } // CmdExecute
//    Uart.Printf("Cmd Handled\r");
    CmdLength = 0;
    PCmdBuf = Cmd;
    memset(PCmdBuf, 0, ClearLen);
}

*/
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

