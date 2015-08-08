/*
 * beeper.cpp
 *
 *  Created on: 08 рту. 2015 у.
 *      Author: RomaJam
 */



#include "beeper.h"

beeper_t Beeper;

void TmrBeeperCallback(void *p) {
    chSysLockFromIsr();
    Beeper.Timeout();
    chSysUnlockFromIsr();
}

void beeper_t::Timeout() {
    if(isBeep()) { // start timer to delay and decrese the count
        BeeperOff();
        Seq.Count--;
        if(Seq.Count != 0)
            chVTSetI(&Timer, MS2ST(BEEPER_DELAY_MS), TmrBeeperCallback, nullptr);
    }
    else { // beep and start timer
        BeeperOn();
        chVTSetI(&Timer, MS2ST(BEEPER_DURATION_MS), TmrBeeperCallback, nullptr);
    }
}

void beeper_t::Init() {
    PinSetupOut(BEEPER_GPIO, BEEPER_PIN, omPushPull, ps50MHz);
    BeeperOff();
//    chVTSetI(&Beeper.Timer, MS2ST(UART_RX_POLLING_MS), TmrUartRxCallback, nullptr);
}

void beeper_t::Sequence(uint8_t BeepCnt) {
    if(isBeep()) { // turn off timer and clean beeper
        chVTResetI(&Timer);
        BeeperOff();
    }
    Seq.Count = BeepCnt;
    BeeperOn();
    chVTSet(&Timer, MS2ST(BEEPER_DURATION_MS), TmrBeeperCallback, nullptr);
}
