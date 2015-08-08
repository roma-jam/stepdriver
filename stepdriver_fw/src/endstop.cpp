/*
 * endstop.cpp
 *
 *  Created on: 08 рту. 2015 у.
 *      Author: RomaJam
 */


#include "endstop.h"

endstop_t EndStops;

void endstop_t::Init() {
    PinSetupIn(ENDSTOPS_GPIO, ENDSTOP1, pudNone);
    PinSetupIn(ENDSTOPS_GPIO, ENDSTOP2, pudNone);

    EXTI->IMR = EXTI_IMR_MR0 | EXTI_IMR_MR1; // channel 0 and 1 init
    EXTI->RTSR = EXTI_RTSR_TR0 | EXTI_RTSR_TR1; // rising 0 and 1 init
    EXTI->FTSR = EXTI_FTSR_TR0 | EXTI_FTSR_TR1; // falling 0 and 1 init


    NVIC_EnableIRQ(EXTI0_IRQn);
    NVIC_EnableIRQ(EXTI1_IRQn);
}

void endstop_t::IrqHandler(endstop_ch Ch) {
    EXTI->PR |= Ch; // clear pending bit
    Uart.Printf("EndStopHit ch=%u\r", Ch);

}

extern "C" {
void EXTI0_IRQHandler() {
    EndStops.IrqHandler(endstop_ch_1);
}
void EXTI1_IRQHandler() {
    EndStops.IrqHandler(endstop_ch_2);
}
}
