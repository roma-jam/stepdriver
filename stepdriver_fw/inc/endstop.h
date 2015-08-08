/*
 * endstop.h
 *
 *  Created on: 08 рту. 2015 у.
 *      Author: RomaJam
 */

#ifndef INC_ENDSTOP_H_
#define INC_ENDSTOP_H_

#include "kl_lib_f100.h"


#define ENDSTOPS_GPIO   GPIOA
#define ENDSTOP1        0
#define ENDSTOP2        1

enum endstop_ch {
    endstop_ch_1 = ENDSTOP1,
    endstop_ch_2,
    endstop_max
};

enum endstop_state_t {
    ess_Idle, ess_Hit
};

class endstop_t {
private:
    endstop_state_t State[endstop_max];
public:
    void Init();
    void IrqHandler(endstop_ch esChannel);
};

extern endstop_t EndStops;

#endif /* INC_ENDSTOP_H_ */
