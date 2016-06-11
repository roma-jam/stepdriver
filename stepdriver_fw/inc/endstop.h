/*
 * endstop.h
 *
 *  Created on: 08 рту. 2015 у.
 *      Author: RomaJam
 */

#ifndef INC_ENDSTOP_H_
#define INC_ENDSTOP_H_

#include "kl_lib_f100.h"
#include "config.h"

#define ENDSTOPS_GPIO   GPIOA
#define ENDSTOP1        APP_ENDSTOP_FORWARD
#define ENDSTOP2        APP_ENDSTOP_BACKWARD

enum endstop_ch {
    endstop_ch_1 = ENDSTOP1, // for channel 1 is 1
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
    void Hit(endstop_ch esChannel);
    void Release(endstop_ch esChannel);
};

extern endstop_t EndStops;

#endif /* INC_ENDSTOP_H_ */
