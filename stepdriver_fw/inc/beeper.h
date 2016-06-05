/*
 * beeper.h
 *
 *  Created on: 08 рту. 2015 у.
 *      Author: RomaJam
 */

#ifndef INC_BEEPER_H_
#define INC_BEEPER_H_

#include "kl_lib_f100.h"

#define BEEPER_DURATION_MS  51
#define BEEPER_DELAY_MS     99

#define BEEPER_GPIO         GPIOB
#define BEEPER_PIN          7

#define BEEPER_TIM          TIM4
#define BEEPER_CH           CCR2

#define BEEPER_ENDPOINT_SEQ     1
#define BEEPER_WIFI_RDY_SEQ     2
#define BEEPER_START_SEQ        3

struct BeeperSequence_t {
//    uint8_t Delay;
    uint8_t Count; // count to beep
};

class beeper_t {
private:
    VirtualTimer Timer;
    bool isOn;
    BeeperSequence_t Seq;
    void BeeperOn()         { PinSet(BEEPER_GPIO, BEEPER_PIN); isOn = true;     }
    void BeeperOff()        { PinClear(BEEPER_GPIO, BEEPER_PIN); isOn = false;  }
public:
    void Init();
    void Beep();
    void Sequence(uint8_t BeepCnt);
    void Timeout();
    bool isBeep() { return isOn; }
};

extern beeper_t Beeper;


#endif /* INC_BEEPER_H_ */
