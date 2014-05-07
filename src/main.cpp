/*
 * File:   main.cpp
 * Author: Kreyl
 * Project: Armlet2South
 *
 * Created on Feb 05, 2013, 20:27
 */

#include "led.h"
#include "ch.h"
#include "hal.h"
#include "usb_f103.h"
#include "vcp.h"
#include "motor_ctrl.h"

#include "application.h"

Timer_t StepClk;

static inline void Init();

int main(void) {
    // ==== Init clock system ====
    uint8_t ClkEnable = 1;
    ClkEnable = Clk.ClockConfigure(ciHSE, pllMul6, ahbDiv1, apbDiv1, apbDiv1, psPLLOn);
    Clk.UpdateFreqValues();
    // ==== Init OS ====
    halInit();
    chSysInit();
    // ==== Init Hard & Soft ====
    Init();
    if(!ClkEnable) Uart.Printf("CF=%u\r", ClkEnable);

    StepClk.Init(TIM2);
    StepClk.Enable();
    StepClk.InitPwm(GPIOA, 1, 2, invNotInverted);
    StepClk.SetPrescaler(100);
    StepClk.SetTopValue(100);
    StepClk.SetPwm(50);

//    PinSetupOut(GPIOA, 1, omPushPull, ps50MHz);


    while(TRUE) {
//        PinToggle(GPIOA, 1);
        chThdSleepMilliseconds(249);
    }
}

void Init() {
    JtagDisable();
    Uart.Init(256000);
    Led.Init();
    Uart.Printf("\rSincahonda  AHB=%u; APB1=%u; APB2=%u\r", Clk.AHBFreqHz, Clk.APB1FreqHz, Clk.APB2FreqHz);
    Driver.Init();
    Usb.Init();
    Vcp.Init();

    Usb.Connect();

    App.Init();
}
