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

#include "application.h"
#include "wifi_driver.h"
#include "http_server.h"

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

    while(TRUE) {
        chThdSleepMilliseconds(999);
    }
}

void Init() {
    JtagDisable();
    Uart.Init(115200);
    Led.Init();
    Uart.Printf("\rWiFi Glidertrack AHB=%u MHz\r", Clk.AHBFreqHz/1000000);
//    Usb.Init();
//    Vcp.Init();
//    Usb.Connect();
    App.Init();
    HttpServer.Init();
    WiFi.Init();

//    chThdSleepMilliseconds(3999);
//    Uart.Printf("Lengh: %u\r", WiFi.RplBuf.GetFilledCount());
}
