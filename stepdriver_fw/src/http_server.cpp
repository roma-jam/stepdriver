/*
 * http_server.cpp
 *
 *  Created on: 27 но€б. 2014 г.
 *      Author: r.leonov
 */

#include "http_server.h"
#include "wifi_driver.h"

server_t HttpServer;

static WORKING_AREA(waHttpThread, HTTP_SERVER_THD_SZ);
__attribute__ ((__noreturn__))
static void HttpThread(void *arg) {
    chRegSetThreadName("Http");
    while(1) {
        HttpServer.Sleep();
        HttpServer.Task();
    }
}

void server_t::Task() {
    if(WiFi.RplBuf.GetNextLine(Line, &LineLength) == OK) {
        LineHandle();
    }
    else {Uart.Printf("\rErr (HttpSerever.Task)\r"); }
}

void server_t::LineHandle() {
    Uart.Printf("%s", Line);
}

void server_t::Init() {
    PThd = chThdCreateStatic(waHttpThread, sizeof(waHttpThread), NORMALPRIO, (tfunc_t)HttpThread, NULL);
}
