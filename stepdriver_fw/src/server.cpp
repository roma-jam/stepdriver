/*
 * server.cpp
 *
 *  Created on: 27 но€б. 2014 г.
 *      Author: Roma Jam
 */


#include "server.h"
#include "wifi_driver.h"

server_t HttpServer;

// ==== LED Thread ====
static WORKING_AREA(waHttpServer, SERVER_THD_SZ);
__attribute__ ((__noreturn__))
static void HttpServerThread(void *arg) {
    chRegSetThreadName("HttpServer");
    while(1) {
    	HttpServer.Sleep();
    	HttpServer.Task();
    }
}

void server_t::Task() {
//	Uart.Printf("Wup, %u\r", WiFi.RplBuf.GetFilledCount());
	char *S = WiFi.RplBuf.GetNextLine();
//	Uart.Printf("%s", S);
//	Uart.Printf("%c", *WiFi.RplBuf.pToRead++);
//	chThdSleepMilliseconds(51);
}



void server_t::Init() {
	PThd = chThdCreateStatic(waHttpServer, sizeof(waHttpServer), NORMALPRIO, (tfunc_t)HttpServerThread, NULL);
}


