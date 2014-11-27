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
}

void server_t::LineHandle() {
//    Uart.Printf("(%u)%s", LineLength, Line);
    pInnerLine = Line;
    char *S = strtok(pInnerLine, AT_COMMAND_DELIMETR);
    if(S != nullptr) {
        if(strcasecmp(S, AT_WIND_CMD) == 0) WindCommand();
        else if(strcasecmp(S, AT_OK) == 0) CommandSuccess();
    }
}

void server_t::WindCommand() {
    char *S = strtok(NULL, AT_COMMAND_DELIMETR);
    Uart.Printf("Wind ID: %s ", S);
    S = strtok(NULL, AT_COMMAND_DELIMETR);
    Uart.Printf("{ %s }\r", S);
}

void server_t::CommandSuccess() {
    Uart.Printf("Command Ok\r");
}

void server_t::Init() {
    PThd = chThdCreateStatic(waHttpThread, sizeof(waHttpThread), NORMALPRIO, (tfunc_t)HttpThread, NULL);
}
