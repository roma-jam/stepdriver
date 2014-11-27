/*
 * http_server.cpp
 *
 *  Created on: 27 но€б. 2014 г.
 *      Author: r.leonov
 */

#include "http_server.h"
#include "wifi_driver.h"
#include "application.h"

server_t HttpServer;

char OpenSocketCmd[] = "at+s.sockd=80,t\n\r";
char CloseSocketCmd[] = "at+s.sockd=0\n\r";

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
    CurrData = strtok(pInnerLine, AT_COMMAND_DELIMETR);
    if(CurrData != nullptr) {
        if(strcasecmp(CurrData, AT_WIND_CMD) == 0) WindCommand();
        else if(strcasecmp(CurrData, AT_OK) == 0) CommandSuccess();
        else HostCommand();
    }
}

void server_t::WindCommand() {
    uint8_t ID=0;
    GetWindID(&ID);
//    Uart.Printf("%u ", ID);
//    char *S = strtok(NULL, AT_COMMAND_DELIMETR);
//    Uart.Printf("{%s}\r", S);
    switch (ID) {
        case WIFI_UP:
            App.SendEvent(EVTMSK_WIFI_READY);
            break;
        case WIFI_HTTP_GET:
            break;
        default:
            break;
    }
}

void server_t::HostCommand() {
//    Uart.Printf("(%u)%s\r", LineLength, Line);
    Uart.Printf("Request:%s\r", CurrData);
    char *S = strtok(CurrData, " ");
    if(strcasecmp(S, AT_GET) == 0) App.SendEvent(EVTMSK_WIFI_HTTP_GET_REQUEST);
}

void server_t::CommandSuccess() {
    Uart.Printf("Command Ok\r");
}

void server_t::Init() {
    PThd = chThdCreateStatic(waHttpThread, sizeof(waHttpThread), NORMALPRIO, (tfunc_t)HttpThread, NULL);
}

void server_t::OpenSocket() {
    WiFi.CmdSend((uint8_t *)OpenSocketCmd, sizeof(OpenSocketCmd)-1);
}

void server_t::CloseSocket() {
    WiFi.CmdSend((uint8_t*)CloseSocketCmd, sizeof(CloseSocketCmd)-1);
}
