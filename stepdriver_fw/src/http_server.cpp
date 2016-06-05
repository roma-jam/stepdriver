/*
 * http_server.cpp
 *
 *  Created on: 27 но€б. 2014 г.
 *      Author: r.leonov
 */

#include "http_server.h"
#include "wifi_driver.h"
#include "application.h"
#include "html_page.h"

server_t HttpServer;

char OpenSocketCmd[] = "at+s.sockd=80,t\n\r";
char CloseSocketCmd[] = "at+s.sockd=0\n\r";

static WORKING_AREA(waHttpThread, HTTP_SERVER_THD_SZ);
__attribute__ ((__noreturn__))
static void HttpThread(void *arg)
{
    chRegSetThreadName("Http");
    while(1)
    {
        HttpServer.Sleep();
        HttpServer.Task();
    }
}

void server_t::Task()
{
    if(WiFi.RplBuf.GetNextLine(Line, &LineLength) == OK)
        LineHandle();
}

void server_t::LineHandle()
{
#if (APP_WIFI_LINE_DEBUG)
//    Uart.Printf("[%u] ", LineLength);
    Uart.Printf("%s", Line);
#endif
    pInnerLine = Line;
    CurrData = strtok(pInnerLine, AT_COMMAND_DELIMETR);
    if(CurrData != nullptr)
    {
        if(strcasecmp(CurrData, AT_WIND_CMD) == 0) WindCommand();
        else if(strcasecmp(CurrData, AT_OK) == 0) CommandSuccess();
        else HostCommand();
    }
}

void server_t::WindCommand() {
    uint8_t ID=0;
    GetWindID(&ID);

#if(APP_WIFI_DEBUG)
    Uart.Printf("%u ", ID);
    char *S = strtok(NULL, AT_COMMAND_DELIMETR);
    Uart.Printf("{%s}\r", S);
#endif

    switch (ID)
    {
        case WIFI_UP:
            App.SendEvent(EVTMSK_WIFI_READY);
            break;
        case WIFI_HTTP_GET:
            break;
        default:
            break;
    }
}

void server_t::HostCommand()
{
#if (APP_WIFI_DEBUG)
//    Uart.Printf("[%u] ", LineLength);
    Uart.Printf("%s", Line);
    Uart.Printf("Request:%s\r", CurrData);
#endif

    CurrData = strtok(Line, " ");
    if(strcasecmp(CurrData, AT_GET) == 0) {
        CurrData = strtok(NULL, " ?");
        char *S = strtok(NULL, " ?");
        if(S != 0) {
            if( (strncmp (S,"button",6) == 0) || ((strncmp (S,"speed",5) == 0)) || ((strncmp (S,"time",4) == 0)) ) {
                CurrData = S;
                App.SendEvent(EVTMSK_WIFI_HTTP_ACTION);
            }
            else
                App.SendEvent(EVTMSK_WIFI_HTTP_GET_REQUEST);
        }
    }
}

void server_t::CommandSuccess()
{
    Started = true;
    Uart.Printf("Command Ok\r");
    App.SendEvent(EVTMSK_WIFI_STARTED);
}

void server_t::Init() {
    Started = false;
    PThd = chThdCreateStatic(waHttpThread, sizeof(waHttpThread), NORMALPRIO, (tfunc_t)HttpThread, NULL);
}

void server_t::GetRequest() {
#if (APP_HTTP_SERVER_DEBUG)
    Uart.Printf("Http request: %s\r", HttpServer.CurrData);
#endif

    index_html[936] = 'R';
    index_html[937] = 'D';
    index_html[938] = 'Y';
    index_html[939] = '0';
    SendHttpHeader(973);
    WiFi.CmdSend((uint8_t *)index_html, 973);
}

void server_t::ActionReply() {
    SendHttpHeader(973);
    WiFi.CmdSend((uint8_t *)index_html, 973);
}

void server_t::OpenSocket() {
#if (APP_HTTP_SERVER_DEBUG)
    Uart.Printf("Open socket\r");
#endif

    WiFi.CmdSend((uint8_t *)OpenSocketCmd, sizeof(OpenSocketCmd)-1);
}

void server_t::CloseSocket() {
#if (APP_HTTP_SERVER_DEBUG)
    Uart.Printf("Close socket\r");
#endif

    WiFi.CmdSend((uint8_t*)CloseSocketCmd, sizeof(CloseSocketCmd)-1);
}

void server_t::SendHttpHeader(uint32_t ContentLength) {
    char tmp[4], Output[8];
    char *P = Output;
    int32_t len = 0;
    // Place digits to buffer
    do {
        int32_t digit = ContentLength % 10;
        ContentLength /= 10;
        tmp[len++] = (digit < 10)? '0'+digit : 'A'+digit-10;
    } while(ContentLength > 0);
    while(len > 0) {
    	*P++ = tmp[--len];
    }
    *P++ = '\r'; *P++ = '\n'; // FIXME: need to end number by \r\n!
    *P++ = '\r'; *P++ = '\n'; // FIXME: need to end number by \r\n!
    char TmpBuf[sizeof(get_response_header) + sizeof(Output)];
    memcpy(TmpBuf, get_response_header, sizeof(get_response_header)-1);
    memcpy(&TmpBuf[sizeof(get_response_header)-1], Output, sizeof(Output));
    WiFi.CmdSend((uint8_t*)TmpBuf, sizeof(TmpBuf)-1);
}
