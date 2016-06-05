/*
 * events.h
 *
 *  Created on: 27 но€б. 2014 г.
 *      Author: r.leonov
 */

#ifndef EVENTS_H_
#define EVENTS_H_


#define EVTMSK_WIFI_READY                       EVENT_MASK(0)
#define EVTMSK_WIFI_HTTP_GET_REQUEST            EVENT_MASK(1)
#define EVTMSK_WIFI_HTTP_ACTION                 EVENT_MASK(2)
#define EVTMSK_WIFI_STARTED                     EVENT_MASK(3)
#define EVTMSK_IR_RX                            EVENT_MASK(4)
#define EVTMSK_TIMER                            EVENT_MASK(5)
#define EVTMSK_NEWSECOND                        EVENT_MASK(6)
#define EVTMSK_PILL_CHECK                       EVENT_MASK(7)
#define EVTMSK_UART_RX_POLL                     EVENT_MASK(8)
#define EVTMSK_DFU_REQUEST                      EVENT_MASK(9)


#endif /* EVENTS_H_ */
