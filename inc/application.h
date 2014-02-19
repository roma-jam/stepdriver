/*
 * application.h
 *
 *  Created on: 19 февр. 2014 г.
 *      Author: r.leonov
 */

#ifndef APPLICATION_H_
#define APPLICATION_H_


#include "kl_lib_f100.h"

class App_t {
private:

public:
    Thread *PThd;
    void Init();
};

extern App_t App;

#endif /* APPLICATION_H_ */
