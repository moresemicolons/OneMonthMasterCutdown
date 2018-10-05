/*
 * timercounter.h
 *
 * Created: 10/4/2018 7:01:22 PM
 *  Author: Daniel
 */ 


#ifndef TIMERCOUNTER_H_
#define TIMERCOUNTER_H_

#include <asf.h>

void TC0_setup(TC0_t* TC, PORT_t* port, uint8_t pins_to_ctrl);
void TC_config(TC0_t* TC, float freq, uint8_t duty);

#endif /* TIMERCOUNTER_H_ */