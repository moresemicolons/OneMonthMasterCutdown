/*
 * timercounter.h
 *
 * Created: 10/4/2018 7:01:22 PM
 *  Author: Daniel
 */ 


#ifndef TIMERCOUNTER_H_
#define 

#include <asf.h>

void TC0_setup(TC0_t* TC, PORT_t* port, uint8_t pins_to_ctrl);
void TC_config(TC0_t* TC, float freq, uint8_t duty0, uint8_t duty1, uint8_t duty2, uint8_t duty3);

#endif /* TIMERCOUNTER_H_ */