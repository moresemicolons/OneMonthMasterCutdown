/*
 * timercounter.c
 *
 * Created: 10/4/2018 6:49:46 PM
 *  Author: Daniel
 */ 

#include "timercounter.h"

#define PRESCALER	1024
#define fclk		32000000.0f

void TC0_setup(TC0_t* TC, enum sysclk_port_id sysclk_port, uint8_t pins_to_ctrl)
/*	TC: pointer to timer counter (e.g. &TCE0)
	port: pointer to port (e.g. &PORTE)
	pins_to_ctrl: bitmask where the lower nibble specifies which pins on the port should be controlled by the TC (e.g. 0b1100 for pin 2 and 3)
*/
{
	sysclk_enable_peripheral_clock(TC);
	sysclk_enable_module(sysclk_port, SYSCLK_HIRES);
	
	TC->CTRLA = 0b00000111;
	TC->CTRLB = (pins_to_ctrl << 4) | 0b0011; //Control the user-specified pins, and set to single-slope PWM
}

void TC_config(TC0_t* TC, float freq, float duty)
/*	TC: pointer to timer counter (e.g. &TCE0)
	freq: frequency as a float (e.g. 0.75)
	duty: duty cycle for all controlled pins
*/
{
	TC->PER = (uint16_t)(fclk / (PRESCALER * freq) - 1.0f);
	TC->CCA = (uint16_t)(duty * (float)TC->PER);
	TC->CCB = TC->CCA;
	TC->CCC = TC->CCA;
	TC->CCD = TC->CCA;
}