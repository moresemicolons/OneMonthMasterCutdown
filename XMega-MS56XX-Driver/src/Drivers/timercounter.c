/*
 * timercounter.c
 *
 * Created: 10/4/2018 6:49:46 PM
 *  Author: Daniel
 */ 

#include "timercounter.h"

#define PRESCALER	1024
#define fclk		32000000.0f

void TC0_setup(TC0_t* TC, sysclk_port_id sysclk_port, uint8_t pins_to_ctrl)
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

void TC_config(TC0_t* TC, float freq, uint8_t duty0, uint8_t duty1, uint8_t duty2, uint8_t duty3)
/*	TC: pointer to timer counter (e.g. &TCE0)
	freq: frequency as a float (e.g. 0.75)
	dutyx: duty cycle for that pin. Should be set to 0 for uncontrolled pins
*/
{
	TC->PER = (uint16_t)(fclk / (PRESCALER * freq) - 1.0f);
	TC->CCA = duty0 * TC->PER / 100;
	TC->CCB = duty1 * TC->PER / 100;
	TC->CCC = duty2 * TC->PER / 100;
	TC->CCD = duty3 * TC->PER / 100;
}