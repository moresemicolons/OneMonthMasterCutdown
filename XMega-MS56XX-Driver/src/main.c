#include <asf.h>
#include <stdio.h>
#include "drivers/uart_tools.h"
#include "drivers/SPI.h"
#include "drivers/MS56XX.h"
#include "Drivers/timercounter.h"
#include "Tools/altitude.h"
#include "tools/RingBuffer.h"
#include "Drivers/TC_clock.h"

#define COMMS_USART				USARTE0
#define USART_TX_PIN			IOPORT_CREATE_PIN(PORTE, 3)
#define USART_RX_PIN			IOPORT_CREATE_PIN(PORTE, 2)
#define PRESSURE_SELECT_PIN		IOPORT_CREATE_PIN(PORTC, 4)
#define NUM_ALTITUDES_TRACKED 	20

#define STATE_PRELAUNCH			1
#define STATE_ASCENT			2
#define STATE_DESCENT			3
#define STATE_LANDED			4

#define LED_PORT				PORTE
#define LED_PIN_NUM				0
#define LED_TC0					TCE0
#define LED_SYSCLK_PORT			SYSCLK_PORT_E

#define blinkrate1				1.0f
#define blinkrate2				5.0f
#define blinkrate3				2.5f
#define blinkrate4				1.0f

#define CUTDOWN_ALT				30000 //(cm)

#define HOTWIRE_PIN				IOPORT_CREATE_PIN(PORTA, 0)

//Track last 10 altitudes in a circular buffer
// Altitudes (cm)
int32_t altitude_backing_array[NUM_ALTITUDES_TRACKED];
RingBuffer32_t recentalts;

MS56XX_Data_t filtered_5_pressures(MS56XX_t* sensor);

int main (void)
{
	uint8_t flightstate = 0;
	board_init();
	sysclk_init();
	
	time = 0;
	
	TC0_setup(&LED_TC0, LED_SYSCLK_PORT, 0b0001);
	
	PORTE.DIRSET= 0b00000001;
	
	TC_config(&LED_TC0, 10, 20);

	UART_computer_init(&COMMS_USART, &PORTE, USART_TX_PIN, USART_RX_PIN);
	
	printf("Initializing...\n");
	
	MS56XX_t pressure_sensor = define_new_MS56XX_default_OSR(MS5607, &SPIC, PRESSURE_SELECT_PIN);
	initializespi(&SPIC, &PORTC);
	enable_select_pin(pressure_sensor.select_pin);
	//Pressure sensor initialization routine, also reads calibration data from sensor
	calibratePressureSensor(&pressure_sensor);
	
	//Initialize altitude buffer and fill it with pressure measurements
	int32_t alt, alt_initial;
	rb32_init(&recentalts, altitude_backing_array, NUM_ALTITUDES_TRACKED);
	alt_initial = calc_altitude(filtered_5_pressures(&pressure_sensor).pressure);
	printf("Init alt: %li\n",alt_initial);
	for (uint8_t i = 0; i < rb32_length(&recentalts); i++)
	{
		alt = calc_altitude(filtered_5_pressures(&pressure_sensor).pressure) - alt_initial;
		rb32_write(&recentalts, &alt, 1);
	}
	
	printf("Initialized!\n\n");
	
	flightstate = STATE_PRELAUNCH;
	
	TC_config(&LED_TC0, 0.5, 20);
	
	uint32_t loop_counter = 0;
	while (1)
	{
		//New pressure sample
		MS56XX_Data_t data = filtered_5_pressures(&pressure_sensor);
		alt = calc_altitude(data.pressure) - alt_initial;
		if (!data.valid)
		{
			//Handle the "pressure sensor is broken" case
			TC_config(&LED_TC0, 10, 20);
			printf("MS5607 Data Invalid!\n");
		}
		rb32_write(&recentalts, &(alt), 1);
		printf("alt: %li, ",alt);
		printf("pres: %li, ",data.pressure);
		if (flightstate == STATE_PRELAUNCH)
		{	
			int32_t oldest_alt = rb32_get_nth(&recentalts, rb32_length(&recentalts) - 1);
			printf("oldest_alt: %li, ", oldest_alt);
			printf("alt - oldest_alt: %li, ",alt - oldest_alt);
			printf("state: PRELAUNCH\n");
			
			//Lifted off if more than 2 m/s OR at least 10 m up and some upwards movement over the past second
			if (alt - oldest_alt > 1000 || (alt > 6000 && alt - oldest_alt > 0))
			{
				flightstate = STATE_ASCENT;
				TC_config(&LED_TC0, blinkrate2, 20);
				printf("\n\nAscent!\n\n");
			}
		}
		else if (flightstate == STATE_ASCENT)
		{
			printf("state: ASCENT\n");
			if (alt > CUTDOWN_ALT)
			{
				printf("\n\nCutdown!\n\n");
				TC_config(&LED_TC0, 1.0f, 0);
				gpio_set_pin_high(HOTWIRE_PIN);
				delay_s(8); //TODO: make sure this is long enough
				gpio_set_pin_low(HOTWIRE_PIN);
				
				flightstate = STATE_DESCENT;
				TC_config(&LED_TC0, blinkrate3, 20);
				printf("\n\nDescent!\n\n");
			}
		}
		else if (flightstate == STATE_DESCENT)
		{
			printf("state: DESCENT\n");
			int32_t oldest_alt = rb32_get_nth(&recentalts, rb32_length(&recentalts) - 1);
			if (rb32_get_nth(&recentalts, 0) - oldest_alt < 100)
			{
				flightstate = STATE_LANDED;
				TC_config(&LED_TC0, blinkrate4, 20);
				printf("\n\nLanded!\n\n");
			}
		}
		else if (flightstate == STATE_LANDED)
		{
			printf("state: LANDED\n");
			//Literally nothing to do. Contemplate the meaning of electrical impulses? 
		}
		else
		{
			//Should never be here, indicate error somehow
		}
		loop_counter++;
		//while (time < loop_counter * 100); //Keep to 10 Hz sample rate
	}
}

MS56XX_Data_t filtered_5_pressures(MS56XX_t* sensor)
{
	MS56XX_Data_t pressures[5];
	for (uint8_t i = 0; i < 5; i++)
	{
		readMS56XX(sensor);
		pressures[i] = sensor->data;
	}
	MS56XX_Data_t low = pressures[0], high = pressures[0];
	//First, find lowest and highest
	for (uint8_t i = 0; i < 5; i++)
	{
		if (pressures[i].pressure < low.pressure)
		{
			low = pressures[i];
		}
		if (pressures[i].pressure > high.pressure)
		{
			high = pressures[i];
		}
	}
	uint8_t valid = 1;
	//Iterate through and check that none of the included measurements are flagged as invalid. If they are, subtract them
	for (uint8_t i = 0; i < 5; i++)
	{
		if (!pressures[i].valid && pressures[i].pressure != high.pressure && pressures[i].pressure != low.pressure)
		{
			valid = 0;
		}
	}
	//Pressure and temperature are an average of the results without the top and bottom values
	int32_t p = 0, T = 0;
	for (uint8_t i = 0; i < 5; i++)
	{
		p += pressures[i].pressure;
		T += pressures[i].temperature;
	}
	p -= (high.pressure + low.pressure);
	T -= (high.temperature + low.temperature);
	MS56XX_Data_t result;
	result.pressure = p / 3;
	result.temperature = T / 3;
	//TODO: check validity
	result.valid = valid;
	return result;
}