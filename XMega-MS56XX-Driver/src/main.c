#include <asf.h>
#include <stdio.h>
#include "drivers/uart_tools.h"
#include "drivers/SPI.h"
#include "drivers/MS56XX.h"
#include "Drivers/timercounter.h"
#include "Tools/altitude.h"
#include "tools/RingBuffer.h"

//#define COMPUTER_USART   	//Disable this for flight
#define SD_USART			//Enable this for flight

#ifdef SD_USART
#define COMMS_USART				USARTC0
#define USART_TX_PIN			IOPORT_CREATE_PIN(PORTC, 3)
#define USART_RX_PIN			IOPORT_CREATE_PIN(PORTC, 2)
#define USART_PORT				PORTC
#endif

#ifdef COMPUTER_USART
#define COMMS_USART				USARTE0
#define USART_TX_PIN			IOPORT_CREATE_PIN(PORTE, 3)
#define USART_RX_PIN			IOPORT_CREATE_PIN(PORTE, 2)
#define USART_PORT				PORTE
#endif


#define PRESSURE_SELECT_PIN		IOPORT_CREATE_PIN(PORTC, 4)
#define NUM_ALTITUDES_TRACKED 	20

#define STATE_PRELAUNCH			1
#define STATE_ASCENT			2
#define STATE_DESCENT			3
#define STATE_LANDED			4
#define STATE_CUTDOWN			99

#define LED_PORT				PORTE
#define LED_PIN_NUM				0
#define LED_TC0					TCE0
#define LED_SYSCLK_PORT			SYSCLK_PORT_E

#define blinkratePrelaunch				0.5f
#define blinkdutyPrelaunch				0.05f
#define blinkrateAscent					1.0f
#define blinkdutyAscent					0.1f
#define blinkrateCutdown				5.0f
#define blinkdutyCutdown				0.1f
#define blinkrateDescent				2.0f
#define blinkdutyDescent				0.25f
#define blinkrateLanded					1.0f
#define blinkdutyLanded					0.5f
#define blinkrateError					10.0f
#define blinkdutyError					0.9f

#define CUTDOWN_ALT				35000	//(cm)
#define CUTDOWN_TIME			8		//(s)
#define CUTDOWN_RESTART_DELAY	30		//(s)

#define HOTWIRE_PIN				IOPORT_CREATE_PIN(PORTA, 0)

//Track last 10 altitudes in a circular buffer
// Altitudes (cm)
int32_t altitude_backing_array[NUM_ALTITUDES_TRACKED];
RingBuffer32_t recentalts;

MS56XX_Data_t filtered_5_pressures(MS56XX_t* sensor);

bool hotwire_en = true;
uint32_t cutdown_start_time = 0;

int main (void)
{
	uint8_t flightstate = 0;
	board_init();
	sysclk_init();
	
	uint32_t time = 0;
	
	
	TC0_setup(&LED_TC0, LED_SYSCLK_PORT, 0b0001);
	
	PORTE.DIRSET= 0b00000001;
	PORTA.DIRSET= 0b00000001;
	
	TC_config(&LED_TC0, blinkrateError, blinkdutyError);

	UART_computer_init(&COMMS_USART, &USART_PORT, USART_TX_PIN, USART_RX_PIN);
	
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
	
	TC_config(&LED_TC0, blinkratePrelaunch, blinkdutyPrelaunch);
	
	uint32_t loop_counter = 0;
	while (1)
	{
		//Hacky timer, should be in seconds +/- 5%
		time = (loop_counter*90)/570;
		//New pressure sample
		MS56XX_Data_t data = filtered_5_pressures(&pressure_sensor);
		alt = calc_altitude(data.pressure) - alt_initial;
		if (!data.valid)
		{
			//Handle the "pressure sensor is broken" case
			TC_config(&LED_TC0, blinkrateError, blinkdutyError);
			printf("MS5607 Data Invalid!\n");
			hotwire_en = false;
		}
		else
		{
			hotwire_en = true;
		}
		
		rb32_write(&recentalts, &(alt), 1);
		printf("time: %li, ",time);
		printf("alt: %li, ",alt);
		printf("pres: %li, ",data.pressure);
		if (flightstate == STATE_PRELAUNCH)
		{	
			int32_t oldest_alt = rb32_get_nth(&recentalts, rb32_length(&recentalts) - 1);
			printf("state: PRELAUNCH\n");
			
			//Lifted off if more than 2 m/s OR at least 10 m up and some upwards movement over the past second
			if (alt - oldest_alt > 1000 || (alt > 6000 && alt - oldest_alt > 0))
			{
				flightstate = STATE_ASCENT;
				TC_config(&LED_TC0, blinkrateAscent, blinkdutyAscent);
				printf("\n\nAscent!\n\n");
			}
		}
		else if (flightstate == STATE_ASCENT)
		{
			printf("state: ASCENT\n");
			if (alt > CUTDOWN_ALT)
			{
				printf("\n\nCutdown!\n\n");
				flightstate = STATE_CUTDOWN;
				cutdown_start_time = time;
				TC_config(&LED_TC0, blinkrateCutdown, blinkdutyCutdown);
			}
		}
		else if (flightstate == STATE_CUTDOWN)
		{
			printf("state: CUTDOWN\n");
			gpio_set_pin_high(HOTWIRE_PIN);
			
			if(time > cutdown_start_time + CUTDOWN_TIME)			
			{	
				gpio_set_pin_low(HOTWIRE_PIN);
				
				flightstate = STATE_DESCENT;
				TC_config(&LED_TC0, blinkrateDescent, blinkdutyDescent);
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
				TC_config(&LED_TC0, blinkrateLanded, blinkdutyLanded);
				printf("\n\nLanded!\n\n");
			}
			if (alt > CUTDOWN_ALT+5000 && alt - oldest_alt > 0 && hotwire_en && time > cutdown_start_time + CUTDOWN_RESTART_DELAY)
			{
				printf("Still ascending! Restart cutdown!");
				flightstate = STATE_CUTDOWN;
				cutdown_start_time = time;
				TC_config(&LED_TC0, blinkrateCutdown, blinkdutyCutdown);
			}
		}
		else if (flightstate == STATE_LANDED)
		{
			printf("state: LANDED\n");
			
			int32_t oldest_alt = rb32_get_nth(&recentalts, rb32_length(&recentalts) - 1);
			if (alt > CUTDOWN_ALT+5000 && alt - oldest_alt > 0 && hotwire_en && time > cutdown_start_time + CUTDOWN_RESTART_DELAY)
			{
				printf("Still ascending! Restart cutdown!");
				flightstate = STATE_CUTDOWN;
				cutdown_start_time = time;
				TC_config(&LED_TC0, blinkrateCutdown, blinkdutyCutdown);
			}
			//Literally nothing to do. Contemplate the meaning of electrical impulses? 
		}
		else
		{
			//Should never be here, indicate error somehow
			TC_config(&LED_TC0, blinkrateError, blinkdutyError);
			printf("Invalid State! (how?!?!)\n");
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