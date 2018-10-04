#include <asf.h>
#include <stdio.h>
#include "drivers/uart_tools.h"
#include "drivers/SPI.h"
#include "drivers/MS56XX.h"

#define COMMS_USART				USARTC0
#define USART_TX_PIN			IOPORT_CREATE_PIN(PORTC, 3)
#define USART_RX_PIN			IOPORT_CREATE_PIN(PORTC, 2)
#define PRESSURE_SELECT_PIN		IOPORT_CREATE_PIN(PORTC, 4)
#define NUM_ALTITUDES_TRACKED	10

#define STATE_PRELAUNCH			1
#define STATE_ASCENT			2
#define STATE_DESCENT			3
#define STATE_LANDED			4

//Track last 10 altitudes in a circular buffer
// Altitudes (cm)
int32_t[NUM_ALTITUDES_TRACKED] altitude_backing_array;
RingBuffer32 recentalts;

MS56XX_Data_t filtered_5_pressures(MS56XX_t* sensor);

//Example usage of MS5611/07 driver for One Monthers
int main (void)
{
	uint8_t FLIGHT_STATE
	board_init();
	sysclk_init();

	UART_computer_init(&COMMS_USART, &PORTC, USART_TX_PIN, USART_RX_PIN);
	
	MS56XX_t pressure_sensor = define_new_MS56XX_default_OSR(MS5607, &SPIC, PRESSURE_SELECT_PIN);
	initializespi(&SPIC, &PORTC);
	enable_select_pin(pressure_sensor.select_pin);
	//Pressure sensor initialization routine, also reads calibration data from sensor
	calibratePressureSensor(&pressure_sensor);
	
	//Initialize altitude buffer and fill it with pressure measurements
	void rb32_init(&recentalts, altitude_backing_array, NUM_ALTITUDES_TRACKED);
	for (uint8_t i = 0; i < rb32_length(&recentalts); i++)
	{
		rb32_write(&recentalts, &filtered_5_pressures(&pressure_sensor), 1);
	}
	
	while (1)
	{
		readMS56XX(&pressure_sensor);
		printf("Pressure is %" PRIi32 ", temperature is %" PRIi32 ", %s\n", pressure_sensor.data.pressure, pressure_sensor.data.temperature, pressure_sensor.data.valid ? "Valid" : "Not valid");
		delay_ms(1000);
	}
	while (1)
	{
		//New pressure sample
		MS56XX_Data_t data = filtered_5_pressures(&pressure_sensor);
		if (!data.valid)
		{
			//Handle the "pressure sensor is broken" case
		}
		rb32_write(&recentalts, &(data.pressure), 1);
	}
}


MS56XX_Data_t filtered_5_pressures(MS56XX_t* sensor)
{
	int32_t pressures[5];
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
	//Pressure and temperature are an average of the results without the top and bottom values
	int32_t p, T;
	for (uint8_t i = 0; i < 5; i++)
	{
		p += pressures[i].pressure;
		T += pressures[i].temperature;
	}
	p -= (high.pressure + low.pressure);
	T -= (high.temperature + low.temperature);
	MS56XX_Data_t result;
	result.pressure = p;
	result.temperature = T;
	//TODO: check validity
	result.valid = 1;
	return result;
}