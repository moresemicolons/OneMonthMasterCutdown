/*
 * altitude.h
 *
 * Created: 10/4/2018 7:14:06 PM
 *  Author: Daniel
 */ 


#ifndef ALTITUDE_H_
#define ALTITUDE_H_

#include <asf.h>

#define SEA_LEVEL_PRESS			101325 //(Pa)
#define RHO_AIR					1.225f //(kg/m^3)
#define G_CONST					9.81f //(m/s^2)

int32_t calc_altitude(int32_t press); //Returns altitude (cm) from pressure (Pa)

#endif /* ALTITUDE_H_ */