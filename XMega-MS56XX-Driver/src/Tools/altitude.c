/*
 * altitude.c
 *
 * Created: 10/4/2018 8:43:34 PM
 *  Author: Daniel
 */ 

#include "altitude.h"

int32_t calc_altitude(int32_t press)
//Returns altitude (cm) from pressure (Pa)
{
	return (int32_t)(100 * (SEA_LEVEL_PRESS - press) / (G_CONST * RHO_AIR));
}