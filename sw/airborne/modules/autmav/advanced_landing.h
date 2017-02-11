#ifndef NAV_CATAPULT_H
#define NAV_CATAPULT_H

#include "std.h"
#include "paparazzi.h"

extern float nav_advanced_landing_app_dist;
extern float nav_advanced_landing_direction;

extern bool advanced_landing_setup(void);

// Flightplan Code

extern bool calc_turning_point(uint8_t, uint8_t, uint8_t, uint8_t);

#endif
