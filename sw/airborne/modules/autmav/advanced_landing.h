#ifndef NAV_ADVANCED_LANDING_H
#define NAV_ADVANCED_LANDING_H

#include "std.h"
#include "paparazzi.h"

extern float nav_advanced_landing_app_dist;
extern float nav_advanced_landing_direction;
extern float nav_advanced_landing_flair_time_tresh;

extern float sf11_alt;

extern void advanced_landing_setup(void);

// Flightplan Code

extern void calc_turning_point(uint8_t, uint8_t, uint8_t, uint8_t);
extern void set_turning_direction(int16_t);
extern void set_sf11_agl_mode(bool);

extern void sf11_cb(uint8_t sender_id, float distance);

#endif