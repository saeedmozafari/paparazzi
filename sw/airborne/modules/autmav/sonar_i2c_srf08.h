
#ifndef SONAR_I2C_SR08_H
#define SONAR_I2C_SR08_H

#include "std.h"
#include "mcu_periph/i2c.h"

#define SEND_COMMAND 0
#define SET_RANGE 1
#define REQ_RANGE 2
#define RANGE_READ_OK 3
#define REQ_LIGHT 4
#define READ_LIGHT_OK 5
#define SET_GAIN 6

extern uint8_t sonar_range;
extern uint8_t analog_gain;
extern bool use_rms;

extern float buf0;

extern void sonar_i2c_init(void);
extern void sonar_i2c_read(void);
extern void sonar_srf08_event(void);
extern float GetSonarAlt(void);

#endif