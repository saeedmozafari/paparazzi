#ifndef SONAR_UART_H
#define SONAR_UART_H

#include "std.h"
#include "paparazzi.h"
#include <stdio.h>
#include "std.h"
#include "subsystems/datalink/datalink.h"

union uint162bytes {
  uint8_t bytes[3];
  uint16_t value;
};

extern union uint162bytes u2b;

extern char buffer[3];
extern uint8_t write_idx;
extern float range;

extern void sonar_init(void);
extern void sonar_get_periodic(void);

#endif