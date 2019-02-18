#ifndef SONAR_MAXBOTIX_UART_H
#define SONAR_MAXBOTIX_UART_H

#include "std.h"
#include "paparazzi.h"
#include <stdio.h>
#include "std.h"
#include "subsystems/datalink/datalink.h"


extern float range;
//extern char sonbuf2[10];
//extern uint8_t sonbuf2_len=0;
extern void sonar_init(void);
extern void sonar_get_periodic(void);

#endif