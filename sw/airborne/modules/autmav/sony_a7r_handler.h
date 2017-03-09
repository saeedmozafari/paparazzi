#ifndef SONY_A7R_HANDLER_H
#define SONY_A7R_HANDLER_H

#include "std.h"
#include "paparazzi.h"

extern void sony_a7r_handler_setup(void);

extern void esp_01_jap(void);
extern void esp_01_cipmux(void);
extern void esp_01_cipstart(void);
extern void esp_01_cipsend(void);
extern void esp_01_msearch(void);
extern void esp_01_cipclose(void);
extern void esp_01_cipstart_tcp(void);
extern void esp_01_cipsend_tcp(void);
extern void esp_01_setmode(void);
extern void esp_01_cipsend_shoot(void);
extern void sony_a7r_shoot(void);
extern void endline(void);

#endif