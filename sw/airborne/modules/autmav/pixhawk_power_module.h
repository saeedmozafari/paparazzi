/*
 * Copyright (C) 2010 Martin Muller
 * Copyright (C) 2016 Gautier Hattenberger
 *
 * This file is part of paparazzi.
 *
 * paparazzi is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * paparazzi is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with paparazzi; see the file COPYING.  If not, see
 * <http://www.gnu.org/licenses/>.
 */

/**
 * @file modules/autmav/pixhawk_power_module.h
 *
 * This module reads voltage and current from ADC 2 and 3 in AP process 
 * and applies them to system variables for monitoring
 *
 */

#ifndef PIXHAWK_POWER_MODULE_H
#define PIXHAWK_POWER_MODULE_H

#include "std.h"

extern uint16_t adc_voltage_val;
extern uint16_t adc_current_val;
void power_module_init(void);
void power_module_periodic(void);

#endif /* PIXHAWK_POWER_MODULE_H */
