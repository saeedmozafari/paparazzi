/*
 * Copyright (C) 2010  Gautier Hattenberger
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
 * along with paparazzi; see the file COPYING.  If not, write to
 * the Free Software Foundation, 59 Temple Place - Suite 330,
 * Boston, MA 02111-1307, USA.
 *
 */

/** @file sonar_adc.h
 *  @brief simple driver to deal with one sonar sensor on ADC
 */

#ifndef SONAR_I2C_H
#define SONAR_I2C_H

#include "std.h"
#include "mcu_periph/i2c.h"

struct SonarI2c {
    struct i2c_periph *i2c_p;
    struct i2c_transaction i2c_trans;
    uint16_t meas;			///< Raw value form module
    float distance;                     ///< Distance measured in meters
    //volatile bool_t data_available;     ///< data ready flag
    int32_t prom_cnt;                   ///< number of bytes read from PROM
};

extern struct SonarI2c sonar_i2c;

extern void sonar_i2c_init(void);
extern void sonar_i2c_read(void);

#endif