/*
 * Copyright (C) 2017 Michal Podhradsky <michal.podhradsky@aggiemail.usu.edu>
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

/** @file modules/lidar/lidar_sf11.h
 *  @brief driver for the Parallax SF11-A/B/C Laser Rangefinder connected over i2c bus.
 */
#ifndef TOSHIBA_LED_DRIVER_H
#define TOSHIBA_LED_DRIVER_H

#include "std.h"
#include "math/pprz_algebra_int.h"

#include "mcu_periph/i2c.h"

struct led {
  struct i2c_transaction trans;
  uint8_t addr;
};

extern void toshiba_led_init(void);
extern void toshiba_led_periodic(void);

#endif /* LIDAR_SF11_I2C_H */
