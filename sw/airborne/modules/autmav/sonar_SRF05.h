/*
 * Copyright (C) 2010-2014 The Paparazzi Team
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
 */

/** @file modules/digital_cam/gpio_cam_ctrl.h
 *  @brief Digital Camera Control
 *
 * Provides the control of the shutter and the zoom of a digital camera
 * through standard binary IOs of the board.
 *
 * The required initialization (dc_init()) and periodic process.
 */
#include <pthread.h>
#include "stdint.h"
#include "mcu_periph/gpio.h"
#ifndef SONAR_SRF05_H
#define SONAR_SRF05_H

extern uint32_t distance_cm;
extern void sonar_srf05_init(void);

/** Periodic */
extern void sonar_srf05_periodic(void);

extern uint16_t reading_function(void);
extern uint16_t pulsin (ioportid_t port, uint16_t gpios, uint16_t time_out );
#endif // SONAR_SRF05_H
