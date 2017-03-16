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

/**
 * @file modules/digital_cam/gpio_cam_ctrl.c
 * Control the camera via GPIO pins.
 *
 * Configuration (DC_SHUTTER is mandatory, others optional):
 * @code{.xml}
 * <define name="DC_SHUTTER_GPIO" value="GPIOC,GPIO12"/>
 * <define name="DC_ZOOM_IN_GPIO" value="GPIOC,GPIO2"/>
 * <define name="DC_ZOOM_OUT_GPIO" value="GPIOC,GPIO5"/>
 * <define name="DC_POWER_GPIO" value="GPIOB,GPIO1"/>
 * <define name="DC_POWER_OFF_GPIO" value="GPIOC,GPIO1"/>
 * @endcode
 */

#include "wifi_cam_ctrl.h"
#include "generated/airframe.h"
#include "generated/modules.h"

// Include Standard Camera Control Interface
#include "dc.h"

#include "modules/autmav/sony_a7r_handler.h"

void wifi_cam_ctrl_init(void)
{
  // Call common DC init
  dc_init();
}

void wifi_cam_ctrl_periodic(void)
{
  // Common DC Periodic task
  dc_periodic();
}

/* Command The Camera */
void dc_send_command(uint8_t cmd)
{
  switch (cmd) {
    case DC_SHOOT:
      shoot();
#ifndef SITL      
      tag_image();
#endif 
      break;
    default:
      break;
  }
}
