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
#include "mcu_periph/sys_time.h"

// Include Standard Camera Control Interface
#include "dc.h"

#include "modules/autmav/sony_a7r_handler.h"
#include "modules/autmav/RTK_receive.h"

struct link_device *time_port;
union float_2_byte f2b;

void wifi_cam_ctrl_init(void)
{
  // Call common DC init
  time_port = &((uart4).device);
  uart_periph_set_bits_stop_parity(&uart4, UBITS_8, USTOP_1, UPARITY_NO);
  uart_periph_set_baudrate(&uart4, B115200);
  dc_init();
}

void wifi_cam_ctrl_periodic(void)
{
  // Common DC Periodic task
  dc_periodic();
}

void time_streamer(void){
  float current_time = get_sys_time_float();
  f2b.value = current_time;

  char next_line = '\n';

  time_port->put_byte(time_port->periph, 0, (uint8_t)f2b.bytes[0]);
  time_port->put_byte(time_port->periph, 0, (uint8_t)f2b.bytes[1]);
  time_port->put_byte(time_port->periph, 0, (uint8_t)f2b.bytes[2]);
  time_port->put_byte(time_port->periph, 0, (uint8_t)f2b.bytes[3]);
  time_port->put_byte(time_port->periph, 0, (uint8_t)next_line);
}

/* Command The Camera */
void dc_send_command(uint8_t cmd)
{
  switch (cmd) {
    case DC_SHOOT:
      log_shot_command_time_stamp = get_sys_time_float();
      shoot();
#ifndef SITL      
      tag_image();
#endif 
      break;
    default:
      break;
  }
}
