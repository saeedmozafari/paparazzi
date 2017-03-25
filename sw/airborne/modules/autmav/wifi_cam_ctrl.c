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

#ifndef WIFI_SHOOT_DELAY_MS
#define WIFI_SHOOT_DELAY_MS 550
#endif

struct link_device *time_port;
union float_2_byte f2b;
bool register_shoot_command;
bool calculated_target_shot_time;
bool tag_image_on_target_rtk;
bool tag_next_rtk_msg;
bool tagged_image;

double register_shoot_command_time;
double target_shot_time;
double target_rtk_time;

void wifi_cam_ctrl_init(void)
{
#ifndef SITL  
  // time_port = &((uart4).device);
  // uart_periph_set_bits_stop_parity(&uart4, UBITS_8, USTOP_1, UPARITY_NO);
  // uart_periph_set_baudrate(&uart4, B115200);
#endif
  // Call common DC init
  dc_init();

  register_shoot_command = false;
  calculated_target_shot_time = false;
  tag_image_on_target_rtk = false;
  tag_next_rtk_msg = false;
  tagged_image = false;
}

void wifi_cam_ctrl_periodic(void)
{
  // Common DC Periodic task
  dc_periodic();
#ifndef SITL
  double now = get_sys_time_float();

  if(register_shoot_command == true) {
    
    tagged_image = false;
    double next_rtk_real_data_time = last_rtk_msg_time + RTK_DATA_PERIOD_MS / 1000.0 - RTK_DATA_LAG_MS / 1000.0;
    if(next_rtk_real_data_time >= register_shoot_command_time) {

      target_rtk_time = next_rtk_real_data_time;
      while ((target_rtk_time - now) < (WIFI_SHOOT_DELAY_MS / 1000.0)) target_rtk_time += (RTK_DATA_PERIOD_MS / 1000.0);
      target_shot_time = target_rtk_time - (WIFI_SHOOT_DELAY_MS / 1000.0);
      log_target_rtk_time_stamp = target_shot_time;
      calculated_target_shot_time = true;
      tag_image_on_target_rtk = true;
      register_shoot_command = false;

    }
  }

  if((calculated_target_shot_time) && (target_shot_time < now)) {

    shoot();
    log_shot_command_time_stamp = get_sys_time_float();
    calculated_target_shot_time = false;

  }

  if((tag_image_on_target_rtk) && (target_rtk_time < now)) {
    
    log_phi = DegOfRad(stateGetNedToBodyEulers_f()->phi);
    log_theta = DegOfRad(stateGetNedToBodyEulers_f()->theta);
    log_psi = DegOfRad(stateGetNedToBodyEulers_f()->psi);
    tag_next_rtk_msg = true;
    tag_image_on_target_rtk = false;
  }

    tag_image();
  #endif 
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
      //shoot();
      dc_send_shot_position();
      register_shoot_command = true;
      register_shoot_command_time = get_sys_time_float();
      break;
    default:
      break;
  }
}
