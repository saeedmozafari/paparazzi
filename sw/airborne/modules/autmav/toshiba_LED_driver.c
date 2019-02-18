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

#include <stdint.h>
#include "state.h"
#include <math.h>

#include "toshiba_LED_driver.h"
#include "mcu_periph/i2c.h"
//#include "RTK_receive.h"

// Messages
#include "pprzlink/messages.h"
#include "subsystems/datalink/downlink.h"
#include "subsystems/datalink/datalink.h"
#include "subsystems/datalink/telemetry.h"

#include "state.h"

#ifndef TOSHIBA_I2C_DEV
#define TOSHIBA_I2C_DEV i2c2
#endif

#define TOSHIBA_LED_I2C_ADDR 0x55 << 1
#define TOSHIBA_LED_PWM0    0x01    // pwm0 register
#define TOSHIBA_LED_PWM1    0x02    // pwm1 register
#define TOSHIBA_LED_PWM2    0x03    // pwm2 register
#define TOSHIBA_LED_ENABLE  0x04    // enable register

uint8_t red;
uint8_t green;
uint8_t blue;
struct led toshiba_led;


static void send_led_status(struct transport_tx *trans, struct link_device *dev)
{
  // uint32_t dummy_1 = arm_second_;

  // uint32_t dummy_2 = current_time_;
  // pprz_msg_send_LED_STATUS(trans, dev, AC_ID,
  //                          &dummy_1, &dummy_2);
}

void toshiba_led_init(void)
{
  // register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_LED_STATUS, send_led_status);

  toshiba_led.trans.status = I2CTransDone;
  toshiba_led.addr = TOSHIBA_LED_I2C_ADDR;
  toshiba_led.trans.buf[0] = TOSHIBA_LED_ENABLE;
  toshiba_led.trans.buf[1] = 0x03;
  
  red = 0;
  green = 0;
  blue = 15;
   

  i2c_transmit(&TOSHIBA_I2C_DEV, &toshiba_led.trans, toshiba_led.addr, 2);
  toshiba_led.trans.buf[0] = TOSHIBA_LED_PWM0 ;
  toshiba_led.trans.buf[1] = blue;
  toshiba_led.trans.buf[2] = green;
  toshiba_led.trans.buf[3] = red;

  i2c_transmit(&TOSHIBA_I2C_DEV, &toshiba_led.trans, toshiba_led.addr, 4);

}


void toshiba_led_event(void)
{
  switch (toshiba_led.trans.status) {
    case I2CTransPending:
      // wait and do nothing
      break;
    case I2CTransRunning:
      // wait and do nothing
      break;
    case I2CTransSuccess:
      // set to done
      toshiba_led.trans.status = I2CTransDone;
      // increment status
      break;
    case I2CTransFailed:
      // set to done
      toshiba_led.trans.status = I2CTransDone;
      break;
    case I2CTransDone:
      // do nothing
      break;
    default:
      break;
  }
}


void toshiba_led_periodic(void)
{
toshiba_led.trans.buf[0] = TOSHIBA_LED_ENABLE;
  toshiba_led.trans.buf[1] = 0x03;
  
  red = 0;
  green = 0;
  blue = 15;
   

  i2c_transmit(&TOSHIBA_I2C_DEV, &toshiba_led.trans, toshiba_led.addr, 2);
  toshiba_led.trans.buf[0] = TOSHIBA_LED_PWM0 ;
  toshiba_led.trans.buf[1] = blue;
  toshiba_led.trans.buf[2] = green;
  toshiba_led.trans.buf[3] = red;

  i2c_transmit(&TOSHIBA_I2C_DEV, &toshiba_led.trans, toshiba_led.addr, 4);
  // static uint8_t throttle_cnt = 0;
  // static float current_time;

  // current_time_ = current_time;

  // current_time = get_sys_time_float();

 
  //     if (flag) {
  //       toshiba_led.trans.buf[0] = TOSHIBA_LED_ENABLE;
  //       toshiba_led.trans.buf[1] = 0x03;

  //     } else {
  //       toshiba_led.trans.buf[0] = TOSHIBA_LED_ENABLE;
  //       toshiba_led.trans.buf[1] = 0x02;
  //     }
  //     i2c_transmit(&TOSHIBA_I2C_DEV, &toshiba_led.trans, toshiba_led.addr, 2);
      
  //     turn = 0;
  //     counter = 0;
  //     toshiba_led.trans.buf[0] = TOSHIBA_LED_ENABLE;
  //     toshiba_led.trans.buf[1] = 0x03;
  //     i2c_transmit(&TOSHIBA_I2C_DEV, &toshiba_led.trans, toshiba_led.addr, 2);
     
  //     red = 15;
  // green = 0;
  // blue = 0;
  // toshiba_led.trans.buf[0] = TOSHIBA_LED_PWM0 ;
  // toshiba_led.trans.buf[1] = blue;
  // toshiba_led.trans.buf[2] = green;
  // toshiba_led.trans.buf[3] = red;

  // i2c_transmit(&TOSHIBA_I2C_DEV, &toshiba_led.trans, toshiba_led.addr, 4);

}

