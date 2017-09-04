/*
 * Copyright (C) 2015  Gautier Hattenberger, 2013 Tobias Münch
 * Copyright (C) 2016  Moses Wang <moses.hao@gmail.com> based sonar_adc.c
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

#include "modules/autmav/sonar_i2c.h"
#include "generated/airframe.h"
#include "led.h"
#include "subsystems/abi.h"
#ifdef SITL
#include "state.h"
#endif

#include "mcu_periph/uart.h"
#include "pprzlink/messages.h"
#include "subsystems/datalink/downlink.h"



#ifndef SONAR_I2C_DEV
#define SONAR_I2C_DEV i2c0
#endif

/* address can be 0xEC or 0xEE (CSB\ low = 0xEE) */
#ifndef SONAR_SLAVE_ADDR
#define SONAR_SLAVE_ADDR 0x70
#endif

struct SonarI2c sonar_i2c;

void sonar_i2c_init(void)
{
  /* set i2c_peripheral */
  sonar_i2c.i2c_p = &SONAR_I2C_DEV;
  /* set initial status: Success or Done */
  sonar_i2c.i2c_trans.slave_addr = SONAR_SLAVE_ADDR;
  sonar_i2c.i2c_trans.status = I2CTransDone;     
}


/** Read value to update sonar measurement
 */
void sonar_i2c_read(void)
{
  float distance = 0;
  i2c_transceive(sonar_i2c.i2c_p, &(sonar_i2c.i2c_trans),sonar_i2c.i2c_trans.slave_addr,0,3);
  if(sonar_i2c.i2c_trans.buf[2] == (sonar_i2c.i2c_trans.buf[0]+sonar_i2c.i2c_trans.buf[1])&0xff) //check
  {
    distance = ((sonar_i2c.i2c_trans.buf[0] << 8) | sonar_i2c.i2c_trans.buf[1]);
    sonar_i2c.meas = distance;
    sonar_i2c.distance = distance / 100;  //module is cm
    // Send ABI message
    // AbiSendMsgAGL(AGL_SONAR_ADC_ID, sonar_i2c.distance);
      
#ifdef SENSOR_SYNC_SEND_SONAR
    // Send Telemetry report
    DOWNLINK_SEND_SONAR_ALTITUDE(DefaultChannel, DefaultDevice, &sonar_i2c.distance);
#endif
  }

}


