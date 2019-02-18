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
 * <define name="TRIGGER_GPIO" value="GPIOC,GPIO05"/>
 * <define name="ECHO_GPIO" value="GPIOB,GPIO15"/>
 * @endcode
 */
#include "mcu_periph/pwm_input.h"
#include "sonar_SRF05.h"
#include "generated/airframe.h"
#include "generated/modules.h"
#include "mcu_periph/gpio_arch.h"
#include "subsystems/datalink/downlink.h"
#include "mcu_periph/sys_time_arch.h"
#include "pprzlink/messages.h"
#include "subsystems/datalink/datalink.h"
#include "subsystems/datalink/telemetry.h"
#include "filters/rms_filter.h"
#include "subsystems/abi.h"

#ifndef TRIG_LOW
#define TRIG_LOW gpio_clear
#endif

#ifndef TRIG_HIGH
#define TRIG_HIGH gpio_set
#endif

#ifndef TRIGGER_GPIO
#error DC: Please specify  a TRIGGER_GPIO (e.g. <define name="TRIGGER_GPIO" value="GPIOC,GPIO12"/>)
#endif

float distance_cm_f;
float distance_nof;
uint32_t read_sonar;
float max_timeout = 32000 ;
float update_flag = 0;
uint32_t i = 0;

struct RMSFilterInt sonar_srf_filter;
static void send_sonar_debug(struct transport_tx *trans, struct link_device *dev)
{
  pprz_msg_send_SONAR_DEBUG(trans, dev, AC_ID,
                            &distance_cm_f, &distance_nof, &read_sonar);
}
void sonar_srf05_init(void)
{
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_SONAR_DEBUG, send_sonar_debug);

  distance_cm_f = 0;
  read_sonar = 0;
  gpio_setup_output(TRIGGER_GPIO);
  TRIG_LOW(TRIGGER_GPIO);
  update_flag = 0;
  init_rms_filter(&sonar_srf_filter);
}



void sonar_srf05_periodic(void)
{
  TRIG_HIGH(TRIGGER_GPIO);
  for (i = 0; i < 1500; i++) { /* Wait a bit. */
    __asm__("nop");
  }
  TRIG_LOW(TRIGGER_GPIO);//to ensure clean high pulse
  read_sonar = get_pwm_input_duty_in_usec(SONAR_PWM_CHANNEL);
  distance_nof=read_sonar/58;
  distance_nof=distance_nof/100;
  distance_cm_f=update_rms_filter(&sonar_srf_filter, distance_nof);
  //distance_cm_f = distance_cm_f / 58;

AbiSendMsgAGL(AGL_SONAR_ADC_ID, distance_cm_f);
  /*#ifdef SENSOR_SYNC_SEND_SONAR
    // Send Telemetry report
    DOWNLINK_SEND_SONAR_ALTITUDE(DefaultChannel, DefaultDevice, &distance_cm);
  #endif*/

}