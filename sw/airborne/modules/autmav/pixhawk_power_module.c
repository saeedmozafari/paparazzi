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

#include "pixhawk_power_module.h"
#include "mcu_periph/adc.h"
#include "mcu_periph/uart.h"
#include "pprzlink/messages.h"
#include "subsystems/datalink/downlink.h"

uint16_t adc_voltage_val;
uint16_t adc_current_val;


#ifndef VOLTAGE_ADC_CHANNEL
#error "please define the voltage adc channel"
#endif

#ifndef CURRENT_ADC_CHANNEL
#error "please define the current adc channel"
#endif

#ifndef ADC_CHANNEL_GENERIC_NB_SAMPLES
#define ADC_CHANNEL_GENERIC_NB_SAMPLES DEFAULT_AV_NB_SAMPLE
#endif

#if PERIODIC_TELEMETRY
#include "subsystems/datalink/telemetry.h"
#endif

#ifndef ADC_GENERIC_PERIODIC_SEND
#define ADC_GENERIC_PERIODIC_SEND FALSE
#endif

#ifdef VOLTAGE_ADC_CHANNEL
static struct adc_buf buf_voltage;
#endif

#ifdef CURRENT_ADC_CHANNEL
static struct adc_buf buf_current;
#endif

static void adc_msg_send(struct transport_tx *trans, struct link_device *dev) {
#ifdef VOLTAGE_ADC_CHANNEL
  adc_voltage_val = buf_voltage.sum / buf_voltage.av_nb_sample;
#endif
#ifdef CURRENT_ADC_CHANNEL
  adc_current_val = buf_current.sum / buf_current.av_nb_sample;
#endif
  pprz_msg_send_ADC_GENERIC(trans, dev, AC_ID, &adc_voltage_val, &adc_current_val);
}

void power_module_init(void)
{
  adc_voltage_val = 0;
  adc_current_val = 0;

#ifdef VOLTAGE_ADC_CHANNEL
  adc_buf_channel(VOLTAGE_ADC_CHANNEL, &buf_voltage, ADC_CHANNEL_GENERIC_NB_SAMPLES);
#endif
#ifdef CURRENT_ADC_CHANNEL
  adc_buf_channel(CURRENT_ADC_CHANNEL, &buf_current, ADC_CHANNEL_GENERIC_NB_SAMPLES);
#endif
#if PERIODIC_TELEMETRY
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_ADC_GENERIC, adc_msg_send);
#endif

}

void power_module_periodic(void)
{
#if ADC_GENERIC_PERIODIC_SEND
  adc_msg_send(&(DefaultChannel).trans_tx, &(DefaultDevice).device);
#endif
}

