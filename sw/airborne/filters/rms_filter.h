/*
 * Copyright (c) 2012 Ted Carancho. (AeroQuad)
 * (c) 2012 Gautier Hattenberger
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

#ifndef RMS_H
#define RMS_H
#endif

#ifndef RMS_DATASIZE
#define RMS_DATASIZE 5
#endif

#include <math.h>
#include "std.h"
#include "math/pprz_algebra_int.h"

struct RMSFilterInt {
  float data[RMS_DATASIZE];
  float rms_data;
  int8_t dataIndex;
};

inline void init_rms_filter(struct RMSFilterInt *filter);
inline float update_rms_filter(struct RMSFilterInt *filter, float new_data);
inline int32_t get_rms_filter(struct RMSFilterInt *filter);

inline void init_rms_filter(struct RMSFilterInt *filter)
{
  int i;
  for (i = 0; i < RMS_DATASIZE; i++) {
    filter->data[i] = 0;
    filter->rms_data=0;
  }
  filter->dataIndex = 0;
}

inline float update_rms_filter(struct RMSFilterInt *filter, float new_data)
{
  int i; // used to sort array
  float rms_avg=0;
  float rms_sum=0;
  float tmp=0;

    // Insert new data into raw data array round robin style
  filter->data[filter->dataIndex] = new_data;
  if (filter->dataIndex < (RMS_DATASIZE - 1)) {
    filter->dataIndex++;
  } else {
    filter->dataIndex = 0;
  }

    for (i = 0; i < RMS_DATASIZE; i++) {
    tmp=(filter->data[i])*(filter->data[i]) ;
    rms_sum+=tmp;
  }
  rms_avg=rms_sum/RMS_DATASIZE;
  filter->rms_data=sqrt(rms_avg);

  return filter->rms_data;
}

inline int32_t get_rms_filter(struct RMSFilterInt *filter)
{
  return filter->rms_data;
}