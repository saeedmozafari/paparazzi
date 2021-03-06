/*
 * Copyright (C) 2008-2009 Antoine Drouin <poinix@gmail.com>
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
 * @file stabilization_attitude_euler_float.h
 *
 * Rotorcraft attitude stabilization in euler float version.
 */

#ifndef STABILIZATION_ATTITUDE_EULER_FLOAT_H
#define STABILIZATION_ATTITUDE_EULER_FLOAT_H

#include "math/pprz_algebra_float.h"
#include "firmwares/rotorcraft/stabilization/stabilization_attitude_common_float.h"
#include "firmwares/rotorcraft/stabilization/stabilization_attitude_ref_euler_float.h"

extern struct FloatAttitudeGains stabilization_gains;
extern struct FloatEulers stabilization_att_sum_err;

extern struct FloatEulers stab_att_sp_euler;
extern struct AttRefEulerFloat att_ref_euler_f;

extern float lambda_roll;
extern float lambda_pitch;
extern float lambda_yaw;

extern float beta_roll;
extern float beta_pitch;
extern float beta_yaw;

extern float alpha_roll;
extern float alpha_pitch;
extern float alpha_yaw;

extern float mio_roll;
extern float mio_pitch;
extern float mio_yaw;

extern float e_center;
extern float roll_gain;
extern float pitch_gain;
extern float yaw_gain;

extern float roll_offset;
extern float pitch_offset;
extern float yaw_offset;

extern float t_span_roll;
extern float t_span_pitch;
extern float t_span_yaw;

/*extern float roll_time;
extern float pitch_time;
extern float yaw_time;*/

#endif /* STABILIZATION_ATTITUDE_EULER_FLOAT_H */
