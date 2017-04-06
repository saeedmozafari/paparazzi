/*
############################## AUTMAV ##############################
# PROJECT: AUTMAV MODIFIED PAPARAZZI                               #
# DEVELOPED BY: ALI JAMEEI                                         #
# Copyright (C) 2014-2017                                          #
####################################################################
*/


/* DESCRIPTION:
  The main survey mission module
*/

#include "survey.h"

#include "firmwares/fixedwing/nav.h"
#include "state.h"
#include "autopilot.h"
#include "generated/flight_plan.h"

#ifdef DIGITAL_CAM
#include "modules/digital_cam/dc.h"
#endif

#include "std.h"
#include "paparazzi.h"
#include <stdio.h>

#include "math/pprz_algebra_float.h"
#include "subsystems/navigation/common_nav.h"

#include "subsystems/datalink/datalink.h"
#include "subsystems/datalink/telemetry.h"

float mission_max_distance;
float mission_max_height;
float mission_min_height;
uint32_t mission_after_to;
uint32_t mission_after_mission;
float mission_transition_alt;
float mission_land_direction;
float mission_shot_pitch_angle;
uint32_t mission_strong_wind;
uint32_t mission_poor_gps;
uint32_t mission_low_endurance;
uint32_t mission_camera_malfunction;
uint32_t mission_link_loss;
uint32_t mission_ground_proximity;
uint32_t failsafe_flags;


void initialize_mission_vars(void)
{
  mission_max_distance = 5000;
  mission_max_height = 0;
  mission_min_height = 0;
  mission_after_to = 0;
  mission_after_mission = 0;
  mission_transition_alt = 0;
  mission_land_direction = 0;
  mission_shot_pitch_angle = 0;
  mission_strong_wind = 0;
  mission_poor_gps = 0;
  mission_low_endurance = 0;
  mission_camera_malfunction = 0;
  mission_link_loss = 0;
  mission_ground_proximity = 0;
  failsafe_flags = 0;
}
void mission_handler_init(void)
{
  initialize_mission_vars();
}

void send_mission_ack(uint8_t id)
{
  pprz_msg_send_MISSION_ACK(&(DefaultChannel).trans_tx, &(DefaultDevice).device, AC_ID, &id);
}

bool max_distance_failsafe(void) 
{
  if(mission_max_distance < dist2_to_home) {
    SetBit(failsafe_flags, MAXIMUM_DISTANCE_FS);
    return true;
  } else {
    ClearBit(failsafe_flags, MAXIMUM_DISTANCE_FS);
    return false;
  }
}

bool datalink_failsafe(void)
{
  if(datalink_time > 30) {
    SetBit(failsafe_flags, DATALINK_FS);
    return true;
  } else {
    ClearBit(failsafe_flags, DATALINK_FS);
    return false;
  }
}
