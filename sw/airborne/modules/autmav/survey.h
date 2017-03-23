/*
############################## AUTMAV ##############################
# PROJECT: AUTMAV MODIFIED PAPARAZZI				               #
# DEVELOPED BY: ALI JAMEEI							               #
# Copyright (C) 2014-2017								           #	
####################################################################
*/


/* DESCRIPTION: 
	The main survey mission module 
*/ 

#ifndef SURVEY_NAV_H
#define SURVEY_NAV_H

#include "std.h"
#include "paparazzi.h"

#include <stdio.h>
#include "subsystems/datalink/datalink.h" // dl_buffer

extern void nav_survey_photo_init(void);
extern bool nav_survey_photo_run(void);
extern void send_mission_ack(void);


enum survey_stage_enum {SRV_IDLE, SRV_FLYOVER_SETUP, SRV_TURN, SRV_APPROACH, SRV_FLYOVER};
extern enum survey_stage_enum survey_stage;

extern uint8_t survey_mission_available;
extern uint16_t survey_current_wp;
extern float survey_nav_radius;
extern float survey_angle_deg;
extern uint16_t survey_nb_wp;

static inline void parse_DL_SURVEY_MISSION(void)
{
  //uint8_t packet_id = DL_RTCM_INJECT_packet_id(dl_buffer);
	survey_mission_available = DL_SURVEY_MISSION_mission_availabe(dl_buffer);
	survey_nb_wp = DL_SURVEY_MISSION_total_wp(dl_buffer);
	survey_angle_deg = DL_SURVEY_MISSION_survey_dir(dl_buffer);
	survey_nav_radius = DL_SURVEY_MISSION_radius(dl_buffer);
	send_mission_ack();
}

#endif
