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
extern void clean_current_mission(void);

enum survey_stage_enum {SRV_IDLE, SRV_TURN_SETUP, SRV_FLYOVER_SETUP, SRV_TURN, SRV_APPROACH, SRV_FLYOVER};
extern enum survey_stage_enum survey_stage;

extern uint8_t survey_mission_available;
extern uint8_t survey_perpendicular;
extern uint16_t survey_current_wp;
extern float survey_nav_radius;
extern float survey_angle_deg;
extern uint16_t survey_nb_wp;
extern uint16_t survey_flyover_start_wp;
extern uint16_t survey_flyover_end_wp;
extern float survey_trigger_distance;
extern float survey_side_distance;
extern uint8_t survey_devision;

extern struct point turn_waypoint;
extern struct point approach_waypoint;

static inline void parse_DL_SURVEY_MISSION(void)
{
  	clean_current_mission();	

  	survey_mission_available = DL_SURVEY_MISSION_mission_availabe(dl_buffer);
  	survey_perpendicular = DL_SURVEY_MISSION_perpendicular(dl_buffer);
	survey_nb_wp = DL_SURVEY_MISSION_total_wp(dl_buffer);
	survey_angle_deg = DL_SURVEY_MISSION_survey_dir(dl_buffer);
	survey_side_distance = DL_SURVEY_MISSION_side_distance(dl_buffer);
	survey_trigger_distance = DL_SURVEY_MISSION_trig_distance(dl_buffer);
	send_mission_ack();
}

#endif
