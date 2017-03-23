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

extern void nav_survey_photo_init(void);
extern bool nav_survey_photo_run(void);

enum survey_stage_enum {SRV_IDLE, SRV_FLYOVER_SETUP, SRV_TURN, SRV_APPROACH, SRV_FLYOVER};
extern enum survey_stage_enum survey_stage;

extern bool survey_mission_available;
extern uint16_t survey_current_wp;

#endif
