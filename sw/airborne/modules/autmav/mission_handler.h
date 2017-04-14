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

#ifndef MISSION_HANDLER_H
#define MISSION_HANDLER_H

#include "std.h"
#include "paparazzi.h"

#include <stdio.h>
#include "subsystems/datalink/datalink.h" // dl_buffer

extern float mission_max_distance;

enum mission_ack_enum {
	
	SURVEY_MISSION_ACK = 1,
	MAXIMUM_DISTANCE_ACK,
	MAXIMUM_HEIGHT_ACK,
	MINIMUM_HEIGHT_ACK,
	TRANSITION_ALT_ACK,
	AFTER_TO_ACK,
	AFTER_MISSION_ACK,
	LAND_DIRECTION_ALT,
	SHOT_PITCH_ANGLE_ACK,
	CAMERA_SETTINGS_ACK,
	STRONG_WIND_FAILSAFE_ACK,
	POOR_GPS_FAILSAFE_ACK,
	LOW_ENDURANCE_FAILSAFE_ACK,
	CAMERA_MALFUNCTION_FAILSAFE_ACK,
	LINK_LOSS_FAILSAFE_ACK,
	GROUND_PROXIMTY_FAILSAFE_ACK,
	SURVEY_CLEAR_MISSION_ACK
};

enum mission_failsafe_enum {
	MAXIMUM_DISTANCE_FS,
	DATALINK_FS

};

extern float mission_max_distance;
extern float mission_max_height;
extern float mission_min_height;
extern uint32_t mission_after_to;
extern uint32_t mission_after_mission;
extern float mission_transition_alt;
extern float mission_land_direction;
extern float mission_shot_pitch_angle;
extern uint32_t mission_strong_wind;
extern uint32_t mission_poor_gps;
extern uint32_t	mission_low_endurance;
extern uint32_t mission_camera_malfunction;
extern uint32_t mission_link_loss;
extern uint32_t mission_ground_proximity;

extern void send_mission_ack(uint8_t id);
extern void mission_handler_init(void);
extern bool max_distance_failsafe(void);
extern void initialize_mission_vars(void);
extern bool datalink_failsafe(void);
extern void clean_current_mission(void);

static inline void parse_DL_SURVEY_MISSION_SETTINGS(void)
{
	uint16_t setting_id = DL_SURVEY_MISSION_SETTINGS_setting_id(dl_buffer);

	switch (setting_id) {
		case 1:
			mission_max_distance = DL_SURVEY_MISSION_SETTINGS_setting_value(dl_buffer);
			send_mission_ack(MAXIMUM_DISTANCE_ACK);
		break;

		case 2:
			mission_max_height = DL_SURVEY_MISSION_SETTINGS_setting_value(dl_buffer);
			send_mission_ack(MAXIMUM_HEIGHT_ACK);
		break;

		case 3:
			mission_min_height = DL_SURVEY_MISSION_SETTINGS_setting_value(dl_buffer);
			send_mission_ack(MINIMUM_HEIGHT_ACK);
		break;

		case 4:
			mission_transition_alt = DL_SURVEY_MISSION_SETTINGS_setting_value(dl_buffer);
			send_mission_ack(TRANSITION_ALT_ACK);
		break;

		case 5:
			mission_after_to = DL_SURVEY_MISSION_SETTINGS_setting_value(dl_buffer);
			send_mission_ack(AFTER_TO_ACK);
		break;

		case 6:
			mission_after_mission = DL_SURVEY_MISSION_SETTINGS_setting_value(dl_buffer);
			send_mission_ack(AFTER_MISSION_ACK);
		break;

		case 7:
			mission_land_direction = DL_SURVEY_MISSION_SETTINGS_setting_value(dl_buffer);
			send_mission_ack(LAND_DIRECTION_ALT);
		break;

		case 8:
			mission_shot_pitch_angle = DL_SURVEY_MISSION_SETTINGS_setting_value(dl_buffer);
			mission_shot_pitch_angle -= 1800;
			mission_shot_pitch_angle /= 10;
			send_mission_ack(SHOT_PITCH_ANGLE_ACK);
		break;

		case 9:
			mission_strong_wind = DL_SURVEY_MISSION_SETTINGS_setting_value(dl_buffer);
			send_mission_ack(STRONG_WIND_FAILSAFE_ACK);
		break;

		case 10:
			mission_poor_gps = DL_SURVEY_MISSION_SETTINGS_setting_value(dl_buffer);
			send_mission_ack(POOR_GPS_FAILSAFE_ACK);
		break;
		
		case 11:
			mission_low_endurance = DL_SURVEY_MISSION_SETTINGS_setting_value(dl_buffer);
			send_mission_ack(LOW_ENDURANCE_FAILSAFE_ACK);
		break;

		case 12:
			mission_camera_malfunction = DL_SURVEY_MISSION_SETTINGS_setting_value(dl_buffer);
			send_mission_ack(CAMERA_MALFUNCTION_FAILSAFE_ACK);
		break;

		case 13:
			mission_link_loss = DL_SURVEY_MISSION_SETTINGS_setting_value(dl_buffer);
			send_mission_ack(LINK_LOSS_FAILSAFE_ACK);
		break;

		case 14:
			mission_ground_proximity = DL_SURVEY_MISSION_SETTINGS_setting_value(dl_buffer);
			send_mission_ack(GROUND_PROXIMTY_FAILSAFE_ACK);
		break;

		case 15:
			clean_current_mission();
			initialize_mission_vars();
			send_mission_ack(SURVEY_CLEAR_MISSION_ACK);
		break;
	}
}

#endif
