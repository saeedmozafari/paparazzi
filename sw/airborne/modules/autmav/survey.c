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

#include "survey.h"

#include "firmwares/fixedwing/nav.h"
#include "state.h"

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

uint8_t survey_mission_available;
bool survey_first_time_setup;


enum survey_stage_enum survey_stage;

float survey_nav_radius;
float survey_angle_deg;

uint16_t survey_current_wp;
uint16_t survey_last_wp;
uint16_t survey_nb_wp;
uint16_t survey_flyover_start_wp;
uint16_t survey_flyover_end_wp;

static void send_survey_status(struct transport_tx *trans, struct link_device *dev)
{
	uint8_t stage = survey_stage;
  pprz_msg_send_SURVEY_MISSION_STATUS(trans, dev, AC_ID,
                        &survey_mission_available,
                        &survey_nb_wp,
                        &survey_angle_deg,
                        &survey_nav_radius,
                        &stage,
                        &survey_flyover_start_wp,
                        &survey_current_wp,
                        &survey_flyover_end_wp);
}

void send_mission_ack(void) {
	pprz_msg_send_MISSION_ACK(&(DefaultChannel).trans_tx, &(DefaultDevice).device, AC_ID);
}
void nav_survey_photo_init(void) {

	survey_mission_available = false;
	survey_first_time_setup =false;
	survey_current_wp = 0;
	survey_stage = SRV_IDLE;
	survey_nav_radius = 0;
	survey_angle_deg = 0;
	survey_nb_wp = 0;
	survey_last_wp = 0;
	register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_SURVEY_MISSION_STATUS, send_survey_status);
	uint16_t i;
	for(i=0; i < NB_WAYPOINT; i++) {
		turn_waypoint[i] = false;
		approach_waypoint[i] = false;
	}
}

bool nav_survey_photo_run(void) {

	if(survey_mission_available) {
		
		if(!survey_first_time_setup) {
		
			uint16_t i = 0;
			while((approach_waypoint[i] = false)&&(turn_waypoint[i] == false)&&(i<NB_WAYPOINT)) i++;
			survey_last_wp = i + survey_nb_wp - 1;

			if(turn_waypoint[i] == true) {
				survey_current_wp = i;
				survey_stage = SRV_TURN;
			} else {
				survey_stage = SRV_IDLE;
			}
			survey_first_time_setup = true;
#ifdef DIGITAL_CAM
      		dc_stop();
#endif	
		}

		NavVerticalAutoThrottleMode(0.0);
  		NavVerticalAltitudeMode(waypoints[survey_current_wp].a, 0.0);
		
		switch (survey_stage){
			
			case SRV_IDLE:
			
			break;

			case SRV_TURN:
				nav_circle_XY(waypoints[survey_current_wp].x, waypoints[survey_current_wp].y, survey_nav_radius);
#ifdef DIGITAL_CAM
      			dc_stop();
#endif	
    			if (NavCourseCloseTo(survey_angle_deg)
        			&& nav_approaching_xy(waypoints[survey_current_wp+1].x, waypoints[survey_current_wp+1].y, last_x, last_y, CARROT)
        			&& fabs(stateGetPositionUtm_f()->alt - waypoints[survey_current_wp].a) <= 10) {

    				survey_stage = SRV_APPROACH;
    				survey_current_wp++;

    				nav_init_stage();
    			}
			break;

			case SRV_APPROACH:
				nav_route_xy(waypoints[survey_current_wp].x, waypoints[survey_current_wp].y,waypoints[survey_current_wp+1].x, waypoints[survey_current_wp+1].y);
				if (nav_approaching_xy(waypoints[survey_current_wp+1].x, waypoints[survey_current_wp+1].y,waypoints[survey_current_wp].x, waypoints[survey_current_wp].y, 1.0)){

					survey_stage = SRV_FLYOVER_SETUP;
					nav_init_stage();
					survey_current_wp++;
      			}
			break;

			case SRV_FLYOVER_SETUP:
				survey_flyover_start_wp = survey_current_wp;
				uint16_t i = survey_flyover_start_wp;
				while((turn_waypoint[i] == false)&&(i<NB_WAYPOINT)) i++;
				survey_flyover_end_wp = i-1;
				survey_stage = SRV_FLYOVER;
#ifdef DIGITAL_CAM
      				dc_start_shooting();
#endif
			break;

			case SRV_FLYOVER:
				nav_route_xy(waypoints[survey_flyover_start_wp].x, waypoints[survey_flyover_start_wp].y, waypoints[survey_flyover_end_wp].x, waypoints[survey_flyover_end_wp].y); 
				if(turn_waypoint[survey_current_wp] == true) {
					survey_stage = SRV_TURN;
#ifdef DIGITAL_CAM
      				dc_stop();
#endif		
				}
				if(survey_current_wp == survey_last_wp){
#ifdef DIGITAL_CAM
      				dc_stop();
#endif
					survey_mission_available = false;
				}
			break;
		}
		 
		return true;
	} 
	else
	{
		return false;
	}


}
