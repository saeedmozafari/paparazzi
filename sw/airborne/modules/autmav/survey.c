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

uint8_t survey_mission_available;
uint8_t survey_perpendicular;
bool survey_first_time_setup;


enum survey_stage_enum survey_stage;

float survey_nav_radius;
float survey_trigger_distance;
float survey_side_distance;
float survey_angle_deg;

uint16_t survey_current_wp;
uint16_t survey_last_wp;
uint16_t survey_nb_wp;
uint16_t survey_flyover_start_wp;
uint16_t survey_flyover_end_wp;

struct point turn_waypoint;
struct point approach_waypoint;

static void send_survey_status(struct transport_tx *trans, struct link_device *dev)
{
  uint8_t stage = survey_stage;
  pprz_msg_send_SURVEY_MISSION_STATUS(trans, dev, AC_ID,
                                      &survey_mission_available,
                                      &survey_perpendicular,
                                      &survey_nb_wp,
                                      &survey_angle_deg,
                                      &survey_trigger_distance,
                                      &survey_side_distance,
                                      &stage,
                                      &survey_flyover_start_wp,
                                      &survey_current_wp,
                                      &survey_flyover_end_wp);
}

void nav_survey_photo_init(void)
{
  clean_current_mission();
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_SURVEY_MISSION_STATUS, send_survey_status);
}

bool nav_vicinity_xy(float x, float y, float approaching_distance)
{
  /** distance to waypoint in x */
  float pw_x = x - stateGetPositionEnu_f()->x;
  /** distance to waypoint in y */
  float pw_y = y - stateGetPositionEnu_f()->y;

  if (pw_x * pw_x + pw_y * pw_y <= approaching_distance * approaching_distance) {
    return true;
  } else {
    return false;
  }
}

void clean_current_mission(void)
{

  survey_mission_available = false;
  survey_perpendicular = 0;
  survey_first_time_setup = false;
  survey_current_wp = 0;
  survey_stage = SRV_IDLE;
  survey_trigger_distance = 0.0;
  survey_side_distance = 0.0;
  survey_nav_radius = 0;
  survey_angle_deg = 0;
  survey_nb_wp = 0;
  survey_last_wp = 0;
  turn_waypoint.x = 0.0;
  turn_waypoint.y = 0.0;
  turn_waypoint.a = 0.0;
  approach_waypoint.x = 0.0;
  approach_waypoint.y = 0.0;
  approach_waypoint.a = 0.0;
  uint16_t i;
  for (i = 0; i < NB_WAYPOINT; i++) {
    start_wp[i] = false;
    end_wp[i] = false;
  	waypoints[i].x = 0;
  	waypoints[i].y = 0;
  	waypoints[i].a = 0;
  }
}

bool nav_survey_photo_run(void)
{ 
  static uint8_t last_dc_value = 0;
  if (survey_mission_available) {

    if (!survey_first_time_setup) {

      survey_nav_radius = Max(fabs(survey_side_distance), nav_radius);
      survey_last_wp = survey_nb_wp + 10;
      survey_current_wp = 11;
      survey_stage = SRV_TURN_SETUP;
      survey_first_time_setup = true;
#ifdef DIGITAL_CAM
      dc_stop();
#endif
    }

    NavVerticalAutoThrottleMode(0.0);
    NavVerticalAltitudeMode(waypoints[survey_current_wp].a, 0.0);

    if(survey_stage == SRV_IDLE){
      return true;  
    }

    else if(survey_stage == SRV_TURN_SETUP) {
     
#ifdef DIGITAL_CAM
        dc_stop();
#endif
        turn_waypoint.x = waypoints[survey_current_wp].x - survey_nav_radius * cosf(RadOfDeg(survey_angle_deg)) + 30.0 * sinf(
                            RadOfDeg(survey_angle_deg));
        turn_waypoint.y = waypoints[survey_current_wp].y + survey_nav_radius * sinf(RadOfDeg(survey_angle_deg)) + 30.0 * cosf(
                            RadOfDeg(survey_angle_deg));
        turn_waypoint.a = waypoints[survey_current_wp].a;

        approach_waypoint.x = waypoints[survey_current_wp].x + 30.0 * sinf(RadOfDeg(survey_angle_deg));
        approach_waypoint.y = waypoints[survey_current_wp].y + 30.0 * cosf(RadOfDeg(survey_angle_deg));
        approach_waypoint.a = waypoints[survey_current_wp].a;

        survey_stage = SRV_TURN;
    }

    else if(survey_stage == SRV_TURN) {
        nav_circle_XY(turn_waypoint.x, turn_waypoint.y, survey_nav_radius);

        if (NavCourseCloseTo(survey_angle_deg + 180.0)
            && nav_vicinity_xy(approach_waypoint.x, approach_waypoint.y, fabs(survey_nav_radius) / 5.0)
            && fabs(stateGetPositionUtm_f()->alt - turn_waypoint.a) <= 5.0) {

          survey_side_distance *= -1.0;
          survey_stage = SRV_APPROACH;
          survey_nav_radius *= -1.0;
          survey_angle_deg += 180.0;

          nav_init_stage();
        }
    }

    else if(survey_stage == SRV_APPROACH) {
      nav_route_xy(approach_waypoint.x, approach_waypoint.y, waypoints[survey_current_wp].x, waypoints[survey_current_wp].y);
      if (nav_vicinity_xy(waypoints[survey_current_wp].x, waypoints[survey_current_wp].y, fabs(survey_nav_radius) / 5.0)) {

        survey_stage = SRV_FLYOVER_SETUP;
        nav_init_stage();
      }  
    }

    else if(survey_stage == SRV_FLYOVER_SETUP) {

      if (start_wp[survey_current_wp] == true) {
          survey_flyover_start_wp = survey_current_wp;
        }
        uint16_t i = survey_flyover_start_wp;
        while (!end_wp[i]) { i++; }
        if (end_wp[i] == true) {
          survey_flyover_end_wp = i;
          survey_stage = SRV_FLYOVER;
        }

#ifdef DIGITAL_CAM
//        dc_survey(survey_trigger_distance, waypoints[survey_flyover_start_wp].x, waypoints[survey_flyover_start_wp].y);
#endif
    }
    
    else if(survey_stage == SRV_FLYOVER) {

        nav_route_xy(waypoints[survey_flyover_start_wp].x, waypoints[survey_flyover_start_wp].y,
                     waypoints[survey_flyover_end_wp].x, waypoints[survey_flyover_end_wp].y);
        
        uint16_t dummy = survey_current_wp;
        uint8_t mod = dummy - floor((dummy / 2)) * 2;
                
        if (nav_approaching_xy(waypoints[survey_current_wp].x, waypoints[survey_current_wp].y, approach_waypoint.x,
                               approach_waypoint.y, 0.0)) {

          survey_current_wp = survey_current_wp + 1;

        }
        
        
        if(mod != last_dc_value) {
          if (mod == 0) {
#ifdef DIGITAL_CAM
          dc_survey(survey_trigger_distance, waypoints[survey_current_wp - 1].x, waypoints[survey_current_wp - 1].y);
#endif
          last_dc_value = mod;
          }

          if (mod == 1) {
#ifdef DIGITAL_CAM
          dc_stop();
#endif
          last_dc_value = mod;
          }
        }

        
        if (survey_current_wp == (survey_last_wp + 1)) {
#ifdef DIGITAL_CAM
          dc_stop();
#endif
          survey_mission_available = false;
          return false;
        }

        if (survey_current_wp == (survey_flyover_end_wp + 1)) {

          survey_stage = SRV_TURN_SETUP;

        }

    }

    return true;
  } else {
    return false;
  }


}
