#include "advanced_landing.h"
//#include "subsystems/navigation/waypoints.h"
#include "firmwares/fixedwing/nav.h"
#include "subsystems/gps.h"
#include "modules/lidar/lidar_sf11.h"

#ifndef NAV_ADVANCED_LANDING_APP_DIST
#define NAV_ADVANCED_LANDING_APP_DIST 300
#endif

#ifndef NAV_ADVANCED_LANDING_DIRECTION
#define NAV_ADVANCED_LANDING_DIRECTION 100
#endif

#ifndef NAV_ADVANCED_LANDING_FLAIR_TIME_TRESH
#define NAV_ADVANCED_LANDING_FLAIR_TIME_TRESH 0.5
#endif

float nav_advanced_landing_app_dist;
float nav_advanced_landing_direction;
float nav_advanced_landing_flair_time_tresh;

void advanced_landing_setup(void){
	
	nav_advanced_landing_app_dist = NAV_ADVANCED_LANDING_APP_DIST;
	nav_advanced_landing_direction = NAV_ADVANCED_LANDING_DIRECTION;
	nav_advanced_landing_flair_time_tresh = NAV_ADVANCED_LANDING_FLAIR_TIME_TRESH;
}

void calc_turning_point(uint8_t home_WP, uint8_t approach_pos, uint8_t target_WP, uint8_t center_WP){
	float rel_pos_x = 0.0, rel_pos_y = 0.0, rel_tar_x = 0.0, rel_tar_y = 0.0, rel_center_x = 0.0, rel_center_y = 0.0;

	rel_pos_x = nav_advanced_landing_app_dist * sinf(nav_advanced_landing_direction * 3.1415 / 180.0);
	rel_pos_y = nav_advanced_landing_app_dist * cosf(nav_advanced_landing_direction * 3.1415 / 180.0);

	WaypointX(approach_pos) = rel_pos_x + WaypointX(home_WP);
	WaypointY(approach_pos) = rel_pos_y + WaypointY(home_WP);

	rel_tar_x = rel_pos_x - 2 * nav_radius * cosf(nav_advanced_landing_direction * 3.1415 / 180.0);
	rel_tar_y = rel_pos_y + 2 * nav_radius * sinf(nav_advanced_landing_direction * 3.1415 / 180.0);

	WaypointX(target_WP) = rel_tar_x + WaypointX(home_WP);
	WaypointY(target_WP) = rel_tar_y + WaypointY(home_WP);

	rel_center_x = rel_pos_x - nav_radius * cosf(nav_advanced_landing_direction * 3.1415 / 180.0);
	rel_center_y = rel_pos_y + nav_radius * sinf(nav_advanced_landing_direction * 3.1415 / 180.0);

	WaypointX(center_WP) = rel_center_x + WaypointX(home_WP);
	WaypointY(center_WP) = rel_center_y + WaypointY(home_WP);
}

void set_turning_direction(int16_t dir){
	nav_advanced_landing_direction = dir;
}

void set_approach_distance(int16_t dist){
	nav_advanced_landing_app_dist = dist;
}

void set_sf11_agl_mode(bool state){
	if(state == TRUE){
		lidar_sf11.update_agl = 1;
	}
	else{
	 	lidar_sf11.update_agl = 0;
	}
}