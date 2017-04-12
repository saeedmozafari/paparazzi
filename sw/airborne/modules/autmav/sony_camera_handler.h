#ifndef SONY_CAMERA_HANDLER_H
#define SONY_CAMERA_HANDLER_H

#include "std.h"
#include "paparazzi.h"
#include <stdio.h>
#include "std.h"
#include "subsystems/datalink/datalink.h" 

enum camera_state{
	SET_REC_MODE,
	WAIT_FOR_REC_MODE_TO_BE_SET,
	SET_POSTVIEW_NAME,
	WAIT_FOR_POSTVIEW_TO_BE_SET,
	IDLE_MODE,
	SEND_SHOOT,
	WAIT_FOR_IMAGE_NAME,
	SETTING_MODE							
};

enum camera_model {
	SONY_A7R,
	SONY_A6000,
	SONY_QX1
};

enum parser {
	IDLE,
	GOT_e,
	GOT_er,
	GOT_err,
	GOT_erro,
	GOT_error,
	GOT_D,
	GOT_DS,
	GOT_DSC,
	GOT_J,
	GOT_JP,
	GOT_JPG,
	GOT_r,
	GOT_re,
	GOT_res,
	GOT_resu,
	GOT_resul,
	GOT_result
};

extern enum camera_state cam_state;
extern enum camera_model cam_model;
extern enum parser cam_parser;

extern uint16_t time_counter;
extern uint16_t delay_counter;
extern uint16_t sival;
extern uint16_t image_name_counter;
extern float sfval;
extern float ssvalue;
extern bool image_name_read;
extern bool mode_set;
extern bool word_processing;
extern bool dsc_received;
extern bool jpg_received;
extern char image_name[100];

extern void sony_camera_handler_setup(void);
extern void sony_camera_handler_periodic(void);

extern void wifi_response_parser(char);
extern void clear_image_name(void);
static inline void on_settings_msg_receive(void){
	
}
extern void set_rec_mode(void);
extern void set_postview_name(void);
extern void send_shoot_command(void);
extern void shoot(void);
extern void send_settings(void);
extern void get_available_shutter_speed(void);
extern void get_available_isos(void);
extern void get_available_fnumbers(void);
extern void set_fnumber(float);
extern void set_iso(uint16_t);
extern void set_shutter_speed(float);
extern void read_image_name(void);

#endif