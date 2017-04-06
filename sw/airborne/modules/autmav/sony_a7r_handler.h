#ifndef SONY_A7R_HANDLER_H
#define SONY_A7R_HANDLER_H

#include "std.h"
#include "paparazzi.h"
#include <stdio.h>
#include "std.h"
#include "subsystems/datalink/datalink.h" 
#include "mission_handler.h"

enum cam_state{
	
	READY,									//0
	WAIT_FOR_READY,							//1
	CONNECTING_TO_CAM_AP,					//2
	WAIT_FOR_CONNECTING_TO_CAM_AP,			//3
	SETTING_CIPMUX,							//4
	WAIT_FOR_SETTING_CIPMUX,				//5
	STARTING_UDP_CONNECTION,				//6
	WAIT_FOR_STARTING_UDP_CONNECTION,		//7
	REQUESTING_SEND,						//8
	WAIT_FOR_REQUESTING_SEND, 				//9
	SENDING_DISCOVERY_MSG,					//10
	WAIT_FOR_SENDING_DISCOVERY_MSG,			//11
	CLOSING_UDP_CONNECTION,					//12
	WAIT_FOR_CLOSING_UDP_CONNECTION,		//13
	STARTING_TCP_CONNECTION,				//14
	WAIT_FOR_STARTING_TCP_CONNECTION,		//15
	REQUESTING_SEND_TCP,					//16
	WAIT_FOR_REQUESTING_SEND_TCP,			//17
	SENDING_SHOOT_MODE_MSG,					//18
	WAIT_FOR_SENDING_SHOOT_MODE_MSG,		//19
	MODE_MSG_SENT,							//20							//21
	GET_AVAILABLE_SETTINGS,					//22
	SENDING_REQUEST_AS,
	WAIT_FOR_AS_REQUEST_TO_BE_SENT,
	GETTING_AVAILABLE_SHUTTER_SPEEDS,		
	WAIT_FOR_GETTING_AVAILABLE_SHUTTER_SPEEDS,
	SENDING_REQUEST_GS,
	WAIT_FOR_GS_REQUEST_TO_BE_SENT,
	GETTING_CURRENT_SHUTTER_SPEED,
	WAIT_FOR_GETTING_CURRENT_SHUTTER_SPEED,
	SENDING_REQUEST_AI,
	WAIT_FOR_AI_REQUEST_TO_BE_SENT,
	GETTING_AVAILABLE_ISOS,
	WAIT_FOR_GETTING_AVAILABLE_ISOS,
	SENDING_REQUEST_GI,
	WAIT_FOR_GI_REQUEST_TO_BE_SENT,
	GETTING_CURRENT_ISO,
	WAIT_FOR_GETTING_CURRENT_ISO,
	SENDING_REQUEST_AF,
	WAIT_FOR_AF_REQUEST_TO_BE_SENT,
	GETTING_AVAILABLE_FNUMBERS,
	WAIT_FOR_GETTING_AVAILABLE_FNUMBERS,
	SENDING_REQUEST_GF,
	WAIT_FOR_GF_REQUEST_TO_BE_SENT,
	GETTING_CURRENT_FNUMBER,
	WAIT_FOR_GETTING_CURRENT_FNUMBER,
	CAM_IDLE_MODE,
	SENDING_REQUEST_SS,
	WAIT_FOR_SS_REQUEST_TO_BE_SENT,
	SETTING_SHUTTER_SPEED,
	WAIT_FOR_SHUTTER_SPEED_TO_BE_SET,
	SENDING_REQUEST_SI,
	WAIT_FOR_SI_REQUEST_TO_BE_SENT,
	SETTING_ISO,
	WAIT_FOR_ISO_TO_BE_SET,
	SENDING_REQUEST_SF,
	WAIT_FOR_SF_REQUEST_TO_BE_SENT,
	SETTING_FNUMBER,
	WAIT_FOR_FNUMBER_TO_BE_SET,
	WAIT_FOR_REQUESTING_SEND_SHOOT_TCP,		//23
	WAIT_FOR_SENDING_SHOOT_COMMAND_MSG_MSG,	//24
	WAIT_FOR_SENDING_SHOOT_COMMAND_MSG,		//25
	SETTINGS_MSG_RECEIVED					//26						
};

enum parser_status{
	IDLE,			//0
	GOT_O,			//1
	GOT_OK,			//2
	GOT_E,			//3
	GOT_ER,			//4
	GOT_ERR,		//5
	GOT_ERRO,		//6
	GOT_ERROR,		//7
	GOT_r,			//8
	GOT_re,			//9
	GOT_res,		//10
	GOT_resu,		//11
	GOT_resul,		//12
	GOT_result,		//13
	GOT_slash,		//14
	GOT_slashp,		//15
	GOT_slashpi,	//16
	GOT_slashpic,	//17
	GOT_slashpict,	//18
	GOT_C,			//19
	GOT_CL,			//20
	GOT_CLO,		//21
	GOT_CLOS,		//22
	GOT_CLOSE,		//23
	GOT_J,			//24
	GOT_JP,			//25
	GOT_JPG,		//26
	GOT_A, 			//27
	GOT_AL, 		//28
	GOT_ALR, 		//29
	GOT_ALRE, 		//30
	GOT_ALREA, 		//31
	GOT_ALREAD, 	//32
	GOT_ALREADY, 	//33
	GOT_F,			//34
	GOT_FA,			//35
	GOT_FAI,		//36
	GOT_FAIL,		//37
	GOT_4,
	GOT_3, 
	GOT_D,
	GOT_DS,
	GOT_DSC
};

enum cam_order{
	CAMERA_IDLE,
	CAMERA_SHOOT,
	SET_CAMERA_PARAMETERS
};

enum camera_model {
	SONY_A7R,
	SONY_A6000,
	SONY_QX1
};

extern enum cam_state sony_a7r_state;
extern enum parser_status camera_parser_status;
extern enum cam_order camera_order;
extern enum camera_model cam_model;

extern int result_counter;
extern int name_counter;
extern int delay_counter;
extern uint8_t tcp_connection_errors;
extern int udp_connection_errors;
extern int time_counter;
extern int got_status_counter;
extern int last_recieved_setting;
extern uint8_t last_set_setting;
extern uint8_t connection_counter;
extern int shutter_speed_response_counter;
extern int iso_response_counter;
extern int fnumber_response_counter;
extern uint16_t sival;
extern bool result_read;
extern bool mode_set;
extern bool tcp_connected;
extern bool word_processing;
extern bool already_connected;
extern bool delay_mode;
extern bool got_status;
extern bool chars_recieved;
extern bool image_name_started;
extern bool image_name_finished;
extern bool settings_set;
extern bool enable_camera;
extern char image_name[100];
extern char final_name[100];
extern char shutter_speed_response[1000];
extern char iso_response[1000];
extern char fnumber_response[1000];
extern float curr_time;
extern float start_time;
extern float sfval;
extern float ssvalue;
extern FILE *image_names;

extern void sony_a7r_handler_setup(void);
extern void sony_a7r_handler_periodic(void);

extern void clear_image_name(void);

static inline void on_settings_msg_receive(void){

	cam_model = DL_CAMERA_SETTINGS_GS_camera_model(dl_buffer);
	ssvalue = DL_CAMERA_SETTINGS_GS_shutter_speed(dl_buffer);
	sival = DL_CAMERA_SETTINGS_GS_iso(dl_buffer);
	sfval = DL_CAMERA_SETTINGS_GS_fnumber(dl_buffer);
	enable_camera = DL_CAMERA_SETTINGS_GS_enable(dl_buffer);

	if(enable_camera){
		camera_order = SET_CAMERA_PARAMETERS;
		clear_image_name();
		connection_counter = 0;

		if(cam_model == SONY_QX1){
			mode_set = true;
		}
	}
	send_mission_ack(CAMERA_SETTINGS_ACK);
}

extern void wifi_response_parser(char);

extern void esp_01_jap(void);
extern void esp_01_cipmux(void);
extern void esp_01_cipstart(void);
extern void esp_01_cipsend(void);
extern void esp_01_msearch(void);
extern void esp_01_cipclose(void);
extern void esp_01_cipstart_tcp(void);
extern void esp_01_cipsend_tcp(void);
extern void esp_01_setmode(void);
extern void esp_01_cipsend_shoot(void);
extern void esp_01_cipsend_setting_sf(void);
extern void esp_01_cipsend_setting_si(void);
extern void esp_01_cipsend_setting_ss(void);
extern void check_esp_01_connection_status(void);
extern void sony_a7r_shoot(void);
extern void endline(void);
extern void shoot(void);
extern void send_settings(void);
extern void get_available_shutter_speed(void);
extern void get_current_shutter_speed(void);
extern void get_available_isos(void);
extern void get_current_iso(void);
extern void get_available_fnumbers(void);
extern void get_current_fnumber(void);
extern void set_fnumber(float);
extern void set_iso(uint16_t);
extern void set_shutter_speed(float);
extern void idle(void);

#endif