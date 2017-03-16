#ifndef SONY_A7R_HANDLER_H
#define SONY_A7R_HANDLER_H

#include "std.h"
#include "paparazzi.h"
#include <stdio.h>
#include "std.h"

enum cam_state{
	
	SEND_AT,						//0
	WAIT_FOR_READY,					//1
	ESP_CONNECTED_TO_APPOGEE,		//2
	ESP_CONNECTING_TO_CAM_AP,		//3
	ESP_CONNECTED_TO_CAM_AP,		//4
	SETTING_CIPMUX,					//5
	CIPMUX_SET,						//6
	CONNECTING_TO_CAM_CLIENT_UDP,	//7
	CONNECTED_TO_CAM_CLIENT_UDP,	//8
	SENDING_SEND_COMMAND_UDP, 		//9
	READY_TO_SEND_DISCOVERY_MSG,	//10
	SENDING_DISCOVERY_MSG,			//11
	CAM_DISCOVERED,					//12
	CLOSING_UDP_CONNECTION,			//13
	UDP_CONNECTION_CLOSED,			//14
	CONNECTING_TO_CAM_CLIENT_TCP,	//15
	CONNECTED_TO_CAM_CLIENT_TCP,	//16
	SENDING_SEND_COMMAND_TCP,		//17
	READY_TO_SEND_MODE_MSG,			//18
	SENDING_MODE_MSG,				//19
	MODE_MSG_SENT,					//20
						//21
	SENDING_SEND_SHOOT,				//22
	READY_TO_SEND_SHOOT_COMMAND,	//23
	SENDING_SHOOT_COMMAND,			//24
	SHOOT_COMMAND_SENT,				//25
	IMAGE_NAME_RECIEVED,			//26
	CAM_IDLE_MODE,
	CAMERA_WAIT_FOR_STATUS_TO_BE_SENT,
	RESET_CONNECTION,
	CAMERA_WAIT_FOR_CONNECTION_TO_BE_RESET,
	CAMERA_SHOOT_MODE,
	CAMERA_WAIT_FOR_SEND_TO_BE_OK,
	CAMERA_ACT_SHOOT,
	CAMERA_SHOOT_FOR_SEND_TO_BE_FINISHED,
	CAMERA_CLOSE_CONNECTION,
	CAMERA_WAIT_FOR_CONNECTION_TO_BE_CLOSED
						
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
	GOT_3
};

enum cam_order{
	CAMERA_IDLE,
	CAMERA_SHOOT
};

extern enum cam_state sony_a7r_state;
extern enum parser_status camera_parser_status;
extern enum cam_order camera_order;

extern int result_counter;
extern int name_counter;
extern int delay_counter;
extern int tcp_connection_errors;
extern int time_counter;
extern int got_status_counter;
extern bool result_read;
extern bool mode_set;
extern bool tcp_connected;
extern bool word_processing;
extern bool already_connected;
extern bool delay_mode;
extern bool got_status;
extern bool chars_recieved;
extern char image_name[100];
extern float curr_time;
extern float start_time;
extern FILE *image_names;

extern void sony_a7r_handler_setup(void);
extern void sony_a7r_handler_periodic(void);

extern void wifi_response_parser(char);
extern void clear_image_name(void);

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
extern void check_esp_01_connection_status(void);
extern void sony_a7r_shoot(void);
extern void endline(void);
extern void shoot(void);
extern void idle(void);

#endif