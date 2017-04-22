#include "sony_camera_handler.h"
#include "RTK_receive.h"
#include "pprzlink/pprzlink_device.h"
#include "mcu_periph/uart.h"
#include "subsystems/datalink/datalink.h" 
#include "generated/airframe.h"
#include "subsystems/datalink/telemetry.h"
#include "led.h"
#include "mcu_periph/uart.h"
#include "mcu_periph/sys_time.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>
#include <math.h>
#include <string.h>

struct link_device *wifi_command;

#ifndef ESP_01_BAUD
#define ESP_01_BAUD B115200
#endif

#ifndef ESP_01_UART_PORT 
#define ESP_01_UART_PORT uart6
#endif

#ifndef ISO_SPEED_RATE
#define ISO_SPEED_RATE 400
#endif

#ifndef SHUTTER_SPEED
#define SHUTTER_SPEED 0.008
#endif

#ifndef FNUMBER
#define FNUMBER 2.8
#endif

uint8_t last_set_setting;
uint16_t time_counter;
uint16_t sival = 160;
uint16_t delay_counter = 0;
uint16_t image_name_counter = 0;
float sfval = 2.8;
float ssvalue = 0.000625;
char image_name[100];
bool image_name_read = false;
bool mode_set = false;
bool word_processing = false;
bool dsc_received = false;
bool jpg_received = false;
bool result_received = false;

enum camera_state cam_state;
enum camera_model cam_model;
enum parser cam_parser;

static void send_camera_state(struct transport_tx *trans, struct link_device *dev)
 {
   	uint8_t cam_state_for_debug = cam_state;
   	uint8_t zero = 0;
   	//uint8_t parser_state_for_debug = cam_parser;
	pprz_msg_send_SONY_CAMERA_STATUS(trans, dev, AC_ID,
                         &cam_state_for_debug,
                         &zero);
}

void sony_camera_handler_setup(void){
	wifi_command = &((ESP_01_UART_PORT).device);
	uart_periph_set_bits_stop_parity(&ESP_01_UART_PORT, UBITS_8, USTOP_1, UPARITY_NO);
	uart_periph_set_baudrate(&ESP_01_UART_PORT, ESP_01_BAUD);
	register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_SONY_CAMERA_STATUS, send_camera_state);
	clear_image_name();
	cam_model = SONY_A6000;

	if(cam_model == SONY_QX1){
		mode_set = true;
	}
	else{
		mode_set = false;
	}
}

void set_rec_mode(void){
	char prepare_to_shoot_msg[264] = "POST /sony/camera HTTP/1.1\r\nContent-Type: application/json; charset=utf-8\r\nAccept: Accept-application/json\r\nHost: 192.168.122.1:8080\r\nContent-Length: 60\r\nExpect: 100-continue\r\nConnection: Keep-Alive\r\n\r\n{\"method\":\"startRecMode\",\"params\":[],\"id\":1,\"version\":\"1.0\"}\r\n";
	for(int i=0; i<264; i++){
		wifi_command->put_byte(wifi_command->periph, 0, (uint8_t)prepare_to_shoot_msg[i]);
	}
}

void set_postview_name(void){
	char set_postview_msg[282] = "POST /sony/camera HTTP/1.1\r\nContent-Type: application/json; charset=utf-8\r\nAccept: Accept-application/json\r\nHost: 192.168.122.1:8080\r\nContent-Length: 78\r\nExpect: 100-continue\r\nConnection: Keep-Alive\r\n\r\n{\"method\":\"setPostviewImageSize\",\"params\":[\"Original\"],\"id\":1,\"version\":\"1.0\"}\r\n";
	for(int i=0; i<282; i++){
		wifi_command->put_byte(wifi_command->periph, 0, (uint8_t)set_postview_msg[i]);
	}
}

void send_shoot_command(void){
	char shoot_command_msg[266] = "POST /sony/camera HTTP/1.1\r\nContent-Type: application/json; charset=utf-8\r\nAccept: Accept-application/json\r\nHost: 192.168.122.1:8080\r\nContent-Length: 62\r\nExpect: 100-continue\r\nConnection: Keep-Alive\r\n\r\n{\"method\":\"actTakePicture\",\"params\":[],\"id\":1,\"version\":\"1.0\"}\r\n";
	for(int i=0; i<266; i++){
		wifi_command->put_byte(wifi_command->periph, 0, (uint8_t)shoot_command_msg[i]);
	}
}

void get_available_shutter_speed(void){
	char available_shutter_speeds_msg[276] = "POST /sony/camera HTTP/1.1\r\nContent-Type: application/json; charset=utf-8\r\nAccept: Accept-application/json\r\nHost: 192.168.122.1:8080\r\nContent-Length: 72\r\nExpect: 100-continue\r\nConnection: Keep-Alive\r\n\r\n{\"method\":\"getAvailableShutterSpeed\",\"params\":[],\"id\":1,\"version\":\"1.0\"}\r\n";
	for(int i=0; i<276; i++){
		wifi_command->put_byte(wifi_command->periph, 0, (uint8_t)available_shutter_speeds_msg[i]);
	}
}

void set_shutter_speed(float val1){
	char value[9];

	if(val1 == 30 ){
		char temp_value[9] = "30.0";
		strncpy(value, temp_value, 9);
	}
	if(val1 == 25){
		char temp_value[9] = "25.0";
		strncpy(value, temp_value, 9);
	}
	if(val1 == 20){
		char temp_value[9] = "20.0";
		strncpy(value, temp_value, 9);
	}
	if(val1 == 15){
		char temp_value[9] = "15.0";
		strncpy(value, temp_value, 9);
	}
	if(val1 == 13){
		char temp_value[9] = "13.0";
		strncpy(value, temp_value, 9);
	}
	if(val1 == 10){
		char temp_value[9] = "10.0";
		strncpy(value, temp_value, 9);
	}
	if(val1 == 8){
		char temp_value[9] = "8.0";
		strncpy(value, temp_value, 9);
	}
	if(val1 == 6){
		char temp_value[9] = "6.0";
		strncpy(value, temp_value, 9);
	}
	if(val1 == 5){
		char temp_value[9] = "5.0";
		strncpy(value, temp_value, 9);
	}
	if(val1 == 4){
		char temp_value[9] = "4.0";
		strncpy(value, temp_value, 9);
	}
	if(val1 == 3.2){
		char temp_value[9] = "3.2";
		strncpy(value, temp_value, 9);
	}
	if(val1 == 2.5){
		char temp_value[9] = "2.5";
		strncpy(value, temp_value, 9);
	}
	if(val1 == 2){
		char temp_value[9] = "2.0";
		strncpy(value, temp_value, 9);
	}
	if(val1 == 1.6){
		char temp_value[9] = "1.6";
		strncpy(value, temp_value, 9);
	}
	if(val1 == 1.3){
		char temp_value[9] = "1.3";
		strncpy(value, temp_value, 9);
	}
	if(val1 == 1){
		char temp_value[9] = "1.0";
		strncpy(value, temp_value, 9);
	}
	if(val1 == 0.8){
		char temp_value[9] = "0.8";
		strncpy(value, temp_value, 9);
	}
	if(val1 == 0.6){
		char temp_value[9] = "0.6";
		strncpy(value, temp_value, 9);
	}
	if(val1 == 0.5){
		char temp_value[9] = "0.5";
		strncpy(value, temp_value, 9);
	}
	if(val1 == 0.4){
		char temp_value[9] = "0.4";
		strncpy(value, temp_value, 9);
	}
	if(val1 == 0.25){
		char temp_value[9] = "0.25";
		strncpy(value, temp_value, 9);
	}
	if(val1 == 0.2){
		char temp_value[9] = "0.2";
		strncpy(value, temp_value, 9);
	}
	if(val1 == 0.125){
		char temp_value[9] = "0.125";
		strncpy(value, temp_value, 9);
	}
	if(val1 == 0.1){
		char temp_value[9] = "0.1";
		strncpy(value, temp_value, 9);
	}
	if(val1 == 0.05){
		char temp_value[9] = "0.05";
		strncpy(value, temp_value, 9);
	}
	if(val1 == 0.04){
		char temp_value[9] = "0.04";
		strncpy(value, temp_value, 9);		
	}
	if(val1 == 0.025){
		char temp_value[9] = "0.025";
		strncpy(value, temp_value, 9);		
	}
	if(val1 == 0.02){
		char temp_value[9] = "0.02";
		strncpy(value, temp_value, 9);		
	}
	if(val1 == 0.0125){
		char temp_value[9] = "0.0125";
		strncpy(value, temp_value, 9);		
	}
	if(val1 == 0.01){
		char temp_value[9] = "0.01";
		strncpy(value, temp_value, 9);		
	}
	if(val1 == 0.008){
		char temp_value[9] = "0.008";
		strncpy(value, temp_value, 9);		
	}
	if(val1 == 0.005){
		char temp_value[9] = "0.005";
		strncpy(value, temp_value, 9);		
	}
	if(val1 == 0.004){
		char temp_value[9] = "0.004";
		strncpy(value, temp_value, 9);		
	}
	if(val1 == 0.003125){
		char temp_value[9] = "0.003125";
		strncpy(value, temp_value, 9);		
	}
	if(val1 == 0.0025){
		char temp_value[9] = "0.0025";
		strncpy(value, temp_value, 9);		
	}
	if(val1 == 0.002){
		char temp_value[9] = "0.002";
		strncpy(value, temp_value, 9);		
	}
	if(val1 == 0.0015625){
		char temp_value[9] = "0.0015625";
		strncpy(value, temp_value, 9);		
	}
	if(val1 == 0.00125){
		char temp_value[9] = "0.00125";
		strncpy(value, temp_value, 9);		
	}
	if(val1 == 0.001){
		char temp_value[9] = "0.001";
		strncpy(value, temp_value, 9);		
	}
	if(val1 == 0.0008){
		char temp_value[9] = "0.0008";
		strncpy(value, temp_value, 9);	
	}
	if(val1 == 0.000625){
		char temp_value[9] = "0.000625";
		strncpy(value, temp_value, 9);		
	}
	if(val1 == 0.0005){
		char temp_value[9] = "0.0005";
		strncpy(value, temp_value, 9);		
	}
	if(val1 == 0.0004){
		char temp_value[9] = "0.0004";
		strncpy(value, temp_value, 9);		
	}
	if(val1 == 0.0003125){
		char temp_value[9] = "0.0003125";
		strncpy(value, temp_value, 9);		
	}
	if(val1 == 0.00025){
		char temp_value[9] = "0.00025";
		strncpy(value, temp_value, 9);	
	}

	char set_shutter_speeds_msg[278] = "POST /sony/camera HTTP/1.1\r\nContent-Type: application/json; charset=utf-8\r\nAccept: Accept-application/json\r\nHost: 192.168.122.1:8080\r\nContent-Length: 74\r\nExpect: 100-continue\r\nConnection: Keep-Alive\r\n\r\n{\"method\":\"setShutterSpeed\",\"params\":[\"         \"],\"id\":1,\"version\":\"1.0\"}\r\n";
	
	for(int i=0; i<9; i++){
		set_shutter_speeds_msg[i + 241] = value[i];
	}

	for(int i=0; i<278; i++){
		wifi_command->put_byte(wifi_command->periph, 0, (uint8_t)set_shutter_speeds_msg[i]);
	}
}

void get_available_isos(void){
	char available_isos_msg[276] = "POST /sony/camera HTTP/1.1\r\nContent-Type: application/json; charset=utf-8\r\nAccept: Accept-application/json\r\nHost: 192.168.122.1:8080\r\nContent-Length: 72\r\nExpect: 100-continue\r\nConnection: Keep-Alive\r\n\r\n{\"method\":\"getAvailableIsoSpeedRate\",\"params\":[],\"id\":1,\"version\":\"1.0\"}\r\n";
	for(int i=0; i<276; i++){
		wifi_command->put_byte(wifi_command->periph, 0, (uint8_t)available_isos_msg[i]);
	}
}

void set_iso(uint16_t val){
	char value[5];

	if(val == 0){
		char temp_value[4] = "AUTO";
		strncpy(value, temp_value, 4);		
	}
	if(val == 100){
		char temp_value[3] = "100";
		strncpy(value, temp_value, 3);		
	}
	if(val == 125){
		char temp_value[3] = "125";
		strncpy(value, temp_value, 3);		
	}
	if(val == 160){
		char temp_value[3] = "160";
		strncpy(value, temp_value, 3);		
	}
	if(val == 200){
		char temp_value[3] = "200";
		strncpy(value, temp_value, 3);		
	}
	if(val == 250){
		char temp_value[3] = "250";
		strncpy(value, temp_value, 3);		
	}
	if(val == 320){
		char temp_value[3] = "320";
		strncpy(value, temp_value, 3);		
	}
	if(val == 400){
		char temp_value[3] = "400";
		strncpy(value, temp_value, 3);		
	}
	if(val == 500){
		char temp_value[3] = "500";
		strncpy(value, temp_value, 3);		
	}
	if(val == 640){
		char temp_value[3] = "640";
		strncpy(value, temp_value, 3);		
	}
	if(val == 800){
		char temp_value[3] = "800";
		strncpy(value, temp_value, 3);		
	}
	if(val == 1000){
		char temp_value[4] = "1000";
		strncpy(value, temp_value, 4);		
	}
	if(val == 1250){
		char temp_value[4] = "1250";
		strncpy(value, temp_value, 4);		
	}
	if(val == 1600){
		char temp_value[4] = "1600";
		strncpy(value, temp_value, 4);		
	}
	if(val == 2000){
		char temp_value[4] = "2000";
		strncpy(value, temp_value, 4);		
	}
	if(val == 2500){
		char temp_value[4] = "2500";
		strncpy(value, temp_value, 4);		
	}
	if(val == 3200){
		char temp_value[4] = "3200";
		strncpy(value, temp_value, 4);		
	}
	if(val == 4000){
		char temp_value[4] = "4000";
		strncpy(value, temp_value, 4);		
	}
	if(val == 5000){
		char temp_value[4] = "5000";
		strncpy(value, temp_value, 4);		
	}
	if(val == 6400){
		char temp_value[4] = "6400";
		strncpy(value, temp_value, 4);		
	}
	if(val == 8000){
		char temp_value[4] = "8000";
		strncpy(value, temp_value, 4);		
	}
	if(val == 10000){
		char temp_value[5] = "10000";
		strncpy(value, temp_value, 5);	
	}
	if(val == 12800){
		char temp_value[5] = "12800";
		strncpy(value, temp_value, 5);	
	}
	if(val == 16000){
		char temp_value[5] = "16000";
		strncpy(value, temp_value, 5);	
	}

	char set_iso_msg[274] = "POST /sony/camera HTTP/1.1\r\nContent-Type: application/json; charset=utf-8\r\nAccept: Accept-application/json\r\nHost: 192.168.122.1:8080\r\nContent-Length: 70\r\nExpect: 100-continue\r\nConnection: Keep-Alive\r\n\r\n{\"method\":\"setIsoSpeedRate\",\"params\":[\"     \"],\"id\":1,\"version\":\"1.0\"}\r\n";
	for(int i=0; i<5; i++){
		set_iso_msg[241 + i] = value[i];
	}

	for(int i=0; i<274; i++){
		wifi_command->put_byte(wifi_command->periph, 0, (uint8_t)set_iso_msg[i]);
	}
}

void get_available_fnumbers(void){
	char available_fnumbers_msg[271] = "POST /sony/camera HTTP/1.1\r\nContent-Type: application/json; charset=utf-8\r\nAccept: Accept-application/json\r\nHost: 192.168.122.1:8080\r\nContent-Length: 67\r\nExpect: 100-continue\r\nConnection: Keep-Alive\r\n\r\n{\"method\":\"getAvailableFNumber\",\"params\":[],\"id\":1,\"version\":\"1.0\"}\r\n";
	for(int i=0; i<271; i++){
		wifi_command->put_byte(wifi_command->periph, 0, (uint8_t)available_fnumbers_msg[i]);
	}
}

void set_fnumber(float val){
	char value[3];

	if(val == 2.8){
		char temp_value[3] = "2.8";
		strncpy(value, temp_value, 3);		 
	}
	if(val == 3.5){
		char temp_value[3] = "3.5";
		strncpy(value, temp_value, 3);		 
	}
	if(val == 4.0){
		char temp_value[3] = "4.0";
		strncpy(value, temp_value, 3);		 
	}
	if(val == 4.5){
		char temp_value[3] = "4.5";
		strncpy(value, temp_value, 3);		 
	}
	if(val == 5.0){
		char temp_value[3] = "5.0";
		strncpy(value, temp_value, 3);		 
	}
	if(val == 5.6){
		char temp_value[3] = "5.6";
		strncpy(value, temp_value, 3);		 
	}
	if(val == 6.3){
		char temp_value[3] = "6.3";
		strncpy(value, temp_value, 3);		 
	}
	if(val == 7.1){
		char temp_value[3] = "7.1";
		strncpy(value, temp_value, 3);		 
	}
	if(val == 8.0){
		char temp_value[3] = "8.0";
		strncpy(value, temp_value, 3);		 
	}
	if(val == 9.0){
		char temp_value[3] = "9.0";
		strncpy(value, temp_value, 3);		 
	}
	if(val == 10){
		char temp_value[2] = "10";
		strncpy(value, temp_value, 2);		 
	}
	if(val == 11){
		char temp_value[2] = "11";
		strncpy(value, temp_value, 2);		 
	}
	if(val == 13){
		char temp_value[2] = "13";
		strncpy(value, temp_value, 2);		 
	}
	if(val == 14){
		char temp_value[2] = "14";
		strncpy(value, temp_value, 2);		 
	}
	if(val == 16){
		char temp_value[2] = "16";
		strncpy(value, temp_value, 2);		 
	}
	if(val == 18){
		char temp_value[2] = "18";
		strncpy(value, temp_value, 2);		 
	}
	if(val == 20){
		char temp_value[2] = "20";
		strncpy(value, temp_value, 2);		 
	}
	if(val == 22){
		char temp_value[2] = "22";
		strncpy(value, temp_value, 2);		 
	}

	char set_fnumber_msg[267] = "POST /sony/camera HTTP/1.1\r\nContent-Type: application/json; charset=utf-8\r\nAccept: Accept-application/json\r\nHost: 192.168.122.1:8080\r\nContent-Length: 63\r\nExpect: 100-continue\r\nConnection: Keep-Alive\r\n\r\n{\"method\":\"setFNumber\",\"params\":[\"   \"],\"id\":1,\"version\":\"1.0\"}\r\n";
	for(int i=0; i<3; i++){
		set_fnumber_msg[236 + i] = value[i];
	}

	for(int i=0; i<267; i++){
		wifi_command->put_byte(wifi_command->periph, 0, (uint8_t)set_fnumber_msg[i]);
	}
}

void sony_camera_handler_periodic(void){
	switch(cam_state){
		case SET_REC_MODE:
			if(delay_counter < 20000){
				delay_counter++;
			}
			else{
				set_rec_mode();
				time_counter = 0;
				cam_state = WAIT_FOR_REC_MODE_TO_BE_SET;
				result_received = false;
			}
		break;
		case WAIT_FOR_REC_MODE_TO_BE_SET:
			if(result_received){
				cam_state = SET_POSTVIEW_NAME;
			}
			else{
				if(time_counter < 500){
					time_counter++;
				}
				else{
					set_rec_mode();
					time_counter = 0;
					result_received = false;
				}
			}
		break;
		case SET_POSTVIEW_NAME:
			set_postview_name();
			time_counter = 0;
			cam_state = WAIT_FOR_POSTVIEW_TO_BE_SET;
			result_received = false;
		break;
		case WAIT_FOR_POSTVIEW_TO_BE_SET:
			if(result_received){
				cam_state = IDLE_MODE;
			}
			else{
				if(time_counter < 500){
					time_counter++;
				}
				else{
					set_postview_name();
					time_counter = 0;
					result_received = false;
				}
			}
		break;
		case IDLE_MODE:
			
		break;
		case SEND_SHOOT:
			send_shoot_command();
			cam_state = WAIT_FOR_IMAGE_NAME;
			time_counter = 0;
			image_name_read = false;
		break;
		case WAIT_FOR_IMAGE_NAME:
			if(image_name_read /*|| time_counter > 10*/){
				cam_state = IDLE_MODE;
			}
			else{
				time_counter++;
			}
		break;
		case SETTING_SHUTTER_SPEED:
			set_shutter_speed(ssvalue);
			time_counter = 0;
			cam_state = WAIT_FOR_SHUTTER_SPEED;
			result_received = false;
		break;
		case WAIT_FOR_SHUTTER_SPEED:
			if(result_received){
				cam_state = SETTING_ISO;
			}
			else{
				if(time_counter < 500){
					time_counter++;
				}
				else{
					cam_state = SETTING_ISO;
					uint8_t one = 1;
					last_set_setting = 0;
					DOWNLINK_SEND_CAMERA_SETTINGS(DefaultChannel, DefaultDevice,
															&ssvalue,
															&sival,
															&sfval,
															&last_set_setting,
															&one);
				}
			}
		break;
		case SETTING_ISO:
			set_iso(sival);
			time_counter = 0;
			cam_state = WAIT_FOR_ISO;
			result_received = false;
		break;
		case WAIT_FOR_ISO:
			if(result_received){
				cam_state = SETTING_FNUMBER;
			}
			else{
				if(time_counter < 500){
					time_counter++;
				}
				else{
					cam_state = SETTING_FNUMBER;
					uint8_t one = 1;
					last_set_setting = 1;
					DOWNLINK_SEND_CAMERA_SETTINGS(DefaultChannel, DefaultDevice,
															&ssvalue,
															&sival,
															&sfval,
															&last_set_setting,
															&one);
				}
			}
		break;
		case SETTING_FNUMBER:
			set_fnumber(sfval);
			time_counter = 0;
			cam_state = WAIT_FOR_FNUMBER;
			result_received = false;
		break;
		case WAIT_FOR_FNUMBER:
			if(result_received){
				cam_state = IDLE_MODE;
				uint8_t one = 1;
				last_set_setting = 3;
				DOWNLINK_SEND_CAMERA_SETTINGS(DefaultChannel, DefaultDevice,
															&ssvalue,
															&sival,
															&sfval,
															&last_set_setting,
															&one);
			}
			else{
				if(time_counter < 500){
					time_counter++;
				}
				else{
					cam_state = IDLE_MODE;
					uint8_t one = 1;
					last_set_setting = 2;
					DOWNLINK_SEND_CAMERA_SETTINGS(DefaultChannel, DefaultDevice,
															&ssvalue,
															&sival,
															&sfval,
															&last_set_setting,
															&one);
				}
			}
		break;
	}
}

void clear_image_name(void){
	for(int i = 0; i < 100; i++){
		image_name[i] = '&';
	}
}

void shoot(void){
	if(cam_state == IDLE_MODE){
		cam_state = SEND_SHOOT;
	}
}

void send_settings(void){
}

void read_image_name(void){
	char temp;

	while(wifi_command->char_available(wifi_command->periph)){
		
		temp = (char)wifi_command->get_byte(wifi_command->periph);
		wifi_response_parser(temp);
		if(cam_parser == GOT_result){
			result_received = true;
		}
		if(dsc_received && !jpg_received){
			image_name[image_name_counter] = temp;
			image_name_counter++;
		}
		if(cam_parser == GOT_DSC){
			clear_image_name();
			image_name[0] = 'D';
			image_name[1] = 'S';
			image_name[2] = 'C';
			image_name_counter = 3;
			dsc_received = true;
			jpg_received = false;
		}
		if(cam_parser == GOT_JPG){
			if(dsc_received){
				tag_image_log();
				image_name_read = true;
			}
			jpg_received = true;
			dsc_received = false;		
		}
	}

}

void wifi_response_parser(char curr_byte){

	lable:

	if(!word_processing && curr_byte == 'e'){
		cam_parser = GOT_e;
		word_processing = true;
		return;
	}
	if(cam_parser == GOT_e && curr_byte == 'r'){
		cam_parser = GOT_er;
		return;
	}
	else if(cam_parser == GOT_e && curr_byte != 'r'){
		cam_parser = IDLE;
		word_processing = false;
		goto lable;
	}
	if(cam_parser == GOT_er && curr_byte == 'r'){
		cam_parser = GOT_err;
		return;
	}
	else if(cam_parser == GOT_er && curr_byte != 'r'){
		cam_parser = IDLE;
		word_processing = false;
		goto lable;
	}
	if(cam_parser == GOT_err && curr_byte == 'o'){
		cam_parser = GOT_erro;
		return;
	}
	else if(cam_parser == GOT_err && curr_byte != 'o'){
		cam_parser = IDLE;
		word_processing = false;
		goto lable;
	}
	if(cam_parser == GOT_erro && curr_byte == 'r'){
		cam_parser = GOT_error;
		word_processing = false;
		return;
	}
	else if(cam_parser == GOT_erro && curr_byte != 'r'){
		cam_parser = IDLE;
		word_processing = false;
		goto lable;
	}

	if(!word_processing && curr_byte == 'r'){
		cam_parser = GOT_r;
		word_processing = true;
		return;
	}
	if(cam_parser == GOT_r && curr_byte == 'e'){
		cam_parser = GOT_re;
		return;
	}
	else if(cam_parser == GOT_r && curr_byte != 'e'){
		cam_parser = IDLE;
		word_processing = false;
		goto lable;
	}
	if(cam_parser == GOT_re && curr_byte == 's'){
		cam_parser = GOT_res;
		return;
	}
	else if(cam_parser == GOT_re && curr_byte != 's'){
		cam_parser = IDLE;
		word_processing = false;
		goto lable;
	}
	if(cam_parser == GOT_res && curr_byte == 'u'){
		cam_parser = GOT_resu;
		return;
	}
	else if(cam_parser == GOT_res && curr_byte != 'u'){
		cam_parser = IDLE;
		word_processing = false;
		goto lable;
	}
	if(cam_parser == GOT_resu && curr_byte == 'l'){
		cam_parser = GOT_resul;
		return;
	}
	else if(cam_parser == GOT_resu && curr_byte != 'l'){
		cam_parser = IDLE;
		word_processing = false;
		goto lable;
	}
	if(cam_parser == GOT_resul && curr_byte == 't'){
		cam_parser = GOT_result;
		word_processing = false;
		return;
	}
	else if(cam_parser == GOT_resul && curr_byte != 't'){
		cam_parser = IDLE;
		word_processing = false;
		goto lable;
	}

	if(!word_processing && curr_byte == 'D'){
		cam_parser = GOT_D;
		word_processing = true;
		return;
	}
	if(cam_parser == GOT_D && curr_byte == 'S'){
		cam_parser = GOT_DS;
		return;
	}
	else if(cam_parser == GOT_D && curr_byte != 'S'){
		cam_parser = IDLE;
		word_processing = false;
		goto lable;
	}
	if(cam_parser == GOT_DS && curr_byte == 'C'){
		cam_parser = GOT_DSC;
		word_processing = false;
		return;
	}
	else if(cam_parser == GOT_DS && curr_byte != 'C'){
		cam_parser = IDLE;
		word_processing = false;
		goto lable;
	}

	if(!word_processing && curr_byte == 'J'){
		cam_parser = GOT_J;
		word_processing = true;
		return;
	}
	if(cam_parser == GOT_J && curr_byte == 'P'){
		cam_parser = GOT_JP;
		return;
	}
	else if(cam_parser == GOT_J && curr_byte != 'P'){
		cam_parser = IDLE;
		word_processing = false;
		goto lable;
	}
	if(cam_parser == GOT_JP && curr_byte == 'G'){
		cam_parser = GOT_JPG;
		word_processing = false;
		return;
	}
	else if(cam_parser == GOT_JP && curr_byte != 'G'){
		cam_parser = IDLE;
		word_processing = false;
		goto lable;
	}

	cam_parser = IDLE;
}