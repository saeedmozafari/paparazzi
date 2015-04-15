/*
############################## AUTMAV ##############################
# PROJECT: AUTMAV MODIFIED PAPARAZZI				              			   #
# DEVELOPED BY: ALI JAMEEI							                   			   #
# Copyright (C) 2014-2015								                     		   #	
####################################################################
*/


/* DESCRIPTION: 
	The I2C basic driver for px4 flow module from pixhawk project. 
*/ 

#include "modules/autmav/px4flow_i2c.h"
#include "led.h" 
#include "mcu_periph/sys_time.h"


struct  px4flow_I2c px4flow;

void px4flow_i2c_init( void )
{
  /* set i2c_peripheral */
  px4flow.i2c_p = &PX4FLOW_I2C_DEV;
  /* slave address */
  px4flow.i2c_trans.slave_addr = PX4FLOW_I2C_ADDR;
  /* set inital status: Success or Done */
  px4flow.i2c_trans.status = I2CTransDone;
  // initial data not avilable. 
  px4flow.data_available = FALSE;
}

void px4flow_i2c_event( void )
{ if (px4flow.i2c_trans.status == I2CTransDone) {
    //reading form the sensor 
    px4flow.i2c_trans.buf[0] = PX4FLOW_FRAME_READ_CMD;
    i2c_transceive(px4flow.i2c_p, &(px4flow.i2c_trans),px4flow.i2c_trans.slave_addr,1,1);
  }
  else if (px4flow.i2c_trans.status ==  I2CTransFailed) {
    px4flow.i2c_trans.status = I2CTransDone;
  }
  else if (px4flow.i2c_trans.status == I2CTransSuccess) {
  	//acquire the values  
  	LED_ON(3);
    px4flow.data.frame_count =  ((int16_t)((px4flow.i2c_trans.buf[1]<<8) | px4flow.i2c_trans.buf[2]));
    px4flow.data.pixel_flow_x_sum = ((int16_t)((px4flow.i2c_trans.buf[1]<<8) | px4flow.i2c_trans.buf[2]));;
    px4flow.data.pixel_flow_y_sum = ((int16_t)((px4flow.i2c_trans.buf[1]<<8) | px4flow.i2c_trans.buf[2]));;
    px4flow.data.flow_comp_m_x = ((int16_t)((px4flow.i2c_trans.buf[1]<<8) | px4flow.i2c_trans.buf[2]));;
    px4flow.data.flow_comp_m_y = ((int16_t)((px4flow.i2c_trans.buf[1]<<8) | px4flow.i2c_trans.buf[2]));;
    px4flow.data.qual = ((int16_t)((px4flow.i2c_trans.buf[1]<<8) | px4flow.i2c_trans.buf[2]));;
    px4flow.data.gyro_x_rate = ((int16_t)((px4flow.i2c_trans.buf[1]<<8) | px4flow.i2c_trans.buf[2]));;
    px4flow.data.gyro_y_rate = ((int16_t)((px4flow.i2c_trans.buf[1]<<8) | px4flow.i2c_trans.buf[2]));;
    px4flow.data.gyro_z_rate = ((int16_t)((px4flow.i2c_trans.buf[1]<<8) | px4flow.i2c_trans.buf[2]));;
    px4flow.data.gyro_range =  ((uint8_t)(px4flow.i2c_trans.buf[19])) ;
    px4flow.data.sonar_timestamp =  ((uint8_t)(px4flow.i2c_trans.buf[20]));
    px4flow.data.ground_distance = ((int16_t)((px4flow.i2c_trans.buf[1]<<8) | px4flow.i2c_trans.buf[2]));;
    px4flow.data_available = TRUE;
  }
}
