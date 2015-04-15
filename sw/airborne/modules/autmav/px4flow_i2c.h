/*
############################## AUTMAV ##############################
# PROJECT: AUTMAV MODIFIED PAPARAZZI							   #
# DEVELOPED BY: ALI JAMEEI										   #
# Copyright (C) 2014-2015										   #	
####################################################################
*/


/* DESCRIPTION: 
	The I2C basic driver for px4 flow module from pixhawk project. 
*/ 


// header files

#include "mcu_periph/i2c.h"

// variables 
struct i2c_frame_px4flow
{	uint16_t frame_count;// counts created I2C frames [#frames]
    int16_t pixel_flow_x_sum;// latest x flow measurement in pixels*10 [pixels]
    int16_t pixel_flow_y_sum;// latest y flow measurement in pixels*10 [pixels]
    int16_t flow_comp_m_x;// x velocity*1000 [meters/sec]
    int16_t flow_comp_m_y;// y velocity*1000 [meters/sec]
    int16_t qual;// Optical flow quality / confidence [0: bad, 255: maximum quality]
    int16_t gyro_x_rate; // latest gyro x rate [rad/sec]
    int16_t gyro_y_rate; // latest gyro y rate [rad/sec]
    int16_t gyro_z_rate; // latest gyro z rate [rad/sec]
    uint8_t gyro_range; // gyro range [0 .. 7] equals [50 deg/sec .. 2000 deg/sec] 
    uint8_t sonar_timestamp;// time since last sonar update [milliseconds]
    int16_t ground_distance;// Ground distance in meters*1000 [meters]. Positive value: distance known. Negative value: Unknown distance
};

struct i2c_integral_frame_px4flow
{   uint16_t frame_count_since_last_readout;//number of flow measurements since last I2C readout [#frames]
    int16_t pixel_flow_x_integral;//accumulated flow in radians*10000 around x axis since last I2C readout [rad*10000]
    int16_t pixel_flow_y_integral;//accumulated flow in radians*10000 around y axis since last I2C readout [rad*10000]
    int16_t gyro_x_rate_integral;//accumulated gyro x rates in radians*10000 since last I2C readout [rad*10000] 
    int16_t gyro_y_rate_integral;//accumulated gyro y rates in radians*10000 since last I2C readout [rad*10000] 
    int16_t gyro_z_rate_integral;//accumulated gyro z rates in radians*10000 since last I2C readout [rad*10000] 
    uint32_t integration_timespan;//accumulation timespan in microseconds since last I2C readout [microseconds]
    uint32_t sonar_timestamp;// time since last sonar update [microseconds]
    int16_t ground_distance;// Ground distance in meters*1000 [meters*1000]
    int16_t gyro_temperature;// Temperature * 100 in centi-degrees Celsius [degcelsius*100]
    uint8_t quality;// averaged quality of accumulated flow values [0:bad quality;255: max quality]
};

struct px4flow_I2c {
  struct i2c_periph *i2c_p;
  struct i2c_transaction i2c_trans;
  struct i2c_frame_px4flow data;
  struct i2c_integral_frame_px4flow integral_data;
  volatile bool_t data_available;     ///< data ready flag
};
// general addresses 
#ifndef PX4FLOW_I2C_DEV
#define PX4FLOW_I2C_DEV i2c2
#endif
#define PX4FLOW_I2C_ADDR 0x42 << 1

//register addresses
#define PX4FLOW_FRAME_READ_CMD 0x00


#ifndef PX4FLOW_SENSOR
#define PX4FLOW_SENSOR
//functions
extern void px4flow_i2c_init( void );
extern void px4flow_i2c_event( void );
#endif