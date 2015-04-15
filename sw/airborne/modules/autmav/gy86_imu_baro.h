/*
############################## AUTMAV ##############################
# PROJECT: AUTMAV MODIFIED PAPARAZZI							   #
# DEVELOPED BY: ALI JAMEEI										   #
# Copyright (C) 2014-2015										   #	
####################################################################
*/


/* DESCRIPTION: 
	the generic driver for Apogee baro doesn't work on AUTMAV boards. 
	GY-86 is an external hardware bought for this purpose which utilizes 
	MPU6050	alongside MS5611 barometer chip. this module file is going to 
	integrate this hardware to the board.
*/ 


// header files

#include "math/pprz_isa.h"
#include "mcu_periph/sys_time.h"
#include "subsystems/abi.h"
#include "mcu_periph/uart.h"
#include "messages.h"
#include "subsystems/datalink/downlink.h"
#include "mcu_periph/i2c.h"
#include "peripherals/ms5611_i2c.h"
#include "std.h"
#include "peripherals/mpu60x0_i2c.h"
#include "peripherals/mpu60x0.h"
#include "boards/apogee/imu_apogee.h"

// barometer configuration and parameters definition
#ifndef BARO_MS56111_I2C_H
#define BARO_MS56111_I2C_H
// functions
extern void gy86_baro_init(void);
extern void gy86_baro_periodic(void);
extern void gy86_baro_event(void);
extern void gy86_mpu_init(void);
extern void gy86_mpu_periodic(void);
extern void gy86_mpu_event(void);
#endif 

// I2C address to be used for external hardware 
#ifndef MS5611_I2C_DEV
#define MS5611_I2C_DEV i2c1
#endif

/* address can be 0xEC or 0xEE (CSB\ low = 0xEE) */
#ifndef MS5611_SLAVE_ADDR
#define MS5611_SLAVE_ADDR 0xEE
#endif


