/*
############################## AUTMAV ##############################
#	PROJECT: AUTMAV MODIFIED PAPARAZZI						              	   #
#	DEVELOPED BY: ALI JAMEEI								                    	   #
#	Copyright (C) 2014-2015								                     		   #	
####################################################################
*/


/* DESCRIPTION: 
	  the generic driver for Apogee baro doesn't work on AUTMAV boards. 
	  GY-86 is an external hardware bought for this purpose which utilizes 
	  MPU6050	alongside MS5611 barometer chip. this module file is going to 
	  integrate this hardware to the board.
*/ 

#include "modules/autmav/gy86_imu_baro.h"
#include "led.h"
// some variables for the barometer
struct Ms5611_I2c baro_ms5611;
struct ImuApogee imu_aut_aux;

float fbaroms;
float baro_ms5611_alt;
bool_t baro_ms5611_alt_valid;
bool_t baro_ms5611_enabled;

extern bool_t configure_baro_slave(Mpu60x0ConfigSet mpu_set, void *mpu);

void gy86_baro_init(void) {
    ms5611_i2c_init(&baro_ms5611, &MS5611_I2C_DEV, MS5611_SLAVE_ADDR,FALSE);
    baro_ms5611_enabled = TRUE;
    baro_ms5611_alt_valid = FALSE;
}

// set MPU in bypass mode for the baro
void gy86_mpu_init(void)
{ /////////////////////////////////////////////////////////////////////
  // MPU-60X0
  mpu60x0_i2c_init(&imu_aut_aux.mpu, &MS5611_I2C_DEV, MPU60X0_ADDR);
  imu_aut_aux.mpu.config.nb_slaves = 1;
  imu_aut_aux.mpu.config.slaves[0].configure = &configure_baro_slave;
  imu_aut_aux.mpu.config.i2c_bypass = TRUE;
}

void gy86_baro_periodic( void ) {
  if(imu_aut_aux.mpu.config.initialized){
    ms5611_i2c_periodic_check(&baro_ms5611);
    if (sys_time.nb_sec > 1) {
      ms5611_i2c_read(&baro_ms5611);
    }
  }
}

void gy86_mpu_periodic( void )
{
  // Start reading the latest gyroscope data
  mpu60x0_i2c_periodic(&imu_aut_aux.mpu);
  //RunOnceEvery(10,imu_aut_aux_downlink_raw());
}

void gy86_baro_event( void ) {
  ms5611_i2c_event(&baro_ms5611);
  if (baro_ms5611.data_available) {
    #ifdef AUX_BARO_LED
      LED_ON(AUX_BARO_LED);
    #endif
    float pressure = (float)baro_ms5611.data.pressure;
    AbiSendMsgBARO_ABS(BARO_MS5611_SENDER_ID, pressure);
    float temp = baro_ms5611.data.temperature / 100.0f;
    AbiSendMsgTEMPERATURE(BARO_MS5611_SENDER_ID, temp);
    baro_ms5611.data_available = FALSE;

    baro_ms5611_alt = pprz_isa_altitude_of_pressure(pressure);
    baro_ms5611_alt_valid = TRUE;

#ifdef SENSOR_SYNC_SEND
    fbaroms = baro_ms5611.data.pressure / 100.;
    DOWNLINK_SEND_BARO_MS5611(DefaultChannel, DefaultDevice,
                               &baro_ms5611.data.d1, &baro_ms5611.data.d2,
                              &fbaroms, &temp);
#endif
  }
}

void gy86_mpu_event( void ) {
  mpu60x0_i2c_read(&imu_aut_aux.mpu);
  // If the itg3200 I2C transaction has succeeded: convert the data
  mpu60x0_i2c_event(&imu_aut_aux.mpu);
  if (imu_aut_aux.mpu.data_available) {
    
    //RATES_ASSIGN(imu.gyro_unscaled, imu_aut_aux.mpu.data_rates.rates.p, -imu_aut_aux.mpu.data_rates.rates.q, -imu_aut_aux.mpu.data_rates.rates.r);
    //VECT3_ASSIGN(imu.accel_unscaled, imu_aut_aux.mpu.data_accel.vect.x, -imu_aut_aux.mpu.data_accel.vect.y, -imu_aut_aux.mpu.data_accel.vect.z);
    imu_aut_aux.gyr_valid = TRUE;
    imu_aut_aux.acc_valid = TRUE;
    
    imu_aut_aux.mpu.data_available = FALSE;
  }
}
