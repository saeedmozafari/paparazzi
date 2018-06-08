
#include "modules/autmav/sonar_i2c_srf08.h"
#include "generated/airframe.h"
#include "led.h"
#include "filters/rms_filter.h"
#include "subsystems/abi.h"
#ifdef SITL
#include "state.h"
#endif
#include "generated/modules.h"
#include "mcu_periph/uart.h"
#include "pprzlink/messages.h"
#include "subsystems/datalink/downlink.h"
#include "subsystems/datalink/datalink.h"
#include "subsystems/datalink/telemetry.h"
#include "mcu_periph/i2c.h"
//#include "filters/median_filter.h"

//i2c number
#ifndef SONAR_I2C_DEV
#define SONAR_I2C_DEV i2c2
#endif

//sonar address
#ifndef SONAR_SLAVE_ADDR
#define SONAR_SLAVE_ADDR 0xE0
#endif

// 0x50 >> inch , 0x51 >> cm , 0x52 >> ms
#ifndef READ_SCALE
#define READ_SCALE 0x51
#endif

#ifndef GAIN_REG
#define GAIN_REG 0
#endif

#ifndef SONAR_RANGE
#define SONAR_RANGE 3
#endif

#ifndef USE_RMS_FILTER
#define USE_RMS_FILTER FALSE
#endif

//#define  MEDIAN_DATASIZE 10



//register id
#ifndef RESULT_REGISTER
#define RESULT_REGISTER 0x02
#endif

//set max range
#ifndef RANGE_REGISTER
#define RANGE_REGISTER 0xFF
#endif

static struct i2c_transaction srf08_trans;
static struct RMSFilterInt srf08_rms_filter;
//struct MedianFilterInt srf08_rms_filter;
float buf0 = 0;
int32_t buf1 = 0;
int32_t buf2 = 0;
int32_t buf3 = 0;
int32_t flag1 = 0;
int32_t flag2 = 0;
int32_t flag3 = 0;
int32_t flag4 = 0;
float distance;
uint8_t sonar_status;
uint8_t sonar_range;
uint8_t analog_gain;
uint32_t i=0;
bool use_rms;

/*static void send_sonar_i2c_srf08(struct transport_tx *trans, struct link_device *dev)
{
  pprz_msg_send_SONAR_I2C_SRF08(trans, dev, AC_ID,
                                          &buf0, &buf1, &buf2, &buf3, &flag1, &flag2, &flag3, &flag4);
}*/
static void send_sonar_debug(struct transport_tx *trans, struct link_device *dev)
{
  pprz_msg_send_SONAR_DEBUG(trans, dev, AC_ID,
                            &buf0, &buf0, &buf1);
}

void sonar_i2c_init(void)
{
  //register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_SONAR_I2C_SRF08, send_sonar_i2c_srf08);
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_SONAR_DEBUG, send_sonar_debug); 
  srf08_trans.status = I2CTransDone;
  sonar_status=SEND_COMMAND;
  sonar_range=SONAR_RANGE;
  analog_gain=GAIN_REG;
  use_rms=USE_RMS_FILTER;
  distance=0;
  //if(use_rms){
    //init_rms_filter(&srf08_rms_filter);
 // init_median_filter(&srf08_rms_filter);

  //}
  //i2c_transmit(&SONAR_I2C_DEV, &srf08_trans, SONAR_SLAVE_ADDR, 1);
}




void sonar_i2c_read(void)
{
  switch (sonar_status) {
    case SEND_COMMAND:
         srf08_trans.buf[0] = 0;
         srf08_trans.buf[1] = READ_SCALE;
      i2c_transmit(&SONAR_I2C_DEV , &srf08_trans, SONAR_SLAVE_ADDR, 2);
      break;
    case SET_RANGE:
    //srf08_trans.buf[0] = 3;
    srf08_trans.buf[0] = 2;
    srf08_trans.buf[1] = sonar_range*23;
    i2c_transmit(&SONAR_I2C_DEV , &srf08_trans, SONAR_SLAVE_ADDR, 2);
      break;
    case SET_GAIN:
    //srf08_trans.buf[0] = 3;
    srf08_trans.buf[0] = 1;
    srf08_trans.buf[1] = analog_gain;
    i2c_transmit(&SONAR_I2C_DEV , &srf08_trans, SONAR_SLAVE_ADDR, 2);
      break;
    case REQ_RANGE:
    //srf08_trans.buf[0] = 3;
    srf08_trans.buf[0] = 2;
    i2c_transceive(&SONAR_I2C_DEV, &srf08_trans, SONAR_SLAVE_ADDR, 1, 2);
      break;
    case RANGE_READ_OK:
    //if(use_rms){
    flag1=get_sys_time_msec();
    if(flag1>30000){
      flag2++;
    }
    if(flag1>50000 && flag3==0){
      flag3=flag2/20;
    }
     // distance=update_rms_filter(&srf08_rms_filter, (int32_t)((srf08_trans.buf[0] << 8) | srf08_trans.buf[1]));

     // distance=update_median_filter(&srf08_rms_filter,
                                 // (int32_t)((srf08_trans.buf[0] << 8) | srf08_trans.buf[1]));
      
   // }else{
     distance=(uint32_t)((srf08_trans.buf[0] << 8) | srf08_trans.buf[1]);
  //  }
    buf0=distance/100;
    uint8_t zero = 0;
    DOWNLINK_SEND_LIDAR(DefaultChannel, DefaultDevice, &buf0, &zero, &zero);
    AbiSendMsgAGL(AGL_SONAR_ADC_ID, buf0);
     /*buf0=srf08_trans.buf[0];
     buf1=srf08_trans.buf[1];*/
    sonar_status=REQ_LIGHT;
      break;
    case REQ_LIGHT:
    srf08_trans.buf[0] = 1;
    i2c_transceive(&SONAR_I2C_DEV, &srf08_trans, SONAR_SLAVE_ADDR, 1, 1);
      break;
    case READ_LIGHT_OK:
    buf1=srf08_trans.buf[0];
    sonar_status=SEND_COMMAND;
      break;
    default:
      break;
  }
  /*flag1++;
  if(flag1/100==1){
    flag1=0;
    flag2--;
    if(flag2<0){
      flag2=31;
    }
  }*/
}
void sonar_srf08_event(void)
{

  switch (srf08_trans.status) {
    case I2CTransPending:
      // wait and do nothing
      break;
    case I2CTransRunning:
      // wait and do nothing
    flag4++;
      break;
    case I2CTransSuccess:
      // set to done
    srf08_trans.status = I2CTransDone;
    if(sonar_status==SEND_COMMAND){
      sonar_status=SET_RANGE;
    }else if(sonar_status==SET_RANGE){
          sonar_status=SET_GAIN;
    }else if(sonar_status==SET_GAIN){
          sonar_status=REQ_RANGE;
    }else if(sonar_status==REQ_RANGE){
      sonar_status=RANGE_READ_OK;
    }else if(sonar_status==REQ_LIGHT){
      sonar_status=READ_LIGHT_OK;
    }
      break;
    case I2CTransFailed:
      // set to done
      srf08_trans.status = I2CTransDone;
      break;
    case I2CTransDone:
      // do nothing
      break;
    default:
      break;
  }
}

float GetSonarAlt(void){
  return buf0;
}
/*void init_median_filter(struct MedianFilterInt *filter)
{
  int i;
  for (i = 0; i < MEDIAN_DATASIZE; i++) {
    filter->data[i] = 0;
    filter->sortData[i] = 0;
  }
  filter->dataIndex = 0;
}
int32_t update_median_filter(struct MedianFilterInt *filter, int32_t new_data)
{
  int temp, i, j; // used to sort array

  // Insert new data into raw data array round robin style
  filter->data[filter->dataIndex] = new_data;
  if (filter->dataIndex < (MEDIAN_DATASIZE - 1)) {
    filter->dataIndex++;
  } else {
    filter->dataIndex = 0;
  }

  // Copy raw data to sort data array
  memcpy(filter->sortData, filter->data, sizeof(filter->data));

  // Insertion Sort
  for (i = 1; i <= (MEDIAN_DATASIZE - 1); i++) {
    temp = filter->sortData[i];
    j = i - 1;
    while (j >= 0 && temp < filter->sortData[j]) {
      filter->sortData[j + 1] = filter->sortData[j];
      j = j - 1;
    }
    filter->sortData[j + 1] = temp;
  }
  return filter->sortData[(MEDIAN_DATASIZE) >> 1]; // return data value in middle of sorted array
}*/