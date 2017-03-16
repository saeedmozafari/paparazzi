#ifndef RTK_RECEIVE_H
#define RTK_RECEIVE_H

#include "subsystems/datalink/datalink.h" // dl_buffer
#include "generated/airframe.h"           // AC_ID

#include "pprzlink/pprzlink_device.h"
#include "mcu_periph/uart.h"

#include "subsystems/gps.h"

extern void relay_msg(uint8_t length, uint8_t *relay_data);
extern void RTK_receive_init(void);
extern void tag_image_log(void);
extern void tag_image(void);

static inline void parse_DL_RTCM_INJECT(void)
{
#ifndef SITL	
  //uint8_t packet_id = DL_RTCM_INJECT_packet_id(dl_buffer);
	uint8_t data_length = DL_RTCM_INJECT_data_length(dl_buffer);
	uint8_t *RTCM3_data = DL_RTCM_INJECT_data(dl_buffer);	
  relay_msg(data_length, RTCM3_data);
#endif
}

extern void rtk_gps_ubx_init(void);
extern void rtk_gps_ubx_event(void);

#define rtk_gps_ubx_periodic_check() rtk_gps_periodic_check(&rtk_gps_ubx.state)

#define RTK_GPS_UBX_NB_CHANNELS 16

#define RTK_GPS_UBX_MAX_PAYLOAD 255
struct RTKGpsUbx {
  bool msg_available;
  uint8_t msg_buf[RTK_GPS_UBX_MAX_PAYLOAD] __attribute__((aligned));
  uint8_t msg_id;
  uint8_t msg_class;

  uint8_t status;
  uint16_t len;
  uint8_t msg_idx;
  uint8_t ck_a, ck_b;
  uint8_t send_ck_a, send_ck_b;
  uint8_t error_cnt;
  uint8_t error_last;

  uint8_t status_flags;
  uint8_t sol_flags;

  struct GpsState state;
};

extern struct RTKGpsUbx rtk_gps_ubx;

#if USE_RTK_GPS_UBX_RXM_RAW
struct RTKGpsUbxGpsUbxRawMes {
  double cpMes;
  double prMes;
  float doMes;
  uint8_t sv;
  int8_t mesQI;
  int8_t cno;
  uint8_t lli;
};

struct RTKGpsUbxGpsUbxRaw {
  int32_t iTOW;
  int16_t week;
  uint8_t numSV;
  struct GpsUbxRawMes measures[RTK_GPS_UBX_NB_CHANNELS];
};

extern struct GpsUbxRaw rtk_gps_ubx_raw;
#endif

extern double log_phi;
extern double log_theta;
extern double log_psi;
extern double log_lat,log_lon,log_alt;
extern double log_vacc,log_hacc;

/*
 * This part is used by the autopilot to read data from a uart
 */
#include "pprzlink/pprzlink_device.h"

extern void ubx_header(struct link_device *dev, uint8_t nav_id, uint8_t msg_id, uint16_t len);
extern void ubx_trailer(struct link_device *dev);
extern void ubx_send_bytes(struct link_device *dev, uint8_t len, uint8_t *bytes);
extern void ubx_send_cfg_rst(struct link_device *dev, uint16_t bbr, uint8_t reset_mode);

extern void rtk_gps_ubx_read_message(void);
extern void rtk_gps_ubx_parse(uint8_t c);
extern void rtk_gps_ubx_msg(void);

/*
 * GPS Reset
 */

#define CFG_RST_Reset_Hardware 0x00
#define CFG_RST_Reset_Controlled 0x01
#define CFG_RST_Reset_Controlled_RTK_GPS_only 0x02
#define CFG_RST_Reset_Controlled_RTK_GPS_stop 0x08
#define CFG_RST_Reset_Controlled_RTK_GPS_start 0x09

#define CFG_RST_BBR_Hotstart  0x0000
#define CFG_RST_BBR_Warmstart 0x0001
#define CFG_RST_BBR_Coldstart 0xffff

#define rtk_gps_ubx_Reset(_val) {                               \
    rtk_gps_ubx.reset = _val;                                       \
    if (rtk_gps_ubx.reset > CFG_RST_BBR_Warmstart)                  \
      rtk_gps_ubx.reset = CFG_RST_BBR_Coldstart;                    \
    ubx_send_cfg_rst(&(UBX_RTK_GPS_LINK).device, rtk_gps_ubx.reset, CFG_RST_Reset_Controlled);   \
  }

#endif