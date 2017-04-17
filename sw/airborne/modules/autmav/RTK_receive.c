#define RTK_RECEIVE_C

#include "RTK_receive.h"

#include "subsystems/gps.h"
#include "subsystems/abi_sender_ids.h" 
#include "subsystems/datalink/downlink.h"
#include "modules/loggers/sdlog_chibios.h"
#include "modules/autmav/sony_camera_handler.h"
#include "state.h"
#include "subsystems/datalink/telemetry.h"
#include "subsystems/abi.h"
#include "mcu_periph/sys_time.h"

/** Includes macros generated from ubx.xml */
#include "ubx_protocol.h"
#include "led.h"
#include "wifi_cam_ctrl.h"
#include "subsystems/gps/librtcm3/CRC24Q.h"

struct link_device *RTK_receive_device;

void RTK_receive_init(void){
    
    RTK_receive_device = &((RTK_RTCM3_UART_DEV).device);
	uart_periph_set_bits_stop_parity(&RTK_RTCM3_UART_DEV, UBITS_8, USTOP_1, UPARITY_NO);
	uart_periph_set_baudrate(&RTK_RTCM3_UART_DEV, RTK_RTCM3_UART_BAUD);

}

void relay_msg(uint8_t length, uint8_t *relay_data){

	int i;
	for (i = 0; i < length; i++) {
    	RTK_receive_device->put_byte(RTK_receive_device->periph, 0, relay_data[i]);
  	}
}


/* parser status */
#define UNINIT        0
#define GOT_SYNC1     1
#define GOT_SYNC2     2
#define GOT_CLASS     3
#define GOT_ID        4
#define GOT_LEN1      5
#define GOT_LEN2      6
#define GOT_PAYLOAD   7
#define GOT_CHECKSUM1 8

/* last error type */
#define RTK_GPS_UBX_ERR_NONE         0
#define RTK_GPS_UBX_ERR_OVERRUN      1
#define RTK_GPS_UBX_ERR_MSG_TOO_LONG 2
#define RTK_GPS_UBX_ERR_CHECKSUM     3
#define RTK_GPS_UBX_ERR_UNEXPECTED   4
#define RTK_GPS_UBX_ERR_OUT_OF_SYNC  5

#define UTM_HEM_NORTH 0
#define UTM_HEM_SOUTH 1

#define RXM_RTCM_VERSION        0x02
#define NAV_RELPOSNED_VERSION   0x00

#define RTK_GPS_UBX_ID GPS_UBX_ID
struct RTKGpsUbx rtk_gps_ubx;

struct RTKGpsUbxRaw rtk_gps_ubx_raw;

double log_phi;
double log_theta;
double log_psi;
double log_lat,log_lon,log_alt;
double log_vacc,log_hacc;

double log_shot_command_time_stamp;
double log_target_rtk_time_stamp;

double last_rtk_msg_time;

extern struct GpsRelposNED gps_relposned;
extern struct RtcmMan rtcm_man;

struct GpsTimeSync rtk_gps_rtk_ubx_time_sync;

static void send_rtk_status(struct transport_tx *trans, struct link_device *dev)
{
  pprz_msg_send_RTK_STATUS(trans, dev, AC_ID,
                        &rtk_gps_ubx.state.ecef_pos.x, &rtk_gps_ubx.state.ecef_pos.y, &rtk_gps_ubx.state.ecef_pos.z,
                        &rtk_gps_ubx.state.lla_pos.lat, &rtk_gps_ubx.state.lla_pos.lon, &rtk_gps_ubx.state.lla_pos.alt,
                        &rtk_gps_ubx.state.hmsl,
                        &rtk_gps_ubx.state.ecef_vel.x, &rtk_gps_ubx.state.ecef_vel.y, &rtk_gps_ubx.state.ecef_vel.z,
                        &rtk_gps_ubx.state.pacc, &rtk_gps_ubx.state.sacc,
                        &rtk_gps_ubx.state.tow,
                        &rtk_gps_ubx.state.pdop,
                        &rtk_gps_ubx.state.num_sv,
                        &rtk_gps_ubx.state.fix,
                        &rtk_gps_ubx.state.comp_id,
                        &rtk_gps_ubx.state.hacc,
                        &rtk_gps_ubx.state.vacc,
                        &pprzLogFile);
}

void rtk_gps_ubx_init(void)
{
  rtk_gps_ubx.status = UNINIT;
  rtk_gps_ubx.msg_available = false;
  rtk_gps_ubx.error_cnt = 0;
  rtk_gps_ubx.error_last = RTK_GPS_UBX_ERR_NONE;
  rtk_gps_ubx.state.comp_id = RTK_GPS_UBX_ID;
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_RTK_STATUS, send_rtk_status);
  log_phi = 0;
  log_theta = 0;
  log_psi = 0;
  log_lat = 0; log_lon = 0; log_alt = 0;
  log_vacc = 0; log_hacc = 0;
  log_shot_command_time_stamp = 0.0;
  log_target_rtk_time_stamp = 0.0;
}

void rtk_gps_ubx_event(void)
{
  struct link_device *dev = &((UBX_RTK_GPS_LINK).device);

  while (dev->char_available(dev->periph)) {
    rtk_gps_ubx_parse(dev->get_byte(dev->periph));
    if (rtk_gps_ubx.msg_available) {
      rtk_gps_ubx_msg();
    }
  }
}

void rtk_gps_ubx_read_message(void)
{

  if (rtk_gps_ubx.msg_class == UBX_NAV_ID) {
    if (rtk_gps_ubx.msg_id == UBX_NAV_SOL_ID) {
      /* get hardware clock ticks */
      rtk_gps_rtk_ubx_time_sync.t0_ticks      = sys_time.nb_tick;
      rtk_gps_rtk_ubx_time_sync.t0_tow        = UBX_NAV_SOL_ITOW(rtk_gps_ubx.msg_buf);
      rtk_gps_rtk_ubx_time_sync.t0_tow_frac   = UBX_NAV_SOL_Frac(rtk_gps_ubx.msg_buf);
      rtk_gps_ubx.state.tow        = UBX_NAV_SOL_ITOW(rtk_gps_ubx.msg_buf);
      rtk_gps_ubx.state.week       = UBX_NAV_SOL_week(rtk_gps_ubx.msg_buf);
      //rtk_gps_ubx.state.fix        = UBX_NAV_SOL_GPSfix(rtk_gps_ubx.msg_buf);
      rtk_gps_ubx.state.ecef_pos.x = UBX_NAV_SOL_ECEF_X(rtk_gps_ubx.msg_buf);
      rtk_gps_ubx.state.ecef_pos.y = UBX_NAV_SOL_ECEF_Y(rtk_gps_ubx.msg_buf);
      rtk_gps_ubx.state.ecef_pos.z = UBX_NAV_SOL_ECEF_Z(rtk_gps_ubx.msg_buf);
      SetBit(rtk_gps_ubx.state.valid_fields, GPS_VALID_POS_ECEF_BIT);
      rtk_gps_ubx.state.pacc       = UBX_NAV_SOL_Pacc(rtk_gps_ubx.msg_buf);
      rtk_gps_ubx.state.ecef_vel.x = UBX_NAV_SOL_ECEFVX(rtk_gps_ubx.msg_buf);
      rtk_gps_ubx.state.ecef_vel.y = UBX_NAV_SOL_ECEFVY(rtk_gps_ubx.msg_buf);
      rtk_gps_ubx.state.ecef_vel.z = UBX_NAV_SOL_ECEFVZ(rtk_gps_ubx.msg_buf);
      SetBit(rtk_gps_ubx.state.valid_fields, GPS_VALID_VEL_ECEF_BIT);
      rtk_gps_ubx.state.sacc       = UBX_NAV_SOL_Sacc(rtk_gps_ubx.msg_buf);
      rtk_gps_ubx.state.pdop       = UBX_NAV_SOL_PDOP(rtk_gps_ubx.msg_buf);
      rtk_gps_ubx.state.num_sv     = UBX_NAV_SOL_numSV(rtk_gps_ubx.msg_buf);
#ifdef RTK_GPS_LED
      if (rtk_gps_ubx.state.fix == GPS_FIX_3D) {
        LED_ON(RTK_GPS_LED);
      } else {
        LED_TOGGLE(RTK_GPS_LED);
      }
#endif
    } else if (rtk_gps_ubx.msg_id == UBX_NAV_POSLLH_ID) {
      rtk_gps_ubx.state.lla_pos.lat = UBX_NAV_POSLLH_LAT(rtk_gps_ubx.msg_buf);
      rtk_gps_ubx.state.lla_pos.lon = UBX_NAV_POSLLH_LON(rtk_gps_ubx.msg_buf);
      rtk_gps_ubx.state.lla_pos.alt = UBX_NAV_POSLLH_HEIGHT(rtk_gps_ubx.msg_buf);
      SetBit(rtk_gps_ubx.state.valid_fields, GPS_VALID_POS_LLA_BIT);
      rtk_gps_ubx.state.hmsl        = UBX_NAV_POSLLH_HMSL(rtk_gps_ubx.msg_buf);
      SetBit(rtk_gps_ubx.state.valid_fields, GPS_VALID_HMSL_BIT);
      rtk_gps_ubx.state.hacc = UBX_NAV_POSLLH_Hacc(rtk_gps_ubx.msg_buf);
	    rtk_gps_ubx.state.vacc = UBX_NAV_POSLLH_Vacc(rtk_gps_ubx.msg_buf);
      last_rtk_msg_time = get_sys_time_float();

    } else if (rtk_gps_ubx.msg_id == UBX_NAV_POSUTM_ID) {
      rtk_gps_ubx.state.utm_pos.east = UBX_NAV_POSUTM_EAST(rtk_gps_ubx.msg_buf);
      rtk_gps_ubx.state.utm_pos.north = UBX_NAV_POSUTM_NORTH(rtk_gps_ubx.msg_buf);
      uint8_t hem = UBX_NAV_POSUTM_HEM(rtk_gps_ubx.msg_buf);
      if (hem == UTM_HEM_SOUTH) {
        rtk_gps_ubx.state.utm_pos.north -= 1000000000;  /* Subtract false northing: -10000km */
      }
      rtk_gps_ubx.state.utm_pos.alt = UBX_NAV_POSUTM_ALT(rtk_gps_ubx.msg_buf) * 10;
      rtk_gps_ubx.state.utm_pos.zone = UBX_NAV_POSUTM_ZONE(rtk_gps_ubx.msg_buf);
      SetBit(rtk_gps_ubx.state.valid_fields, GPS_VALID_POS_UTM_BIT);

      rtk_gps_ubx.state.hmsl = rtk_gps_ubx.state.utm_pos.alt;
      SetBit(rtk_gps_ubx.state.valid_fields, GPS_VALID_HMSL_BIT);
    } else if (rtk_gps_ubx.msg_id == UBX_NAV_VELNED_ID) {
      rtk_gps_ubx.state.speed_3d = UBX_NAV_VELNED_Speed(rtk_gps_ubx.msg_buf);
      rtk_gps_ubx.state.gspeed = UBX_NAV_VELNED_GSpeed(rtk_gps_ubx.msg_buf);
      rtk_gps_ubx.state.ned_vel.x = UBX_NAV_VELNED_VEL_N(rtk_gps_ubx.msg_buf);
      rtk_gps_ubx.state.ned_vel.y = UBX_NAV_VELNED_VEL_E(rtk_gps_ubx.msg_buf);
      rtk_gps_ubx.state.ned_vel.z = UBX_NAV_VELNED_VEL_D(rtk_gps_ubx.msg_buf);
      SetBit(rtk_gps_ubx.state.valid_fields, GPS_VALID_VEL_NED_BIT);
      // Ublox gives I4 heading in 1e-5 degrees, apparenty from 0 to 360 degrees (not -180 to 180)
      // I4 max = 2^31 = 214 * 1e5 * 100 < 360 * 1e7: overflow on angles over 214 deg -> casted to -214 deg
      // solution: First to radians, and then scale to 1e-7 radians
      // First x 10 for loosing less resolution, then to radians, then multiply x 10 again
      rtk_gps_ubx.state.course = (RadOfDeg(UBX_NAV_VELNED_Heading(rtk_gps_ubx.msg_buf) * 10)) * 10;
      SetBit(rtk_gps_ubx.state.valid_fields, GPS_VALID_COURSE_BIT);
      rtk_gps_ubx.state.cacc = (RadOfDeg(UBX_NAV_VELNED_CAcc(rtk_gps_ubx.msg_buf) * 10)) * 10;
      rtk_gps_ubx.state.tow = UBX_NAV_VELNED_ITOW(rtk_gps_ubx.msg_buf);
    } else if (rtk_gps_ubx.msg_id == UBX_NAV_SVINFO_ID) {
      rtk_gps_ubx.state.nb_channels = Min(UBX_NAV_SVINFO_NCH(rtk_gps_ubx.msg_buf), GPS_NB_CHANNELS);
      uint8_t i;
      for (i = 0; i < rtk_gps_ubx.state.nb_channels; i++) {
        rtk_gps_ubx.state.svinfos[i].svid = UBX_NAV_SVINFO_SVID(rtk_gps_ubx.msg_buf, i);
        rtk_gps_ubx.state.svinfos[i].flags = UBX_NAV_SVINFO_Flags(rtk_gps_ubx.msg_buf, i);
        rtk_gps_ubx.state.svinfos[i].qi = UBX_NAV_SVINFO_QI(rtk_gps_ubx.msg_buf, i);
        rtk_gps_ubx.state.svinfos[i].cno = UBX_NAV_SVINFO_CNO(rtk_gps_ubx.msg_buf, i);
        rtk_gps_ubx.state.svinfos[i].elev = UBX_NAV_SVINFO_Elev(rtk_gps_ubx.msg_buf, i);
        rtk_gps_ubx.state.svinfos[i].azim = UBX_NAV_SVINFO_Azim(rtk_gps_ubx.msg_buf, i);
      }
    } else if (rtk_gps_ubx.msg_id == UBX_NAV_STATUS_ID) {
      rtk_gps_ubx.state.fix = UBX_NAV_STATUS_GPSfix(rtk_gps_ubx.msg_buf);
      rtk_gps_ubx.status_flags = UBX_NAV_STATUS_Flags(rtk_gps_ubx.msg_buf);
      rtk_gps_ubx.sol_flags = UBX_NAV_SOL_Flags(rtk_gps_ubx.msg_buf);
    } else if (rtk_gps_ubx.msg_id == UBX_NAV_RELPOSNED_ID) {
      uint8_t version = UBX_NAV_RELPOSNED_VERSION(rtk_gps_ubx.msg_buf);
      if (version == NAV_RELPOSNED_VERSION) {
        gps_relposned.iTOW          = UBX_NAV_RELPOSNED_ITOW(rtk_gps_ubx.msg_buf);
        gps_relposned.refStationId  = UBX_NAV_RELPOSNED_refStationId(rtk_gps_ubx.msg_buf);
        gps_relposned.relPosN     = UBX_NAV_RELPOSNED_RELPOSN(rtk_gps_ubx.msg_buf);
        gps_relposned.relPosE     = UBX_NAV_RELPOSNED_RELPOSE(rtk_gps_ubx.msg_buf);
        gps_relposned.relPosD     = UBX_NAV_RELPOSNED_RELPOSD(rtk_gps_ubx.msg_buf) ;
        gps_relposned.relPosHPN   = UBX_NAV_RELPOSNED_RELPOSNHP(rtk_gps_ubx.msg_buf);
        gps_relposned.relPosHPE   = UBX_NAV_RELPOSNED_RELPOSEHP(rtk_gps_ubx.msg_buf);
        gps_relposned.relPosHPD   = UBX_NAV_RELPOSNED_RELPOSDHP(rtk_gps_ubx.msg_buf);
        gps_relposned.accN      = UBX_NAV_RELPOSNED_Nacc(rtk_gps_ubx.msg_buf);
        gps_relposned.accE      = UBX_NAV_RELPOSNED_Eacc(rtk_gps_ubx.msg_buf);
        gps_relposned.accD      = UBX_NAV_RELPOSNED_Dacc(rtk_gps_ubx.msg_buf);
        uint8_t flags           = UBX_NAV_RELPOSNED_Flags(rtk_gps_ubx.msg_buf);
        gps_relposned.carrSoln    = RTCMgetbitu(&flags, 3, 2);
        gps_relposned.relPosValid   = RTCMgetbitu(&flags, 5, 1);
        gps_relposned.diffSoln    = RTCMgetbitu(&flags, 6, 1);
        gps_relposned.gnssFixOK   = RTCMgetbitu(&flags, 7, 1);
        if (gps_relposned.gnssFixOK > 0) {
          if (gps_relposned.diffSoln > 0) {
            if (gps_relposned.carrSoln == 2) {
              rtk_gps_ubx.state.fix = 5; // rtk
            } else {
              rtk_gps_ubx.state.fix = 4; // dgnss
            }
          } else {
            rtk_gps_ubx.state.fix = 3; // 3D
          }
        } else {
          rtk_gps_ubx.state.fix = 0;
        }
      }

    }
  }
  else if (rtk_gps_ubx.msg_class == UBX_RXM_ID) {
    if (rtk_gps_ubx.msg_id == UBX_RXM_RAW_ID) {
      rtk_gps_ubx_raw.iTOW = UBX_RXM_RAW_iTOW(rtk_gps_ubx.msg_buf);
      rtk_gps_ubx_raw.week = UBX_RXM_RAW_week(rtk_gps_ubx.msg_buf);
      rtk_gps_ubx_raw.numSV = UBX_RXM_RAW_numSV(rtk_gps_ubx.msg_buf);
      uint8_t i;
      uint8_t max_SV = Min(rtk_gps_ubx_raw.numSV, GPS_UBX_NB_CHANNELS);
      for (i = 0; i < max_SV; i++) {
        rtk_gps_ubx_raw.measures[i].cpMes = UBX_RXM_RAW_cpMes(rtk_gps_ubx.msg_buf, i);
        rtk_gps_ubx_raw.measures[i].prMes = UBX_RXM_RAW_prMes(rtk_gps_ubx.msg_buf, i);
        rtk_gps_ubx_raw.measures[i].doMes = UBX_RXM_RAW_doMes(rtk_gps_ubx.msg_buf, i);
        rtk_gps_ubx_raw.measures[i].sv = UBX_RXM_RAW_sv(rtk_gps_ubx.msg_buf, i);
        rtk_gps_ubx_raw.measures[i].mesQI = UBX_RXM_RAW_mesQI(rtk_gps_ubx.msg_buf, i);
        rtk_gps_ubx_raw.measures[i].cno = UBX_RXM_RAW_cno(rtk_gps_ubx.msg_buf, i);
        rtk_gps_ubx_raw.measures[i].lli = UBX_RXM_RAW_lli(rtk_gps_ubx.msg_buf, i);
      }
    } else if (rtk_gps_ubx.msg_id == UBX_RXM_RTCM_ID) {
      uint8_t version   = UBX_RXM_RTCM_version(rtk_gps_ubx.msg_buf);
      if (version == RXM_RTCM_VERSION) {
        //      uint8_t flags     = UBX_RXM_RTCM_flags(gps_ubx.msg_buf);
        //      bool crcFailed    = RTCMgetbitu(&flags, 7, 1);
        //      uint16_t refStation = UBX_RXM_RTCM_refStation(gps_ubx.msg_buf);
        //      uint16_t msgType  = UBX_RXM_RTCM_msgType(gps_ubx.msg_buf);
        //      DEBUG_PRINT("Message %i from refStation %i processed (CRCfailed: %i)\n", msgType, refStation, crcFailed);

        rtcm_man.RefStation  = UBX_RXM_RTCM_refStation(rtk_gps_ubx.msg_buf);
        rtcm_man.MsgType     = UBX_RXM_RTCM_msgType(rtk_gps_ubx.msg_buf);
        uint8_t flags     = UBX_RXM_RTCM_flags(rtk_gps_ubx.msg_buf);
        bool crcFailed    = RTCMgetbitu(&flags, 7, 1);
        switch (rtcm_man.MsgType) {
          case 1005:
            rtcm_man.Cnt105 += 1;
            rtcm_man.Crc105 += crcFailed;
            break;
          case 1077:
            rtcm_man.Cnt177 += 1;
            rtcm_man.Crc177 += crcFailed;;
            break;
          case 1087:
            rtcm_man.Cnt187 += 1;
            rtcm_man.Crc187 += crcFailed;;
            break;
          default:
            break;
        }
      } else {
        //DEBUG_PRINT("Unknown RXM_RTCM version: %i\n", version);
      }
    }
  } 
}

#if LOG_RAW_GPS
#include "modules/loggers/sdlog_chibios.h"
#endif

/* UBX parsing */
void rtk_gps_ubx_parse(uint8_t c)
{
#if LOG_RAW_GPS
  sdLogWriteByte(pprzLogFile, c);
#endif
  if (rtk_gps_ubx.status < GOT_PAYLOAD) {
    rtk_gps_ubx.ck_a += c;
    rtk_gps_ubx.ck_b += rtk_gps_ubx.ck_a;
  }
  switch (rtk_gps_ubx.status) {
    case UNINIT:
      if (c == UBX_SYNC1) {
        rtk_gps_ubx.status++;
      }
      break;
    case GOT_SYNC1:
      if (c != UBX_SYNC2) {
        rtk_gps_ubx.error_last = RTK_GPS_UBX_ERR_OUT_OF_SYNC;
        goto error;
      }
      rtk_gps_ubx.ck_a = 0;
      rtk_gps_ubx.ck_b = 0;
      rtk_gps_ubx.status++;
      break;
    case GOT_SYNC2:
      if (rtk_gps_ubx.msg_available) {
        /* Previous message has not yet been parsed: discard this one */
        rtk_gps_ubx.error_last = RTK_GPS_UBX_ERR_OVERRUN;
        goto error;
      }
      rtk_gps_ubx.msg_class = c;
      rtk_gps_ubx.status++;
      break;
    case GOT_CLASS:
      rtk_gps_ubx.msg_id = c;
      rtk_gps_ubx.status++;
      break;
    case GOT_ID:
      rtk_gps_ubx.len = c;
      rtk_gps_ubx.status++;
      break;
    case GOT_LEN1:
      rtk_gps_ubx.len |= (c << 8);
      if (rtk_gps_ubx.len > RTK_GPS_UBX_MAX_PAYLOAD) {
        rtk_gps_ubx.error_last = RTK_GPS_UBX_ERR_MSG_TOO_LONG;
        goto error;
      }
      rtk_gps_ubx.msg_idx = 0;
      rtk_gps_ubx.status++;
      break;
    case GOT_LEN2:
      rtk_gps_ubx.msg_buf[rtk_gps_ubx.msg_idx] = c;
      rtk_gps_ubx.msg_idx++;
      if (rtk_gps_ubx.msg_idx >= rtk_gps_ubx.len) {
        rtk_gps_ubx.status++;
      }
      break;
    case GOT_PAYLOAD:
      if (c != rtk_gps_ubx.ck_a) {
        rtk_gps_ubx.error_last = RTK_GPS_UBX_ERR_CHECKSUM;
        goto error;
      }
      rtk_gps_ubx.status++;
      break;
    case GOT_CHECKSUM1:
      if (c != rtk_gps_ubx.ck_b) {
        rtk_gps_ubx.error_last = RTK_GPS_UBX_ERR_CHECKSUM;
        goto error;
      }
      rtk_gps_ubx.msg_available = true;
      goto restart;
      break;
    default:
      rtk_gps_ubx.error_last = RTK_GPS_UBX_ERR_UNEXPECTED;
      goto error;
  }
  return;
error:
  rtk_gps_ubx.error_cnt++;
restart:
  rtk_gps_ubx.status = UNINIT;
  return;
}

static void rtk_ubx_send_1byte(struct link_device *dev, uint8_t byte)
{
  dev->put_byte(dev->periph, 0, byte);
  rtk_gps_ubx.send_ck_a += byte;
  rtk_gps_ubx.send_ck_b += rtk_gps_ubx.send_ck_a;
}

void rtk_ubx_header(struct link_device *dev, uint8_t nav_id, uint8_t msg_id, uint16_t len)
{
  dev->put_byte(dev->periph, 0, UBX_SYNC1);
  dev->put_byte(dev->periph, 0, UBX_SYNC2);
  rtk_gps_ubx.send_ck_a = 0;
  rtk_gps_ubx.send_ck_b = 0;
  rtk_ubx_send_1byte(dev, nav_id);
  rtk_ubx_send_1byte(dev, msg_id);
  rtk_ubx_send_1byte(dev, (uint8_t)(len&0xFF));
  rtk_ubx_send_1byte(dev, (uint8_t)(len>>8));
}

void rtk_ubx_trailer(struct link_device *dev)
{
  dev->put_byte(dev->periph, 0, rtk_gps_ubx.send_ck_a);
  dev->put_byte(dev->periph, 0, rtk_gps_ubx.send_ck_b);
  dev->send_message(dev->periph, 0);
}

void rtk_ubx_send_bytes(struct link_device *dev, uint8_t len, uint8_t *bytes)
{
  int i;
  for (i = 0; i < len; i++) {
    rtk_ubx_send_1byte(dev, bytes[i]);
  }
}

void rtk_ubx_send_cfg_rst(struct link_device *dev, uint16_t bbr , uint8_t reset_mode)
{
#ifdef UBX_RTK_GPS_LINK
  UbxSend_CFG_RST(dev, bbr, reset_mode, 0x00);
#endif /* else less harmful for HITL */
}

#ifndef RTK_GPS_UBX_UCENTER
#define rtk_gps_ubx_ucenter_event() {}
#else
#include "modules/gps/rtk_gps_rtk_ubx_ucenter.h"
#endif

void rtk_gps_ubx_msg(void)
{
  //current timestamp
  uint32_t now_ts = get_sys_time_usec();

  rtk_gps_ubx.state.last_msg_ticks = sys_time.nb_sec_rem;
  rtk_gps_ubx.state.last_msg_time = sys_time.nb_sec;
  rtk_gps_ubx_read_message();
  rtk_gps_ubx_ucenter_event();
  if (rtk_gps_ubx.msg_class == UBX_NAV_ID &&
      (rtk_gps_ubx.msg_id == UBX_NAV_VELNED_ID ||
       (rtk_gps_ubx.msg_id == UBX_NAV_SOL_ID &&
        !bit_is_set(rtk_gps_ubx.state.valid_fields, GPS_VALID_VEL_NED_BIT)))) {
    if (rtk_gps_ubx.state.fix == GPS_FIX_3D) {
      rtk_gps_ubx.state.last_3dfix_ticks = sys_time.nb_sec_rem;
      rtk_gps_ubx.state.last_3dfix_time = sys_time.nb_sec;
    }
    AbiSendMsgGPS(RTK_GPS_UBX_ID, now_ts, &rtk_gps_ubx.state);
  }
  rtk_gps_ubx.msg_available = false;
}
#ifndef SITL
void tag_image(void){
  
  double now = get_sys_time_float();

  if((tag_next_rtk_msg) && (last_rtk_msg_time < now)) {
    
    log_lat = ((double)(rtk_gps_ubx.state.lla_pos.lat)/10000000.0);
    log_lon = ((double)(rtk_gps_ubx.state.lla_pos.lon)/10000000.0);
    log_alt = ((double)(rtk_gps_ubx.state.lla_pos.alt)/1000.0);
    log_vacc = ((double)(rtk_gps_ubx.state.vacc)/1000.0);
    log_hacc = ((double)(rtk_gps_ubx.state.hacc)/1000.0); 
    tag_next_rtk_msg = false;
    tagged_image = true;
  } 
	
}
void tag_image_log(void){
  
  if(tagged_image){

    int image_name_cnt = 0;

    if (pprzLogFile != -1){

       // sdLogWriteLog(pprzLogFile, "%.7f,%.7f\n",
       //             log_shot_command_time_stamp,log_target_rtk_time_stamp);

      while(image_name[image_name_cnt] != '&'){
        sdLogWriteLog(pprzLogFile, "%c",image_name[image_name_cnt]);
        image_name_cnt++;
      }

      sdLogWriteLog(pprzLogFile, ",%.7f,%.7f,%.2f,%.5f,%.5f,%.5f,%.2f,%.2f\n",
                  log_lat, log_lon, log_alt, log_phi, log_theta, log_psi, log_hacc, log_vacc);
      //DOWNLINK_SEND_DEBUG(DefaultChannel, DefaultDevice, 1, err);
    }
    tagged_image = false;
   //LED_ON(4);
  }
 //LED_ON(3);
}
#else

#endif
