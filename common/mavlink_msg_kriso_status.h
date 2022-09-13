#pragma once
// MESSAGE KRISO_STATUS PACKING

#define MAVLINK_MSG_ID_KRISO_STATUS 9100


typedef struct __mavlink_kriso_status_t {
 uint64_t time_usec; /*< [us] Timestamp (synced to UNIX time or since system boot).*/
 double nav_longitude; /*< [degree] degree */
 double nav_latitude; /*< [degree] degree */
 float nav_roll; /*< [degree] degree roll*/
 float nav_pitch; /*< [degree] degree roll*/
 float nav_yaw; /*< [degree] degree roll*/
 float nav_cog; /*< [degree] degree COG*/
 float nav_sog; /*< [knots] knots degree SOG*/
 float nav_uspd; /*< [knots] longitudinal speed*/
 float nav_vspd; /*< [knots] traversal speed*/
 float nav_wspd; /*< [knots] vertical speed*/
 float nav_heave; /*< [m] Heave*/
 float nav_gpstime; /*< [time] hhmmss time*/
 float wea_airtem; /*< [degree] air temperature*/
 float wea_wattem; /*< [degree] water temperature*/
 float wea_press; /*< [bar] pressure*/
 float wea_relhum; /*< [%] relative humidity*/
 float wea_dewpt; /*< [degree] dew */
 float wea_windirt; /*< [degree] wind direction*/
 float wea_winspdt; /*< [degree] wind velocity*/
 float wea_windirr; /*< [degree] wind direction relative*/
 float wea_watspdr; /*< [knots] water velocity*/
 float wea_watdir; /*< [degree] water direction*/
 float wea_watspd; /*< [knots] water velocity*/
 float wea_visibiran; /*< [m] visual sight*/
 uint8_t nav_mode; /*<  Navigation mode*/
} mavlink_kriso_status_t;

#define MAVLINK_MSG_ID_KRISO_STATUS_LEN 113
#define MAVLINK_MSG_ID_KRISO_STATUS_MIN_LEN 113
#define MAVLINK_MSG_ID_9100_LEN 113
#define MAVLINK_MSG_ID_9100_MIN_LEN 113

#define MAVLINK_MSG_ID_KRISO_STATUS_CRC 234
#define MAVLINK_MSG_ID_9100_CRC 234



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_KRISO_STATUS { \
    9100, \
    "KRISO_STATUS", \
    26, \
    {  { "time_usec", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_kriso_status_t, time_usec) }, \
         { "nav_mode", NULL, MAVLINK_TYPE_UINT8_T, 0, 112, offsetof(mavlink_kriso_status_t, nav_mode) }, \
         { "nav_roll", NULL, MAVLINK_TYPE_FLOAT, 0, 24, offsetof(mavlink_kriso_status_t, nav_roll) }, \
         { "nav_pitch", NULL, MAVLINK_TYPE_FLOAT, 0, 28, offsetof(mavlink_kriso_status_t, nav_pitch) }, \
         { "nav_yaw", NULL, MAVLINK_TYPE_FLOAT, 0, 32, offsetof(mavlink_kriso_status_t, nav_yaw) }, \
         { "nav_cog", NULL, MAVLINK_TYPE_FLOAT, 0, 36, offsetof(mavlink_kriso_status_t, nav_cog) }, \
         { "nav_sog", NULL, MAVLINK_TYPE_FLOAT, 0, 40, offsetof(mavlink_kriso_status_t, nav_sog) }, \
         { "nav_uspd", NULL, MAVLINK_TYPE_FLOAT, 0, 44, offsetof(mavlink_kriso_status_t, nav_uspd) }, \
         { "nav_vspd", NULL, MAVLINK_TYPE_FLOAT, 0, 48, offsetof(mavlink_kriso_status_t, nav_vspd) }, \
         { "nav_wspd", NULL, MAVLINK_TYPE_FLOAT, 0, 52, offsetof(mavlink_kriso_status_t, nav_wspd) }, \
         { "nav_longitude", NULL, MAVLINK_TYPE_DOUBLE, 0, 8, offsetof(mavlink_kriso_status_t, nav_longitude) }, \
         { "nav_latitude", NULL, MAVLINK_TYPE_DOUBLE, 0, 16, offsetof(mavlink_kriso_status_t, nav_latitude) }, \
         { "nav_heave", NULL, MAVLINK_TYPE_FLOAT, 0, 56, offsetof(mavlink_kriso_status_t, nav_heave) }, \
         { "nav_gpstime", NULL, MAVLINK_TYPE_FLOAT, 0, 60, offsetof(mavlink_kriso_status_t, nav_gpstime) }, \
         { "wea_airtem", NULL, MAVLINK_TYPE_FLOAT, 0, 64, offsetof(mavlink_kriso_status_t, wea_airtem) }, \
         { "wea_wattem", NULL, MAVLINK_TYPE_FLOAT, 0, 68, offsetof(mavlink_kriso_status_t, wea_wattem) }, \
         { "wea_press", NULL, MAVLINK_TYPE_FLOAT, 0, 72, offsetof(mavlink_kriso_status_t, wea_press) }, \
         { "wea_relhum", NULL, MAVLINK_TYPE_FLOAT, 0, 76, offsetof(mavlink_kriso_status_t, wea_relhum) }, \
         { "wea_dewpt", NULL, MAVLINK_TYPE_FLOAT, 0, 80, offsetof(mavlink_kriso_status_t, wea_dewpt) }, \
         { "wea_windirt", NULL, MAVLINK_TYPE_FLOAT, 0, 84, offsetof(mavlink_kriso_status_t, wea_windirt) }, \
         { "wea_winspdt", NULL, MAVLINK_TYPE_FLOAT, 0, 88, offsetof(mavlink_kriso_status_t, wea_winspdt) }, \
         { "wea_windirr", NULL, MAVLINK_TYPE_FLOAT, 0, 92, offsetof(mavlink_kriso_status_t, wea_windirr) }, \
         { "wea_watspdr", NULL, MAVLINK_TYPE_FLOAT, 0, 96, offsetof(mavlink_kriso_status_t, wea_watspdr) }, \
         { "wea_watdir", NULL, MAVLINK_TYPE_FLOAT, 0, 100, offsetof(mavlink_kriso_status_t, wea_watdir) }, \
         { "wea_watspd", NULL, MAVLINK_TYPE_FLOAT, 0, 104, offsetof(mavlink_kriso_status_t, wea_watspd) }, \
         { "wea_visibiran", NULL, MAVLINK_TYPE_FLOAT, 0, 108, offsetof(mavlink_kriso_status_t, wea_visibiran) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_KRISO_STATUS { \
    "KRISO_STATUS", \
    26, \
    {  { "time_usec", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_kriso_status_t, time_usec) }, \
         { "nav_mode", NULL, MAVLINK_TYPE_UINT8_T, 0, 112, offsetof(mavlink_kriso_status_t, nav_mode) }, \
         { "nav_roll", NULL, MAVLINK_TYPE_FLOAT, 0, 24, offsetof(mavlink_kriso_status_t, nav_roll) }, \
         { "nav_pitch", NULL, MAVLINK_TYPE_FLOAT, 0, 28, offsetof(mavlink_kriso_status_t, nav_pitch) }, \
         { "nav_yaw", NULL, MAVLINK_TYPE_FLOAT, 0, 32, offsetof(mavlink_kriso_status_t, nav_yaw) }, \
         { "nav_cog", NULL, MAVLINK_TYPE_FLOAT, 0, 36, offsetof(mavlink_kriso_status_t, nav_cog) }, \
         { "nav_sog", NULL, MAVLINK_TYPE_FLOAT, 0, 40, offsetof(mavlink_kriso_status_t, nav_sog) }, \
         { "nav_uspd", NULL, MAVLINK_TYPE_FLOAT, 0, 44, offsetof(mavlink_kriso_status_t, nav_uspd) }, \
         { "nav_vspd", NULL, MAVLINK_TYPE_FLOAT, 0, 48, offsetof(mavlink_kriso_status_t, nav_vspd) }, \
         { "nav_wspd", NULL, MAVLINK_TYPE_FLOAT, 0, 52, offsetof(mavlink_kriso_status_t, nav_wspd) }, \
         { "nav_longitude", NULL, MAVLINK_TYPE_DOUBLE, 0, 8, offsetof(mavlink_kriso_status_t, nav_longitude) }, \
         { "nav_latitude", NULL, MAVLINK_TYPE_DOUBLE, 0, 16, offsetof(mavlink_kriso_status_t, nav_latitude) }, \
         { "nav_heave", NULL, MAVLINK_TYPE_FLOAT, 0, 56, offsetof(mavlink_kriso_status_t, nav_heave) }, \
         { "nav_gpstime", NULL, MAVLINK_TYPE_FLOAT, 0, 60, offsetof(mavlink_kriso_status_t, nav_gpstime) }, \
         { "wea_airtem", NULL, MAVLINK_TYPE_FLOAT, 0, 64, offsetof(mavlink_kriso_status_t, wea_airtem) }, \
         { "wea_wattem", NULL, MAVLINK_TYPE_FLOAT, 0, 68, offsetof(mavlink_kriso_status_t, wea_wattem) }, \
         { "wea_press", NULL, MAVLINK_TYPE_FLOAT, 0, 72, offsetof(mavlink_kriso_status_t, wea_press) }, \
         { "wea_relhum", NULL, MAVLINK_TYPE_FLOAT, 0, 76, offsetof(mavlink_kriso_status_t, wea_relhum) }, \
         { "wea_dewpt", NULL, MAVLINK_TYPE_FLOAT, 0, 80, offsetof(mavlink_kriso_status_t, wea_dewpt) }, \
         { "wea_windirt", NULL, MAVLINK_TYPE_FLOAT, 0, 84, offsetof(mavlink_kriso_status_t, wea_windirt) }, \
         { "wea_winspdt", NULL, MAVLINK_TYPE_FLOAT, 0, 88, offsetof(mavlink_kriso_status_t, wea_winspdt) }, \
         { "wea_windirr", NULL, MAVLINK_TYPE_FLOAT, 0, 92, offsetof(mavlink_kriso_status_t, wea_windirr) }, \
         { "wea_watspdr", NULL, MAVLINK_TYPE_FLOAT, 0, 96, offsetof(mavlink_kriso_status_t, wea_watspdr) }, \
         { "wea_watdir", NULL, MAVLINK_TYPE_FLOAT, 0, 100, offsetof(mavlink_kriso_status_t, wea_watdir) }, \
         { "wea_watspd", NULL, MAVLINK_TYPE_FLOAT, 0, 104, offsetof(mavlink_kriso_status_t, wea_watspd) }, \
         { "wea_visibiran", NULL, MAVLINK_TYPE_FLOAT, 0, 108, offsetof(mavlink_kriso_status_t, wea_visibiran) }, \
         } \
}
#endif

/**
 * @brief Pack a kriso_status message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param time_usec [us] Timestamp (synced to UNIX time or since system boot).
 * @param nav_mode  Navigation mode
 * @param nav_roll [degree] degree roll
 * @param nav_pitch [degree] degree roll
 * @param nav_yaw [degree] degree roll
 * @param nav_cog [degree] degree COG
 * @param nav_sog [knots] knots degree SOG
 * @param nav_uspd [knots] longitudinal speed
 * @param nav_vspd [knots] traversal speed
 * @param nav_wspd [knots] vertical speed
 * @param nav_longitude [degree] degree 
 * @param nav_latitude [degree] degree 
 * @param nav_heave [m] Heave
 * @param nav_gpstime [time] hhmmss time
 * @param wea_airtem [degree] air temperature
 * @param wea_wattem [degree] water temperature
 * @param wea_press [bar] pressure
 * @param wea_relhum [%] relative humidity
 * @param wea_dewpt [degree] dew 
 * @param wea_windirt [degree] wind direction
 * @param wea_winspdt [degree] wind velocity
 * @param wea_windirr [degree] wind direction relative
 * @param wea_watspdr [knots] water velocity
 * @param wea_watdir [degree] water direction
 * @param wea_watspd [knots] water velocity
 * @param wea_visibiran [m] visual sight
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_kriso_status_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint64_t time_usec, uint8_t nav_mode, float nav_roll, float nav_pitch, float nav_yaw, float nav_cog, float nav_sog, float nav_uspd, float nav_vspd, float nav_wspd, double nav_longitude, double nav_latitude, float nav_heave, float nav_gpstime, float wea_airtem, float wea_wattem, float wea_press, float wea_relhum, float wea_dewpt, float wea_windirt, float wea_winspdt, float wea_windirr, float wea_watspdr, float wea_watdir, float wea_watspd, float wea_visibiran)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_KRISO_STATUS_LEN];
    _mav_put_uint64_t(buf, 0, time_usec);
    _mav_put_double(buf, 8, nav_longitude);
    _mav_put_double(buf, 16, nav_latitude);
    _mav_put_float(buf, 24, nav_roll);
    _mav_put_float(buf, 28, nav_pitch);
    _mav_put_float(buf, 32, nav_yaw);
    _mav_put_float(buf, 36, nav_cog);
    _mav_put_float(buf, 40, nav_sog);
    _mav_put_float(buf, 44, nav_uspd);
    _mav_put_float(buf, 48, nav_vspd);
    _mav_put_float(buf, 52, nav_wspd);
    _mav_put_float(buf, 56, nav_heave);
    _mav_put_float(buf, 60, nav_gpstime);
    _mav_put_float(buf, 64, wea_airtem);
    _mav_put_float(buf, 68, wea_wattem);
    _mav_put_float(buf, 72, wea_press);
    _mav_put_float(buf, 76, wea_relhum);
    _mav_put_float(buf, 80, wea_dewpt);
    _mav_put_float(buf, 84, wea_windirt);
    _mav_put_float(buf, 88, wea_winspdt);
    _mav_put_float(buf, 92, wea_windirr);
    _mav_put_float(buf, 96, wea_watspdr);
    _mav_put_float(buf, 100, wea_watdir);
    _mav_put_float(buf, 104, wea_watspd);
    _mav_put_float(buf, 108, wea_visibiran);
    _mav_put_uint8_t(buf, 112, nav_mode);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_KRISO_STATUS_LEN);
#else
    mavlink_kriso_status_t packet;
    packet.time_usec = time_usec;
    packet.nav_longitude = nav_longitude;
    packet.nav_latitude = nav_latitude;
    packet.nav_roll = nav_roll;
    packet.nav_pitch = nav_pitch;
    packet.nav_yaw = nav_yaw;
    packet.nav_cog = nav_cog;
    packet.nav_sog = nav_sog;
    packet.nav_uspd = nav_uspd;
    packet.nav_vspd = nav_vspd;
    packet.nav_wspd = nav_wspd;
    packet.nav_heave = nav_heave;
    packet.nav_gpstime = nav_gpstime;
    packet.wea_airtem = wea_airtem;
    packet.wea_wattem = wea_wattem;
    packet.wea_press = wea_press;
    packet.wea_relhum = wea_relhum;
    packet.wea_dewpt = wea_dewpt;
    packet.wea_windirt = wea_windirt;
    packet.wea_winspdt = wea_winspdt;
    packet.wea_windirr = wea_windirr;
    packet.wea_watspdr = wea_watspdr;
    packet.wea_watdir = wea_watdir;
    packet.wea_watspd = wea_watspd;
    packet.wea_visibiran = wea_visibiran;
    packet.nav_mode = nav_mode;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_KRISO_STATUS_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_KRISO_STATUS;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_KRISO_STATUS_MIN_LEN, MAVLINK_MSG_ID_KRISO_STATUS_LEN, MAVLINK_MSG_ID_KRISO_STATUS_CRC);
}

/**
 * @brief Pack a kriso_status message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param time_usec [us] Timestamp (synced to UNIX time or since system boot).
 * @param nav_mode  Navigation mode
 * @param nav_roll [degree] degree roll
 * @param nav_pitch [degree] degree roll
 * @param nav_yaw [degree] degree roll
 * @param nav_cog [degree] degree COG
 * @param nav_sog [knots] knots degree SOG
 * @param nav_uspd [knots] longitudinal speed
 * @param nav_vspd [knots] traversal speed
 * @param nav_wspd [knots] vertical speed
 * @param nav_longitude [degree] degree 
 * @param nav_latitude [degree] degree 
 * @param nav_heave [m] Heave
 * @param nav_gpstime [time] hhmmss time
 * @param wea_airtem [degree] air temperature
 * @param wea_wattem [degree] water temperature
 * @param wea_press [bar] pressure
 * @param wea_relhum [%] relative humidity
 * @param wea_dewpt [degree] dew 
 * @param wea_windirt [degree] wind direction
 * @param wea_winspdt [degree] wind velocity
 * @param wea_windirr [degree] wind direction relative
 * @param wea_watspdr [knots] water velocity
 * @param wea_watdir [degree] water direction
 * @param wea_watspd [knots] water velocity
 * @param wea_visibiran [m] visual sight
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_kriso_status_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint64_t time_usec,uint8_t nav_mode,float nav_roll,float nav_pitch,float nav_yaw,float nav_cog,float nav_sog,float nav_uspd,float nav_vspd,float nav_wspd,double nav_longitude,double nav_latitude,float nav_heave,float nav_gpstime,float wea_airtem,float wea_wattem,float wea_press,float wea_relhum,float wea_dewpt,float wea_windirt,float wea_winspdt,float wea_windirr,float wea_watspdr,float wea_watdir,float wea_watspd,float wea_visibiran)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_KRISO_STATUS_LEN];
    _mav_put_uint64_t(buf, 0, time_usec);
    _mav_put_double(buf, 8, nav_longitude);
    _mav_put_double(buf, 16, nav_latitude);
    _mav_put_float(buf, 24, nav_roll);
    _mav_put_float(buf, 28, nav_pitch);
    _mav_put_float(buf, 32, nav_yaw);
    _mav_put_float(buf, 36, nav_cog);
    _mav_put_float(buf, 40, nav_sog);
    _mav_put_float(buf, 44, nav_uspd);
    _mav_put_float(buf, 48, nav_vspd);
    _mav_put_float(buf, 52, nav_wspd);
    _mav_put_float(buf, 56, nav_heave);
    _mav_put_float(buf, 60, nav_gpstime);
    _mav_put_float(buf, 64, wea_airtem);
    _mav_put_float(buf, 68, wea_wattem);
    _mav_put_float(buf, 72, wea_press);
    _mav_put_float(buf, 76, wea_relhum);
    _mav_put_float(buf, 80, wea_dewpt);
    _mav_put_float(buf, 84, wea_windirt);
    _mav_put_float(buf, 88, wea_winspdt);
    _mav_put_float(buf, 92, wea_windirr);
    _mav_put_float(buf, 96, wea_watspdr);
    _mav_put_float(buf, 100, wea_watdir);
    _mav_put_float(buf, 104, wea_watspd);
    _mav_put_float(buf, 108, wea_visibiran);
    _mav_put_uint8_t(buf, 112, nav_mode);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_KRISO_STATUS_LEN);
#else
    mavlink_kriso_status_t packet;
    packet.time_usec = time_usec;
    packet.nav_longitude = nav_longitude;
    packet.nav_latitude = nav_latitude;
    packet.nav_roll = nav_roll;
    packet.nav_pitch = nav_pitch;
    packet.nav_yaw = nav_yaw;
    packet.nav_cog = nav_cog;
    packet.nav_sog = nav_sog;
    packet.nav_uspd = nav_uspd;
    packet.nav_vspd = nav_vspd;
    packet.nav_wspd = nav_wspd;
    packet.nav_heave = nav_heave;
    packet.nav_gpstime = nav_gpstime;
    packet.wea_airtem = wea_airtem;
    packet.wea_wattem = wea_wattem;
    packet.wea_press = wea_press;
    packet.wea_relhum = wea_relhum;
    packet.wea_dewpt = wea_dewpt;
    packet.wea_windirt = wea_windirt;
    packet.wea_winspdt = wea_winspdt;
    packet.wea_windirr = wea_windirr;
    packet.wea_watspdr = wea_watspdr;
    packet.wea_watdir = wea_watdir;
    packet.wea_watspd = wea_watspd;
    packet.wea_visibiran = wea_visibiran;
    packet.nav_mode = nav_mode;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_KRISO_STATUS_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_KRISO_STATUS;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_KRISO_STATUS_MIN_LEN, MAVLINK_MSG_ID_KRISO_STATUS_LEN, MAVLINK_MSG_ID_KRISO_STATUS_CRC);
}

/**
 * @brief Encode a kriso_status struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param kriso_status C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_kriso_status_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_kriso_status_t* kriso_status)
{
    return mavlink_msg_kriso_status_pack(system_id, component_id, msg, kriso_status->time_usec, kriso_status->nav_mode, kriso_status->nav_roll, kriso_status->nav_pitch, kriso_status->nav_yaw, kriso_status->nav_cog, kriso_status->nav_sog, kriso_status->nav_uspd, kriso_status->nav_vspd, kriso_status->nav_wspd, kriso_status->nav_longitude, kriso_status->nav_latitude, kriso_status->nav_heave, kriso_status->nav_gpstime, kriso_status->wea_airtem, kriso_status->wea_wattem, kriso_status->wea_press, kriso_status->wea_relhum, kriso_status->wea_dewpt, kriso_status->wea_windirt, kriso_status->wea_winspdt, kriso_status->wea_windirr, kriso_status->wea_watspdr, kriso_status->wea_watdir, kriso_status->wea_watspd, kriso_status->wea_visibiran);
}

/**
 * @brief Encode a kriso_status struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param kriso_status C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_kriso_status_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_kriso_status_t* kriso_status)
{
    return mavlink_msg_kriso_status_pack_chan(system_id, component_id, chan, msg, kriso_status->time_usec, kriso_status->nav_mode, kriso_status->nav_roll, kriso_status->nav_pitch, kriso_status->nav_yaw, kriso_status->nav_cog, kriso_status->nav_sog, kriso_status->nav_uspd, kriso_status->nav_vspd, kriso_status->nav_wspd, kriso_status->nav_longitude, kriso_status->nav_latitude, kriso_status->nav_heave, kriso_status->nav_gpstime, kriso_status->wea_airtem, kriso_status->wea_wattem, kriso_status->wea_press, kriso_status->wea_relhum, kriso_status->wea_dewpt, kriso_status->wea_windirt, kriso_status->wea_winspdt, kriso_status->wea_windirr, kriso_status->wea_watspdr, kriso_status->wea_watdir, kriso_status->wea_watspd, kriso_status->wea_visibiran);
}

/**
 * @brief Send a kriso_status message
 * @param chan MAVLink channel to send the message
 *
 * @param time_usec [us] Timestamp (synced to UNIX time or since system boot).
 * @param nav_mode  Navigation mode
 * @param nav_roll [degree] degree roll
 * @param nav_pitch [degree] degree roll
 * @param nav_yaw [degree] degree roll
 * @param nav_cog [degree] degree COG
 * @param nav_sog [knots] knots degree SOG
 * @param nav_uspd [knots] longitudinal speed
 * @param nav_vspd [knots] traversal speed
 * @param nav_wspd [knots] vertical speed
 * @param nav_longitude [degree] degree 
 * @param nav_latitude [degree] degree 
 * @param nav_heave [m] Heave
 * @param nav_gpstime [time] hhmmss time
 * @param wea_airtem [degree] air temperature
 * @param wea_wattem [degree] water temperature
 * @param wea_press [bar] pressure
 * @param wea_relhum [%] relative humidity
 * @param wea_dewpt [degree] dew 
 * @param wea_windirt [degree] wind direction
 * @param wea_winspdt [degree] wind velocity
 * @param wea_windirr [degree] wind direction relative
 * @param wea_watspdr [knots] water velocity
 * @param wea_watdir [degree] water direction
 * @param wea_watspd [knots] water velocity
 * @param wea_visibiran [m] visual sight
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_kriso_status_send(mavlink_channel_t chan, uint64_t time_usec, uint8_t nav_mode, float nav_roll, float nav_pitch, float nav_yaw, float nav_cog, float nav_sog, float nav_uspd, float nav_vspd, float nav_wspd, double nav_longitude, double nav_latitude, float nav_heave, float nav_gpstime, float wea_airtem, float wea_wattem, float wea_press, float wea_relhum, float wea_dewpt, float wea_windirt, float wea_winspdt, float wea_windirr, float wea_watspdr, float wea_watdir, float wea_watspd, float wea_visibiran)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_KRISO_STATUS_LEN];
    _mav_put_uint64_t(buf, 0, time_usec);
    _mav_put_double(buf, 8, nav_longitude);
    _mav_put_double(buf, 16, nav_latitude);
    _mav_put_float(buf, 24, nav_roll);
    _mav_put_float(buf, 28, nav_pitch);
    _mav_put_float(buf, 32, nav_yaw);
    _mav_put_float(buf, 36, nav_cog);
    _mav_put_float(buf, 40, nav_sog);
    _mav_put_float(buf, 44, nav_uspd);
    _mav_put_float(buf, 48, nav_vspd);
    _mav_put_float(buf, 52, nav_wspd);
    _mav_put_float(buf, 56, nav_heave);
    _mav_put_float(buf, 60, nav_gpstime);
    _mav_put_float(buf, 64, wea_airtem);
    _mav_put_float(buf, 68, wea_wattem);
    _mav_put_float(buf, 72, wea_press);
    _mav_put_float(buf, 76, wea_relhum);
    _mav_put_float(buf, 80, wea_dewpt);
    _mav_put_float(buf, 84, wea_windirt);
    _mav_put_float(buf, 88, wea_winspdt);
    _mav_put_float(buf, 92, wea_windirr);
    _mav_put_float(buf, 96, wea_watspdr);
    _mav_put_float(buf, 100, wea_watdir);
    _mav_put_float(buf, 104, wea_watspd);
    _mav_put_float(buf, 108, wea_visibiran);
    _mav_put_uint8_t(buf, 112, nav_mode);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_KRISO_STATUS, buf, MAVLINK_MSG_ID_KRISO_STATUS_MIN_LEN, MAVLINK_MSG_ID_KRISO_STATUS_LEN, MAVLINK_MSG_ID_KRISO_STATUS_CRC);
#else
    mavlink_kriso_status_t packet;
    packet.time_usec = time_usec;
    packet.nav_longitude = nav_longitude;
    packet.nav_latitude = nav_latitude;
    packet.nav_roll = nav_roll;
    packet.nav_pitch = nav_pitch;
    packet.nav_yaw = nav_yaw;
    packet.nav_cog = nav_cog;
    packet.nav_sog = nav_sog;
    packet.nav_uspd = nav_uspd;
    packet.nav_vspd = nav_vspd;
    packet.nav_wspd = nav_wspd;
    packet.nav_heave = nav_heave;
    packet.nav_gpstime = nav_gpstime;
    packet.wea_airtem = wea_airtem;
    packet.wea_wattem = wea_wattem;
    packet.wea_press = wea_press;
    packet.wea_relhum = wea_relhum;
    packet.wea_dewpt = wea_dewpt;
    packet.wea_windirt = wea_windirt;
    packet.wea_winspdt = wea_winspdt;
    packet.wea_windirr = wea_windirr;
    packet.wea_watspdr = wea_watspdr;
    packet.wea_watdir = wea_watdir;
    packet.wea_watspd = wea_watspd;
    packet.wea_visibiran = wea_visibiran;
    packet.nav_mode = nav_mode;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_KRISO_STATUS, (const char *)&packet, MAVLINK_MSG_ID_KRISO_STATUS_MIN_LEN, MAVLINK_MSG_ID_KRISO_STATUS_LEN, MAVLINK_MSG_ID_KRISO_STATUS_CRC);
#endif
}

/**
 * @brief Send a kriso_status message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_kriso_status_send_struct(mavlink_channel_t chan, const mavlink_kriso_status_t* kriso_status)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_kriso_status_send(chan, kriso_status->time_usec, kriso_status->nav_mode, kriso_status->nav_roll, kriso_status->nav_pitch, kriso_status->nav_yaw, kriso_status->nav_cog, kriso_status->nav_sog, kriso_status->nav_uspd, kriso_status->nav_vspd, kriso_status->nav_wspd, kriso_status->nav_longitude, kriso_status->nav_latitude, kriso_status->nav_heave, kriso_status->nav_gpstime, kriso_status->wea_airtem, kriso_status->wea_wattem, kriso_status->wea_press, kriso_status->wea_relhum, kriso_status->wea_dewpt, kriso_status->wea_windirt, kriso_status->wea_winspdt, kriso_status->wea_windirr, kriso_status->wea_watspdr, kriso_status->wea_watdir, kriso_status->wea_watspd, kriso_status->wea_visibiran);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_KRISO_STATUS, (const char *)kriso_status, MAVLINK_MSG_ID_KRISO_STATUS_MIN_LEN, MAVLINK_MSG_ID_KRISO_STATUS_LEN, MAVLINK_MSG_ID_KRISO_STATUS_CRC);
#endif
}

#if MAVLINK_MSG_ID_KRISO_STATUS_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_kriso_status_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint64_t time_usec, uint8_t nav_mode, float nav_roll, float nav_pitch, float nav_yaw, float nav_cog, float nav_sog, float nav_uspd, float nav_vspd, float nav_wspd, double nav_longitude, double nav_latitude, float nav_heave, float nav_gpstime, float wea_airtem, float wea_wattem, float wea_press, float wea_relhum, float wea_dewpt, float wea_windirt, float wea_winspdt, float wea_windirr, float wea_watspdr, float wea_watdir, float wea_watspd, float wea_visibiran)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint64_t(buf, 0, time_usec);
    _mav_put_double(buf, 8, nav_longitude);
    _mav_put_double(buf, 16, nav_latitude);
    _mav_put_float(buf, 24, nav_roll);
    _mav_put_float(buf, 28, nav_pitch);
    _mav_put_float(buf, 32, nav_yaw);
    _mav_put_float(buf, 36, nav_cog);
    _mav_put_float(buf, 40, nav_sog);
    _mav_put_float(buf, 44, nav_uspd);
    _mav_put_float(buf, 48, nav_vspd);
    _mav_put_float(buf, 52, nav_wspd);
    _mav_put_float(buf, 56, nav_heave);
    _mav_put_float(buf, 60, nav_gpstime);
    _mav_put_float(buf, 64, wea_airtem);
    _mav_put_float(buf, 68, wea_wattem);
    _mav_put_float(buf, 72, wea_press);
    _mav_put_float(buf, 76, wea_relhum);
    _mav_put_float(buf, 80, wea_dewpt);
    _mav_put_float(buf, 84, wea_windirt);
    _mav_put_float(buf, 88, wea_winspdt);
    _mav_put_float(buf, 92, wea_windirr);
    _mav_put_float(buf, 96, wea_watspdr);
    _mav_put_float(buf, 100, wea_watdir);
    _mav_put_float(buf, 104, wea_watspd);
    _mav_put_float(buf, 108, wea_visibiran);
    _mav_put_uint8_t(buf, 112, nav_mode);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_KRISO_STATUS, buf, MAVLINK_MSG_ID_KRISO_STATUS_MIN_LEN, MAVLINK_MSG_ID_KRISO_STATUS_LEN, MAVLINK_MSG_ID_KRISO_STATUS_CRC);
#else
    mavlink_kriso_status_t *packet = (mavlink_kriso_status_t *)msgbuf;
    packet->time_usec = time_usec;
    packet->nav_longitude = nav_longitude;
    packet->nav_latitude = nav_latitude;
    packet->nav_roll = nav_roll;
    packet->nav_pitch = nav_pitch;
    packet->nav_yaw = nav_yaw;
    packet->nav_cog = nav_cog;
    packet->nav_sog = nav_sog;
    packet->nav_uspd = nav_uspd;
    packet->nav_vspd = nav_vspd;
    packet->nav_wspd = nav_wspd;
    packet->nav_heave = nav_heave;
    packet->nav_gpstime = nav_gpstime;
    packet->wea_airtem = wea_airtem;
    packet->wea_wattem = wea_wattem;
    packet->wea_press = wea_press;
    packet->wea_relhum = wea_relhum;
    packet->wea_dewpt = wea_dewpt;
    packet->wea_windirt = wea_windirt;
    packet->wea_winspdt = wea_winspdt;
    packet->wea_windirr = wea_windirr;
    packet->wea_watspdr = wea_watspdr;
    packet->wea_watdir = wea_watdir;
    packet->wea_watspd = wea_watspd;
    packet->wea_visibiran = wea_visibiran;
    packet->nav_mode = nav_mode;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_KRISO_STATUS, (const char *)packet, MAVLINK_MSG_ID_KRISO_STATUS_MIN_LEN, MAVLINK_MSG_ID_KRISO_STATUS_LEN, MAVLINK_MSG_ID_KRISO_STATUS_CRC);
#endif
}
#endif

#endif

// MESSAGE KRISO_STATUS UNPACKING


/**
 * @brief Get field time_usec from kriso_status message
 *
 * @return [us] Timestamp (synced to UNIX time or since system boot).
 */
static inline uint64_t mavlink_msg_kriso_status_get_time_usec(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint64_t(msg,  0);
}

/**
 * @brief Get field nav_mode from kriso_status message
 *
 * @return  Navigation mode
 */
static inline uint8_t mavlink_msg_kriso_status_get_nav_mode(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  112);
}

/**
 * @brief Get field nav_roll from kriso_status message
 *
 * @return [degree] degree roll
 */
static inline float mavlink_msg_kriso_status_get_nav_roll(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  24);
}

/**
 * @brief Get field nav_pitch from kriso_status message
 *
 * @return [degree] degree roll
 */
static inline float mavlink_msg_kriso_status_get_nav_pitch(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  28);
}

/**
 * @brief Get field nav_yaw from kriso_status message
 *
 * @return [degree] degree roll
 */
static inline float mavlink_msg_kriso_status_get_nav_yaw(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  32);
}

/**
 * @brief Get field nav_cog from kriso_status message
 *
 * @return [degree] degree COG
 */
static inline float mavlink_msg_kriso_status_get_nav_cog(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  36);
}

/**
 * @brief Get field nav_sog from kriso_status message
 *
 * @return [knots] knots degree SOG
 */
static inline float mavlink_msg_kriso_status_get_nav_sog(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  40);
}

/**
 * @brief Get field nav_uspd from kriso_status message
 *
 * @return [knots] longitudinal speed
 */
static inline float mavlink_msg_kriso_status_get_nav_uspd(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  44);
}

/**
 * @brief Get field nav_vspd from kriso_status message
 *
 * @return [knots] traversal speed
 */
static inline float mavlink_msg_kriso_status_get_nav_vspd(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  48);
}

/**
 * @brief Get field nav_wspd from kriso_status message
 *
 * @return [knots] vertical speed
 */
static inline float mavlink_msg_kriso_status_get_nav_wspd(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  52);
}

/**
 * @brief Get field nav_longitude from kriso_status message
 *
 * @return [degree] degree 
 */
static inline double mavlink_msg_kriso_status_get_nav_longitude(const mavlink_message_t* msg)
{
    return _MAV_RETURN_double(msg,  8);
}

/**
 * @brief Get field nav_latitude from kriso_status message
 *
 * @return [degree] degree 
 */
static inline double mavlink_msg_kriso_status_get_nav_latitude(const mavlink_message_t* msg)
{
    return _MAV_RETURN_double(msg,  16);
}

/**
 * @brief Get field nav_heave from kriso_status message
 *
 * @return [m] Heave
 */
static inline float mavlink_msg_kriso_status_get_nav_heave(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  56);
}

/**
 * @brief Get field nav_gpstime from kriso_status message
 *
 * @return [time] hhmmss time
 */
static inline float mavlink_msg_kriso_status_get_nav_gpstime(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  60);
}

/**
 * @brief Get field wea_airtem from kriso_status message
 *
 * @return [degree] air temperature
 */
static inline float mavlink_msg_kriso_status_get_wea_airtem(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  64);
}

/**
 * @brief Get field wea_wattem from kriso_status message
 *
 * @return [degree] water temperature
 */
static inline float mavlink_msg_kriso_status_get_wea_wattem(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  68);
}

/**
 * @brief Get field wea_press from kriso_status message
 *
 * @return [bar] pressure
 */
static inline float mavlink_msg_kriso_status_get_wea_press(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  72);
}

/**
 * @brief Get field wea_relhum from kriso_status message
 *
 * @return [%] relative humidity
 */
static inline float mavlink_msg_kriso_status_get_wea_relhum(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  76);
}

/**
 * @brief Get field wea_dewpt from kriso_status message
 *
 * @return [degree] dew 
 */
static inline float mavlink_msg_kriso_status_get_wea_dewpt(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  80);
}

/**
 * @brief Get field wea_windirt from kriso_status message
 *
 * @return [degree] wind direction
 */
static inline float mavlink_msg_kriso_status_get_wea_windirt(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  84);
}

/**
 * @brief Get field wea_winspdt from kriso_status message
 *
 * @return [degree] wind velocity
 */
static inline float mavlink_msg_kriso_status_get_wea_winspdt(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  88);
}

/**
 * @brief Get field wea_windirr from kriso_status message
 *
 * @return [degree] wind direction relative
 */
static inline float mavlink_msg_kriso_status_get_wea_windirr(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  92);
}

/**
 * @brief Get field wea_watspdr from kriso_status message
 *
 * @return [knots] water velocity
 */
static inline float mavlink_msg_kriso_status_get_wea_watspdr(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  96);
}

/**
 * @brief Get field wea_watdir from kriso_status message
 *
 * @return [degree] water direction
 */
static inline float mavlink_msg_kriso_status_get_wea_watdir(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  100);
}

/**
 * @brief Get field wea_watspd from kriso_status message
 *
 * @return [knots] water velocity
 */
static inline float mavlink_msg_kriso_status_get_wea_watspd(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  104);
}

/**
 * @brief Get field wea_visibiran from kriso_status message
 *
 * @return [m] visual sight
 */
static inline float mavlink_msg_kriso_status_get_wea_visibiran(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  108);
}

/**
 * @brief Decode a kriso_status message into a struct
 *
 * @param msg The message to decode
 * @param kriso_status C-struct to decode the message contents into
 */
static inline void mavlink_msg_kriso_status_decode(const mavlink_message_t* msg, mavlink_kriso_status_t* kriso_status)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    kriso_status->time_usec = mavlink_msg_kriso_status_get_time_usec(msg);
    kriso_status->nav_longitude = mavlink_msg_kriso_status_get_nav_longitude(msg);
    kriso_status->nav_latitude = mavlink_msg_kriso_status_get_nav_latitude(msg);
    kriso_status->nav_roll = mavlink_msg_kriso_status_get_nav_roll(msg);
    kriso_status->nav_pitch = mavlink_msg_kriso_status_get_nav_pitch(msg);
    kriso_status->nav_yaw = mavlink_msg_kriso_status_get_nav_yaw(msg);
    kriso_status->nav_cog = mavlink_msg_kriso_status_get_nav_cog(msg);
    kriso_status->nav_sog = mavlink_msg_kriso_status_get_nav_sog(msg);
    kriso_status->nav_uspd = mavlink_msg_kriso_status_get_nav_uspd(msg);
    kriso_status->nav_vspd = mavlink_msg_kriso_status_get_nav_vspd(msg);
    kriso_status->nav_wspd = mavlink_msg_kriso_status_get_nav_wspd(msg);
    kriso_status->nav_heave = mavlink_msg_kriso_status_get_nav_heave(msg);
    kriso_status->nav_gpstime = mavlink_msg_kriso_status_get_nav_gpstime(msg);
    kriso_status->wea_airtem = mavlink_msg_kriso_status_get_wea_airtem(msg);
    kriso_status->wea_wattem = mavlink_msg_kriso_status_get_wea_wattem(msg);
    kriso_status->wea_press = mavlink_msg_kriso_status_get_wea_press(msg);
    kriso_status->wea_relhum = mavlink_msg_kriso_status_get_wea_relhum(msg);
    kriso_status->wea_dewpt = mavlink_msg_kriso_status_get_wea_dewpt(msg);
    kriso_status->wea_windirt = mavlink_msg_kriso_status_get_wea_windirt(msg);
    kriso_status->wea_winspdt = mavlink_msg_kriso_status_get_wea_winspdt(msg);
    kriso_status->wea_windirr = mavlink_msg_kriso_status_get_wea_windirr(msg);
    kriso_status->wea_watspdr = mavlink_msg_kriso_status_get_wea_watspdr(msg);
    kriso_status->wea_watdir = mavlink_msg_kriso_status_get_wea_watdir(msg);
    kriso_status->wea_watspd = mavlink_msg_kriso_status_get_wea_watspd(msg);
    kriso_status->wea_visibiran = mavlink_msg_kriso_status_get_wea_visibiran(msg);
    kriso_status->nav_mode = mavlink_msg_kriso_status_get_nav_mode(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_KRISO_STATUS_LEN? msg->len : MAVLINK_MSG_ID_KRISO_STATUS_LEN;
        memset(kriso_status, 0, MAVLINK_MSG_ID_KRISO_STATUS_LEN);
    memcpy(kriso_status, _MAV_PAYLOAD(msg), len);
#endif
}
