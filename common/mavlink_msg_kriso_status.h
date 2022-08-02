#pragma once
// MESSAGE KRISO_STATUS PACKING

#define MAVLINK_MSG_ID_KRISO_STATUS 9100


typedef struct __mavlink_kriso_status_t {
 uint64_t time_usec; /*< [us] Timestamp (synced to UNIX time or since system boot).*/
 float roll; /*< [rad/s] Desired roll rate*/
 float pitch; /*< [rad/s] Desired pitch rate*/
 float yaw; /*< [rad/s] Desired yaw rate*/
 int32_t lat; /*< [degE7] Latitude, expressed*/
 int32_t lon; /*< [degE7] Longitude, expressed*/
} mavlink_kriso_status_t;

#define MAVLINK_MSG_ID_KRISO_STATUS_LEN 28
#define MAVLINK_MSG_ID_KRISO_STATUS_MIN_LEN 28
#define MAVLINK_MSG_ID_9100_LEN 28
#define MAVLINK_MSG_ID_9100_MIN_LEN 28

#define MAVLINK_MSG_ID_KRISO_STATUS_CRC 251
#define MAVLINK_MSG_ID_9100_CRC 251



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_KRISO_STATUS { \
    9100, \
    "KRISO_STATUS", \
    6, \
    {  { "time_usec", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_kriso_status_t, time_usec) }, \
         { "roll", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_kriso_status_t, roll) }, \
         { "pitch", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_kriso_status_t, pitch) }, \
         { "yaw", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_kriso_status_t, yaw) }, \
         { "lat", NULL, MAVLINK_TYPE_INT32_T, 0, 20, offsetof(mavlink_kriso_status_t, lat) }, \
         { "lon", NULL, MAVLINK_TYPE_INT32_T, 0, 24, offsetof(mavlink_kriso_status_t, lon) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_KRISO_STATUS { \
    "KRISO_STATUS", \
    6, \
    {  { "time_usec", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_kriso_status_t, time_usec) }, \
         { "roll", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_kriso_status_t, roll) }, \
         { "pitch", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_kriso_status_t, pitch) }, \
         { "yaw", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_kriso_status_t, yaw) }, \
         { "lat", NULL, MAVLINK_TYPE_INT32_T, 0, 20, offsetof(mavlink_kriso_status_t, lat) }, \
         { "lon", NULL, MAVLINK_TYPE_INT32_T, 0, 24, offsetof(mavlink_kriso_status_t, lon) }, \
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
 * @param roll [rad/s] Desired roll rate
 * @param pitch [rad/s] Desired pitch rate
 * @param yaw [rad/s] Desired yaw rate
 * @param lat [degE7] Latitude, expressed
 * @param lon [degE7] Longitude, expressed
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_kriso_status_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint64_t time_usec, float roll, float pitch, float yaw, int32_t lat, int32_t lon)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_KRISO_STATUS_LEN];
    _mav_put_uint64_t(buf, 0, time_usec);
    _mav_put_float(buf, 8, roll);
    _mav_put_float(buf, 12, pitch);
    _mav_put_float(buf, 16, yaw);
    _mav_put_int32_t(buf, 20, lat);
    _mav_put_int32_t(buf, 24, lon);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_KRISO_STATUS_LEN);
#else
    mavlink_kriso_status_t packet;
    packet.time_usec = time_usec;
    packet.roll = roll;
    packet.pitch = pitch;
    packet.yaw = yaw;
    packet.lat = lat;
    packet.lon = lon;

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
 * @param roll [rad/s] Desired roll rate
 * @param pitch [rad/s] Desired pitch rate
 * @param yaw [rad/s] Desired yaw rate
 * @param lat [degE7] Latitude, expressed
 * @param lon [degE7] Longitude, expressed
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_kriso_status_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint64_t time_usec,float roll,float pitch,float yaw,int32_t lat,int32_t lon)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_KRISO_STATUS_LEN];
    _mav_put_uint64_t(buf, 0, time_usec);
    _mav_put_float(buf, 8, roll);
    _mav_put_float(buf, 12, pitch);
    _mav_put_float(buf, 16, yaw);
    _mav_put_int32_t(buf, 20, lat);
    _mav_put_int32_t(buf, 24, lon);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_KRISO_STATUS_LEN);
#else
    mavlink_kriso_status_t packet;
    packet.time_usec = time_usec;
    packet.roll = roll;
    packet.pitch = pitch;
    packet.yaw = yaw;
    packet.lat = lat;
    packet.lon = lon;

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
    return mavlink_msg_kriso_status_pack(system_id, component_id, msg, kriso_status->time_usec, kriso_status->roll, kriso_status->pitch, kriso_status->yaw, kriso_status->lat, kriso_status->lon);
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
    return mavlink_msg_kriso_status_pack_chan(system_id, component_id, chan, msg, kriso_status->time_usec, kriso_status->roll, kriso_status->pitch, kriso_status->yaw, kriso_status->lat, kriso_status->lon);
}

/**
 * @brief Send a kriso_status message
 * @param chan MAVLink channel to send the message
 *
 * @param time_usec [us] Timestamp (synced to UNIX time or since system boot).
 * @param roll [rad/s] Desired roll rate
 * @param pitch [rad/s] Desired pitch rate
 * @param yaw [rad/s] Desired yaw rate
 * @param lat [degE7] Latitude, expressed
 * @param lon [degE7] Longitude, expressed
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_kriso_status_send(mavlink_channel_t chan, uint64_t time_usec, float roll, float pitch, float yaw, int32_t lat, int32_t lon)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_KRISO_STATUS_LEN];
    _mav_put_uint64_t(buf, 0, time_usec);
    _mav_put_float(buf, 8, roll);
    _mav_put_float(buf, 12, pitch);
    _mav_put_float(buf, 16, yaw);
    _mav_put_int32_t(buf, 20, lat);
    _mav_put_int32_t(buf, 24, lon);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_KRISO_STATUS, buf, MAVLINK_MSG_ID_KRISO_STATUS_MIN_LEN, MAVLINK_MSG_ID_KRISO_STATUS_LEN, MAVLINK_MSG_ID_KRISO_STATUS_CRC);
#else
    mavlink_kriso_status_t packet;
    packet.time_usec = time_usec;
    packet.roll = roll;
    packet.pitch = pitch;
    packet.yaw = yaw;
    packet.lat = lat;
    packet.lon = lon;

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
    mavlink_msg_kriso_status_send(chan, kriso_status->time_usec, kriso_status->roll, kriso_status->pitch, kriso_status->yaw, kriso_status->lat, kriso_status->lon);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_KRISO_STATUS, (const char *)kriso_status, MAVLINK_MSG_ID_KRISO_STATUS_MIN_LEN, MAVLINK_MSG_ID_KRISO_STATUS_LEN, MAVLINK_MSG_ID_KRISO_STATUS_CRC);
#endif
}

#if MAVLINK_MSG_ID_KRISO_STATUS_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_kriso_status_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint64_t time_usec, float roll, float pitch, float yaw, int32_t lat, int32_t lon)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint64_t(buf, 0, time_usec);
    _mav_put_float(buf, 8, roll);
    _mav_put_float(buf, 12, pitch);
    _mav_put_float(buf, 16, yaw);
    _mav_put_int32_t(buf, 20, lat);
    _mav_put_int32_t(buf, 24, lon);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_KRISO_STATUS, buf, MAVLINK_MSG_ID_KRISO_STATUS_MIN_LEN, MAVLINK_MSG_ID_KRISO_STATUS_LEN, MAVLINK_MSG_ID_KRISO_STATUS_CRC);
#else
    mavlink_kriso_status_t *packet = (mavlink_kriso_status_t *)msgbuf;
    packet->time_usec = time_usec;
    packet->roll = roll;
    packet->pitch = pitch;
    packet->yaw = yaw;
    packet->lat = lat;
    packet->lon = lon;

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
 * @brief Get field roll from kriso_status message
 *
 * @return [rad/s] Desired roll rate
 */
static inline float mavlink_msg_kriso_status_get_roll(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  8);
}

/**
 * @brief Get field pitch from kriso_status message
 *
 * @return [rad/s] Desired pitch rate
 */
static inline float mavlink_msg_kriso_status_get_pitch(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  12);
}

/**
 * @brief Get field yaw from kriso_status message
 *
 * @return [rad/s] Desired yaw rate
 */
static inline float mavlink_msg_kriso_status_get_yaw(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  16);
}

/**
 * @brief Get field lat from kriso_status message
 *
 * @return [degE7] Latitude, expressed
 */
static inline int32_t mavlink_msg_kriso_status_get_lat(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int32_t(msg,  20);
}

/**
 * @brief Get field lon from kriso_status message
 *
 * @return [degE7] Longitude, expressed
 */
static inline int32_t mavlink_msg_kriso_status_get_lon(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int32_t(msg,  24);
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
    kriso_status->roll = mavlink_msg_kriso_status_get_roll(msg);
    kriso_status->pitch = mavlink_msg_kriso_status_get_pitch(msg);
    kriso_status->yaw = mavlink_msg_kriso_status_get_yaw(msg);
    kriso_status->lat = mavlink_msg_kriso_status_get_lat(msg);
    kriso_status->lon = mavlink_msg_kriso_status_get_lon(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_KRISO_STATUS_LEN? msg->len : MAVLINK_MSG_ID_KRISO_STATUS_LEN;
        memset(kriso_status, 0, MAVLINK_MSG_ID_KRISO_STATUS_LEN);
    memcpy(kriso_status, _MAV_PAYLOAD(msg), len);
#endif
}
