#pragma once
// MESSAGE JEBI_DEBUG PACKING

#define MAVLINK_MSG_ID_JEBI_DEBUG 228


typedef struct __mavlink_jebi_debug_t {
 uint32_t time_boot_ms; /*< [ms] */
 float rc_in[4]; /*<   */
 float ctrl_out[4]; /*<   */
 float w_meas[2]; /*<   */
 float torque_meas[2]; /*<   */
 float euler[3]; /*<   */
 float angvel[3]; /*<   */
} mavlink_jebi_debug_t;

#define MAVLINK_MSG_ID_JEBI_DEBUG_LEN 76
#define MAVLINK_MSG_ID_JEBI_DEBUG_MIN_LEN 76
#define MAVLINK_MSG_ID_228_LEN 76
#define MAVLINK_MSG_ID_228_MIN_LEN 76

#define MAVLINK_MSG_ID_JEBI_DEBUG_CRC 141
#define MAVLINK_MSG_ID_228_CRC 141

#define MAVLINK_MSG_JEBI_DEBUG_FIELD_RC_IN_LEN 4
#define MAVLINK_MSG_JEBI_DEBUG_FIELD_CTRL_OUT_LEN 4
#define MAVLINK_MSG_JEBI_DEBUG_FIELD_W_MEAS_LEN 2
#define MAVLINK_MSG_JEBI_DEBUG_FIELD_TORQUE_MEAS_LEN 2
#define MAVLINK_MSG_JEBI_DEBUG_FIELD_EULER_LEN 3
#define MAVLINK_MSG_JEBI_DEBUG_FIELD_ANGVEL_LEN 3

#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_JEBI_DEBUG { \
    228, \
    "JEBI_DEBUG", \
    7, \
    {  { "time_boot_ms", NULL, MAVLINK_TYPE_UINT32_T, 0, 0, offsetof(mavlink_jebi_debug_t, time_boot_ms) }, \
         { "rc_in", NULL, MAVLINK_TYPE_FLOAT, 4, 4, offsetof(mavlink_jebi_debug_t, rc_in) }, \
         { "ctrl_out", NULL, MAVLINK_TYPE_FLOAT, 4, 20, offsetof(mavlink_jebi_debug_t, ctrl_out) }, \
         { "w_meas", NULL, MAVLINK_TYPE_FLOAT, 2, 36, offsetof(mavlink_jebi_debug_t, w_meas) }, \
         { "torque_meas", NULL, MAVLINK_TYPE_FLOAT, 2, 44, offsetof(mavlink_jebi_debug_t, torque_meas) }, \
         { "euler", NULL, MAVLINK_TYPE_FLOAT, 3, 52, offsetof(mavlink_jebi_debug_t, euler) }, \
         { "angvel", NULL, MAVLINK_TYPE_FLOAT, 3, 64, offsetof(mavlink_jebi_debug_t, angvel) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_JEBI_DEBUG { \
    "JEBI_DEBUG", \
    7, \
    {  { "time_boot_ms", NULL, MAVLINK_TYPE_UINT32_T, 0, 0, offsetof(mavlink_jebi_debug_t, time_boot_ms) }, \
         { "rc_in", NULL, MAVLINK_TYPE_FLOAT, 4, 4, offsetof(mavlink_jebi_debug_t, rc_in) }, \
         { "ctrl_out", NULL, MAVLINK_TYPE_FLOAT, 4, 20, offsetof(mavlink_jebi_debug_t, ctrl_out) }, \
         { "w_meas", NULL, MAVLINK_TYPE_FLOAT, 2, 36, offsetof(mavlink_jebi_debug_t, w_meas) }, \
         { "torque_meas", NULL, MAVLINK_TYPE_FLOAT, 2, 44, offsetof(mavlink_jebi_debug_t, torque_meas) }, \
         { "euler", NULL, MAVLINK_TYPE_FLOAT, 3, 52, offsetof(mavlink_jebi_debug_t, euler) }, \
         { "angvel", NULL, MAVLINK_TYPE_FLOAT, 3, 64, offsetof(mavlink_jebi_debug_t, angvel) }, \
         } \
}
#endif

/**
 * @brief Pack a jebi_debug message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param time_boot_ms [ms] 
 * @param rc_in   
 * @param ctrl_out   
 * @param w_meas   
 * @param torque_meas   
 * @param euler   
 * @param angvel   
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_jebi_debug_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint32_t time_boot_ms, const float *rc_in, const float *ctrl_out, const float *w_meas, const float *torque_meas, const float *euler, const float *angvel)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_JEBI_DEBUG_LEN];
    _mav_put_uint32_t(buf, 0, time_boot_ms);
    _mav_put_float_array(buf, 4, rc_in, 4);
    _mav_put_float_array(buf, 20, ctrl_out, 4);
    _mav_put_float_array(buf, 36, w_meas, 2);
    _mav_put_float_array(buf, 44, torque_meas, 2);
    _mav_put_float_array(buf, 52, euler, 3);
    _mav_put_float_array(buf, 64, angvel, 3);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_JEBI_DEBUG_LEN);
#else
    mavlink_jebi_debug_t packet;
    packet.time_boot_ms = time_boot_ms;
    mav_array_memcpy(packet.rc_in, rc_in, sizeof(float)*4);
    mav_array_memcpy(packet.ctrl_out, ctrl_out, sizeof(float)*4);
    mav_array_memcpy(packet.w_meas, w_meas, sizeof(float)*2);
    mav_array_memcpy(packet.torque_meas, torque_meas, sizeof(float)*2);
    mav_array_memcpy(packet.euler, euler, sizeof(float)*3);
    mav_array_memcpy(packet.angvel, angvel, sizeof(float)*3);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_JEBI_DEBUG_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_JEBI_DEBUG;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_JEBI_DEBUG_MIN_LEN, MAVLINK_MSG_ID_JEBI_DEBUG_LEN, MAVLINK_MSG_ID_JEBI_DEBUG_CRC);
}

/**
 * @brief Pack a jebi_debug message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param time_boot_ms [ms] 
 * @param rc_in   
 * @param ctrl_out   
 * @param w_meas   
 * @param torque_meas   
 * @param euler   
 * @param angvel   
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_jebi_debug_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint32_t time_boot_ms,const float *rc_in,const float *ctrl_out,const float *w_meas,const float *torque_meas,const float *euler,const float *angvel)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_JEBI_DEBUG_LEN];
    _mav_put_uint32_t(buf, 0, time_boot_ms);
    _mav_put_float_array(buf, 4, rc_in, 4);
    _mav_put_float_array(buf, 20, ctrl_out, 4);
    _mav_put_float_array(buf, 36, w_meas, 2);
    _mav_put_float_array(buf, 44, torque_meas, 2);
    _mav_put_float_array(buf, 52, euler, 3);
    _mav_put_float_array(buf, 64, angvel, 3);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_JEBI_DEBUG_LEN);
#else
    mavlink_jebi_debug_t packet;
    packet.time_boot_ms = time_boot_ms;
    mav_array_memcpy(packet.rc_in, rc_in, sizeof(float)*4);
    mav_array_memcpy(packet.ctrl_out, ctrl_out, sizeof(float)*4);
    mav_array_memcpy(packet.w_meas, w_meas, sizeof(float)*2);
    mav_array_memcpy(packet.torque_meas, torque_meas, sizeof(float)*2);
    mav_array_memcpy(packet.euler, euler, sizeof(float)*3);
    mav_array_memcpy(packet.angvel, angvel, sizeof(float)*3);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_JEBI_DEBUG_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_JEBI_DEBUG;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_JEBI_DEBUG_MIN_LEN, MAVLINK_MSG_ID_JEBI_DEBUG_LEN, MAVLINK_MSG_ID_JEBI_DEBUG_CRC);
}

/**
 * @brief Encode a jebi_debug struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param jebi_debug C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_jebi_debug_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_jebi_debug_t* jebi_debug)
{
    return mavlink_msg_jebi_debug_pack(system_id, component_id, msg, jebi_debug->time_boot_ms, jebi_debug->rc_in, jebi_debug->ctrl_out, jebi_debug->w_meas, jebi_debug->torque_meas, jebi_debug->euler, jebi_debug->angvel);
}

/**
 * @brief Encode a jebi_debug struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param jebi_debug C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_jebi_debug_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_jebi_debug_t* jebi_debug)
{
    return mavlink_msg_jebi_debug_pack_chan(system_id, component_id, chan, msg, jebi_debug->time_boot_ms, jebi_debug->rc_in, jebi_debug->ctrl_out, jebi_debug->w_meas, jebi_debug->torque_meas, jebi_debug->euler, jebi_debug->angvel);
}

/**
 * @brief Send a jebi_debug message
 * @param chan MAVLink channel to send the message
 *
 * @param time_boot_ms [ms] 
 * @param rc_in   
 * @param ctrl_out   
 * @param w_meas   
 * @param torque_meas   
 * @param euler   
 * @param angvel   
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_jebi_debug_send(mavlink_channel_t chan, uint32_t time_boot_ms, const float *rc_in, const float *ctrl_out, const float *w_meas, const float *torque_meas, const float *euler, const float *angvel)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_JEBI_DEBUG_LEN];
    _mav_put_uint32_t(buf, 0, time_boot_ms);
    _mav_put_float_array(buf, 4, rc_in, 4);
    _mav_put_float_array(buf, 20, ctrl_out, 4);
    _mav_put_float_array(buf, 36, w_meas, 2);
    _mav_put_float_array(buf, 44, torque_meas, 2);
    _mav_put_float_array(buf, 52, euler, 3);
    _mav_put_float_array(buf, 64, angvel, 3);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_JEBI_DEBUG, buf, MAVLINK_MSG_ID_JEBI_DEBUG_MIN_LEN, MAVLINK_MSG_ID_JEBI_DEBUG_LEN, MAVLINK_MSG_ID_JEBI_DEBUG_CRC);
#else
    mavlink_jebi_debug_t packet;
    packet.time_boot_ms = time_boot_ms;
    mav_array_memcpy(packet.rc_in, rc_in, sizeof(float)*4);
    mav_array_memcpy(packet.ctrl_out, ctrl_out, sizeof(float)*4);
    mav_array_memcpy(packet.w_meas, w_meas, sizeof(float)*2);
    mav_array_memcpy(packet.torque_meas, torque_meas, sizeof(float)*2);
    mav_array_memcpy(packet.euler, euler, sizeof(float)*3);
    mav_array_memcpy(packet.angvel, angvel, sizeof(float)*3);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_JEBI_DEBUG, (const char *)&packet, MAVLINK_MSG_ID_JEBI_DEBUG_MIN_LEN, MAVLINK_MSG_ID_JEBI_DEBUG_LEN, MAVLINK_MSG_ID_JEBI_DEBUG_CRC);
#endif
}

/**
 * @brief Send a jebi_debug message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_jebi_debug_send_struct(mavlink_channel_t chan, const mavlink_jebi_debug_t* jebi_debug)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_jebi_debug_send(chan, jebi_debug->time_boot_ms, jebi_debug->rc_in, jebi_debug->ctrl_out, jebi_debug->w_meas, jebi_debug->torque_meas, jebi_debug->euler, jebi_debug->angvel);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_JEBI_DEBUG, (const char *)jebi_debug, MAVLINK_MSG_ID_JEBI_DEBUG_MIN_LEN, MAVLINK_MSG_ID_JEBI_DEBUG_LEN, MAVLINK_MSG_ID_JEBI_DEBUG_CRC);
#endif
}

#if MAVLINK_MSG_ID_JEBI_DEBUG_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_jebi_debug_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint32_t time_boot_ms, const float *rc_in, const float *ctrl_out, const float *w_meas, const float *torque_meas, const float *euler, const float *angvel)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint32_t(buf, 0, time_boot_ms);
    _mav_put_float_array(buf, 4, rc_in, 4);
    _mav_put_float_array(buf, 20, ctrl_out, 4);
    _mav_put_float_array(buf, 36, w_meas, 2);
    _mav_put_float_array(buf, 44, torque_meas, 2);
    _mav_put_float_array(buf, 52, euler, 3);
    _mav_put_float_array(buf, 64, angvel, 3);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_JEBI_DEBUG, buf, MAVLINK_MSG_ID_JEBI_DEBUG_MIN_LEN, MAVLINK_MSG_ID_JEBI_DEBUG_LEN, MAVLINK_MSG_ID_JEBI_DEBUG_CRC);
#else
    mavlink_jebi_debug_t *packet = (mavlink_jebi_debug_t *)msgbuf;
    packet->time_boot_ms = time_boot_ms;
    mav_array_memcpy(packet->rc_in, rc_in, sizeof(float)*4);
    mav_array_memcpy(packet->ctrl_out, ctrl_out, sizeof(float)*4);
    mav_array_memcpy(packet->w_meas, w_meas, sizeof(float)*2);
    mav_array_memcpy(packet->torque_meas, torque_meas, sizeof(float)*2);
    mav_array_memcpy(packet->euler, euler, sizeof(float)*3);
    mav_array_memcpy(packet->angvel, angvel, sizeof(float)*3);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_JEBI_DEBUG, (const char *)packet, MAVLINK_MSG_ID_JEBI_DEBUG_MIN_LEN, MAVLINK_MSG_ID_JEBI_DEBUG_LEN, MAVLINK_MSG_ID_JEBI_DEBUG_CRC);
#endif
}
#endif

#endif

// MESSAGE JEBI_DEBUG UNPACKING


/**
 * @brief Get field time_boot_ms from jebi_debug message
 *
 * @return [ms] 
 */
static inline uint32_t mavlink_msg_jebi_debug_get_time_boot_ms(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint32_t(msg,  0);
}

/**
 * @brief Get field rc_in from jebi_debug message
 *
 * @return   
 */
static inline uint16_t mavlink_msg_jebi_debug_get_rc_in(const mavlink_message_t* msg, float *rc_in)
{
    return _MAV_RETURN_float_array(msg, rc_in, 4,  4);
}

/**
 * @brief Get field ctrl_out from jebi_debug message
 *
 * @return   
 */
static inline uint16_t mavlink_msg_jebi_debug_get_ctrl_out(const mavlink_message_t* msg, float *ctrl_out)
{
    return _MAV_RETURN_float_array(msg, ctrl_out, 4,  20);
}

/**
 * @brief Get field w_meas from jebi_debug message
 *
 * @return   
 */
static inline uint16_t mavlink_msg_jebi_debug_get_w_meas(const mavlink_message_t* msg, float *w_meas)
{
    return _MAV_RETURN_float_array(msg, w_meas, 2,  36);
}

/**
 * @brief Get field torque_meas from jebi_debug message
 *
 * @return   
 */
static inline uint16_t mavlink_msg_jebi_debug_get_torque_meas(const mavlink_message_t* msg, float *torque_meas)
{
    return _MAV_RETURN_float_array(msg, torque_meas, 2,  44);
}

/**
 * @brief Get field euler from jebi_debug message
 *
 * @return   
 */
static inline uint16_t mavlink_msg_jebi_debug_get_euler(const mavlink_message_t* msg, float *euler)
{
    return _MAV_RETURN_float_array(msg, euler, 3,  52);
}

/**
 * @brief Get field angvel from jebi_debug message
 *
 * @return   
 */
static inline uint16_t mavlink_msg_jebi_debug_get_angvel(const mavlink_message_t* msg, float *angvel)
{
    return _MAV_RETURN_float_array(msg, angvel, 3,  64);
}

/**
 * @brief Decode a jebi_debug message into a struct
 *
 * @param msg The message to decode
 * @param jebi_debug C-struct to decode the message contents into
 */
static inline void mavlink_msg_jebi_debug_decode(const mavlink_message_t* msg, mavlink_jebi_debug_t* jebi_debug)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    jebi_debug->time_boot_ms = mavlink_msg_jebi_debug_get_time_boot_ms(msg);
    mavlink_msg_jebi_debug_get_rc_in(msg, jebi_debug->rc_in);
    mavlink_msg_jebi_debug_get_ctrl_out(msg, jebi_debug->ctrl_out);
    mavlink_msg_jebi_debug_get_w_meas(msg, jebi_debug->w_meas);
    mavlink_msg_jebi_debug_get_torque_meas(msg, jebi_debug->torque_meas);
    mavlink_msg_jebi_debug_get_euler(msg, jebi_debug->euler);
    mavlink_msg_jebi_debug_get_angvel(msg, jebi_debug->angvel);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_JEBI_DEBUG_LEN? msg->len : MAVLINK_MSG_ID_JEBI_DEBUG_LEN;
        memset(jebi_debug, 0, MAVLINK_MSG_ID_JEBI_DEBUG_LEN);
    memcpy(jebi_debug, _MAV_PAYLOAD(msg), len);
#endif
}
