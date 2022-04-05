#pragma once
// MESSAGE JEBI_DEBUG_CMD PACKING

#define MAVLINK_MSG_ID_JEBI_DEBUG_CMD 229


typedef struct __mavlink_jebi_debug_cmd_t {
 float ctrl_out[4]; /*<   */
 float position[3]; /*<   */
} mavlink_jebi_debug_cmd_t;

#define MAVLINK_MSG_ID_JEBI_DEBUG_CMD_LEN 28
#define MAVLINK_MSG_ID_JEBI_DEBUG_CMD_MIN_LEN 28
#define MAVLINK_MSG_ID_229_LEN 28
#define MAVLINK_MSG_ID_229_MIN_LEN 28

#define MAVLINK_MSG_ID_JEBI_DEBUG_CMD_CRC 122
#define MAVLINK_MSG_ID_229_CRC 122

#define MAVLINK_MSG_JEBI_DEBUG_CMD_FIELD_CTRL_OUT_LEN 4
#define MAVLINK_MSG_JEBI_DEBUG_CMD_FIELD_POSITION_LEN 3

#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_JEBI_DEBUG_CMD { \
    229, \
    "JEBI_DEBUG_CMD", \
    2, \
    {  { "ctrl_out", NULL, MAVLINK_TYPE_FLOAT, 4, 0, offsetof(mavlink_jebi_debug_cmd_t, ctrl_out) }, \
         { "position", NULL, MAVLINK_TYPE_FLOAT, 3, 16, offsetof(mavlink_jebi_debug_cmd_t, position) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_JEBI_DEBUG_CMD { \
    "JEBI_DEBUG_CMD", \
    2, \
    {  { "ctrl_out", NULL, MAVLINK_TYPE_FLOAT, 4, 0, offsetof(mavlink_jebi_debug_cmd_t, ctrl_out) }, \
         { "position", NULL, MAVLINK_TYPE_FLOAT, 3, 16, offsetof(mavlink_jebi_debug_cmd_t, position) }, \
         } \
}
#endif

/**
 * @brief Pack a jebi_debug_cmd message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param ctrl_out
 * @param position
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_jebi_debug_cmd_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               const float *ctrl_out, const float *position)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_JEBI_DEBUG_CMD_LEN];

    _mav_put_float_array(buf, 0, ctrl_out, 4);
    _mav_put_float_array(buf, 16, position, 3);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_JEBI_DEBUG_CMD_LEN);
#else
    mavlink_jebi_debug_cmd_t packet;

    mav_array_memcpy(packet.ctrl_out, ctrl_out, sizeof(float)*4);
    mav_array_memcpy(packet.position, position, sizeof(float)*3);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_JEBI_DEBUG_CMD_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_JEBI_DEBUG_CMD;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_JEBI_DEBUG_CMD_MIN_LEN, MAVLINK_MSG_ID_JEBI_DEBUG_CMD_LEN, MAVLINK_MSG_ID_JEBI_DEBUG_CMD_CRC);
}

/**
 * @brief Pack a jebi_debug_cmd message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param ctrl_out
 * @param position
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_jebi_debug_cmd_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   const float *ctrl_out,const float *position)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_JEBI_DEBUG_CMD_LEN];

    _mav_put_float_array(buf, 0, ctrl_out, 4);
    _mav_put_float_array(buf, 16, position, 3);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_JEBI_DEBUG_CMD_LEN);
#else
    mavlink_jebi_debug_cmd_t packet;

    mav_array_memcpy(packet.ctrl_out, ctrl_out, sizeof(float)*4);
    mav_array_memcpy(packet.position, position, sizeof(float)*3);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_JEBI_DEBUG_CMD_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_JEBI_DEBUG_CMD;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_JEBI_DEBUG_CMD_MIN_LEN, MAVLINK_MSG_ID_JEBI_DEBUG_CMD_LEN, MAVLINK_MSG_ID_JEBI_DEBUG_CMD_CRC);
}

/**
 * @brief Encode a jebi_debug_cmd struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param jebi_debug_cmd C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_jebi_debug_cmd_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_jebi_debug_cmd_t* jebi_debug_cmd)
{
    return mavlink_msg_jebi_debug_cmd_pack(system_id, component_id, msg, jebi_debug_cmd->ctrl_out, jebi_debug_cmd->position);
}

/**
 * @brief Encode a jebi_debug_cmd struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param jebi_debug_cmd C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_jebi_debug_cmd_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_jebi_debug_cmd_t* jebi_debug_cmd)
{
    return mavlink_msg_jebi_debug_cmd_pack_chan(system_id, component_id, chan, msg, jebi_debug_cmd->ctrl_out, jebi_debug_cmd->position);
}

/**
 * @brief Send a jebi_debug_cmd message
 * @param chan MAVLink channel to send the message
 *
 * @param ctrl_out
 * @param position
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_jebi_debug_cmd_send(mavlink_channel_t chan, const float *ctrl_out, const float *position)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_JEBI_DEBUG_CMD_LEN];

    _mav_put_float_array(buf, 0, ctrl_out, 4);
    _mav_put_float_array(buf, 16, position, 3);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_JEBI_DEBUG_CMD, buf, MAVLINK_MSG_ID_JEBI_DEBUG_CMD_MIN_LEN, MAVLINK_MSG_ID_JEBI_DEBUG_CMD_LEN, MAVLINK_MSG_ID_JEBI_DEBUG_CMD_CRC);
#else
    mavlink_jebi_debug_cmd_t packet;

    mav_array_memcpy(packet.ctrl_out, ctrl_out, sizeof(float)*4);
    mav_array_memcpy(packet.position, position, sizeof(float)*3);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_JEBI_DEBUG_CMD, (const char *)&packet, MAVLINK_MSG_ID_JEBI_DEBUG_CMD_MIN_LEN, MAVLINK_MSG_ID_JEBI_DEBUG_CMD_LEN, MAVLINK_MSG_ID_JEBI_DEBUG_CMD_CRC);
#endif
}

/**
 * @brief Send a jebi_debug_cmd message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_jebi_debug_cmd_send_struct(mavlink_channel_t chan, const mavlink_jebi_debug_cmd_t* jebi_debug_cmd)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_jebi_debug_cmd_send(chan, jebi_debug_cmd->ctrl_out, jebi_debug_cmd->position);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_JEBI_DEBUG_CMD, (const char *)jebi_debug_cmd, MAVLINK_MSG_ID_JEBI_DEBUG_CMD_MIN_LEN, MAVLINK_MSG_ID_JEBI_DEBUG_CMD_LEN, MAVLINK_MSG_ID_JEBI_DEBUG_CMD_CRC);
#endif
}

#if MAVLINK_MSG_ID_JEBI_DEBUG_CMD_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_jebi_debug_cmd_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  const float *ctrl_out, const float *position)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;

    _mav_put_float_array(buf, 0, ctrl_out, 4);
    _mav_put_float_array(buf, 16, position, 3);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_JEBI_DEBUG_CMD, buf, MAVLINK_MSG_ID_JEBI_DEBUG_CMD_MIN_LEN, MAVLINK_MSG_ID_JEBI_DEBUG_CMD_LEN, MAVLINK_MSG_ID_JEBI_DEBUG_CMD_CRC);
#else
    mavlink_jebi_debug_cmd_t *packet = (mavlink_jebi_debug_cmd_t *)msgbuf;

    mav_array_memcpy(packet->ctrl_out, ctrl_out, sizeof(float)*4);
    mav_array_memcpy(packet->position, position, sizeof(float)*3);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_JEBI_DEBUG_CMD, (const char *)packet, MAVLINK_MSG_ID_JEBI_DEBUG_CMD_MIN_LEN, MAVLINK_MSG_ID_JEBI_DEBUG_CMD_LEN, MAVLINK_MSG_ID_JEBI_DEBUG_CMD_CRC);
#endif
}
#endif

#endif

// MESSAGE JEBI_DEBUG_CMD UNPACKING


/**
 * @brief Get field ctrl_out from jebi_debug_cmd message
 *
 * @return
 */
static inline uint16_t mavlink_msg_jebi_debug_cmd_get_ctrl_out(const mavlink_message_t* msg, float *ctrl_out)
{
    return _MAV_RETURN_float_array(msg, ctrl_out, 4,  0);
}

/**
 * @brief Get field position from jebi_debug_cmd message
 *
 * @return
 */
static inline uint16_t mavlink_msg_jebi_debug_cmd_get_position(const mavlink_message_t* msg, float *position)
{
    return _MAV_RETURN_float_array(msg, position, 3,  16);
}

/**
 * @brief Decode a jebi_debug_cmd message into a struct
 *
 * @param msg The message to decode
 * @param jebi_debug_cmd C-struct to decode the message contents into
 */
static inline void mavlink_msg_jebi_debug_cmd_decode(const mavlink_message_t* msg, mavlink_jebi_debug_cmd_t* jebi_debug_cmd)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_jebi_debug_cmd_get_ctrl_out(msg, jebi_debug_cmd->ctrl_out);
    mavlink_msg_jebi_debug_cmd_get_position(msg, jebi_debug_cmd->position);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_JEBI_DEBUG_CMD_LEN? msg->len : MAVLINK_MSG_ID_JEBI_DEBUG_CMD_LEN;
        memset(jebi_debug_cmd, 0, MAVLINK_MSG_ID_JEBI_DEBUG_CMD_LEN);
    memcpy(jebi_debug_cmd, _MAV_PAYLOAD(msg), len);
#endif
}
