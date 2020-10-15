#pragma once
// MESSAGE ATTITUDE2 PACKING

#define MAVLINK_MSG_ID_ATTITUDE2 13001


typedef struct __mavlink_attitude2_t {
 uint32_t time_boot_ms; /*< [ms] Timestamp (time since system boot).*/
 float roll; /*< [rad] Roll angle (-pi..+pi)*/
 float pitch; /*< [rad] Pitch angle (-pi..+pi)*/
 float yaw; /*< [rad] Yaw angle (-pi..+pi)*/
 float rollspeed; /*< [rad/s] Roll angular speed*/
 float pitchspeed; /*< [rad/s] Pitch angular speed*/
 float yawspeed; /*< [rad/s] Yaw angular speed*/
 uint8_t attitude_id; /*<  Attitude Id.*/
} mavlink_attitude2_t;

#define MAVLINK_MSG_ID_ATTITUDE2_LEN 29
#define MAVLINK_MSG_ID_ATTITUDE2_MIN_LEN 29
#define MAVLINK_MSG_ID_13001_LEN 29
#define MAVLINK_MSG_ID_13001_MIN_LEN 29

#define MAVLINK_MSG_ID_ATTITUDE2_CRC 54
#define MAVLINK_MSG_ID_13001_CRC 54



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_ATTITUDE2 { \
    13001, \
    "ATTITUDE2", \
    8, \
    {  { "time_boot_ms", NULL, MAVLINK_TYPE_UINT32_T, 0, 0, offsetof(mavlink_attitude2_t, time_boot_ms) }, \
         { "attitude_id", NULL, MAVLINK_TYPE_UINT8_T, 0, 28, offsetof(mavlink_attitude2_t, attitude_id) }, \
         { "roll", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_attitude2_t, roll) }, \
         { "pitch", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_attitude2_t, pitch) }, \
         { "yaw", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_attitude2_t, yaw) }, \
         { "rollspeed", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_attitude2_t, rollspeed) }, \
         { "pitchspeed", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_attitude2_t, pitchspeed) }, \
         { "yawspeed", NULL, MAVLINK_TYPE_FLOAT, 0, 24, offsetof(mavlink_attitude2_t, yawspeed) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_ATTITUDE2 { \
    "ATTITUDE2", \
    8, \
    {  { "time_boot_ms", NULL, MAVLINK_TYPE_UINT32_T, 0, 0, offsetof(mavlink_attitude2_t, time_boot_ms) }, \
         { "attitude_id", NULL, MAVLINK_TYPE_UINT8_T, 0, 28, offsetof(mavlink_attitude2_t, attitude_id) }, \
         { "roll", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_attitude2_t, roll) }, \
         { "pitch", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_attitude2_t, pitch) }, \
         { "yaw", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_attitude2_t, yaw) }, \
         { "rollspeed", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_attitude2_t, rollspeed) }, \
         { "pitchspeed", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_attitude2_t, pitchspeed) }, \
         { "yawspeed", NULL, MAVLINK_TYPE_FLOAT, 0, 24, offsetof(mavlink_attitude2_t, yawspeed) }, \
         } \
}
#endif

/**
 * @brief Pack a attitude2 message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param time_boot_ms [ms] Timestamp (time since system boot).
 * @param attitude_id  Attitude Id.
 * @param roll [rad] Roll angle (-pi..+pi)
 * @param pitch [rad] Pitch angle (-pi..+pi)
 * @param yaw [rad] Yaw angle (-pi..+pi)
 * @param rollspeed [rad/s] Roll angular speed
 * @param pitchspeed [rad/s] Pitch angular speed
 * @param yawspeed [rad/s] Yaw angular speed
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_attitude2_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint32_t time_boot_ms, uint8_t attitude_id, float roll, float pitch, float yaw, float rollspeed, float pitchspeed, float yawspeed)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_ATTITUDE2_LEN];
    _mav_put_uint32_t(buf, 0, time_boot_ms);
    _mav_put_float(buf, 4, roll);
    _mav_put_float(buf, 8, pitch);
    _mav_put_float(buf, 12, yaw);
    _mav_put_float(buf, 16, rollspeed);
    _mav_put_float(buf, 20, pitchspeed);
    _mav_put_float(buf, 24, yawspeed);
    _mav_put_uint8_t(buf, 28, attitude_id);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_ATTITUDE2_LEN);
#else
    mavlink_attitude2_t packet;
    packet.time_boot_ms = time_boot_ms;
    packet.roll = roll;
    packet.pitch = pitch;
    packet.yaw = yaw;
    packet.rollspeed = rollspeed;
    packet.pitchspeed = pitchspeed;
    packet.yawspeed = yawspeed;
    packet.attitude_id = attitude_id;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_ATTITUDE2_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_ATTITUDE2;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_ATTITUDE2_MIN_LEN, MAVLINK_MSG_ID_ATTITUDE2_LEN, MAVLINK_MSG_ID_ATTITUDE2_CRC);
}

/**
 * @brief Pack a attitude2 message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param time_boot_ms [ms] Timestamp (time since system boot).
 * @param attitude_id  Attitude Id.
 * @param roll [rad] Roll angle (-pi..+pi)
 * @param pitch [rad] Pitch angle (-pi..+pi)
 * @param yaw [rad] Yaw angle (-pi..+pi)
 * @param rollspeed [rad/s] Roll angular speed
 * @param pitchspeed [rad/s] Pitch angular speed
 * @param yawspeed [rad/s] Yaw angular speed
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_attitude2_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint32_t time_boot_ms,uint8_t attitude_id,float roll,float pitch,float yaw,float rollspeed,float pitchspeed,float yawspeed)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_ATTITUDE2_LEN];
    _mav_put_uint32_t(buf, 0, time_boot_ms);
    _mav_put_float(buf, 4, roll);
    _mav_put_float(buf, 8, pitch);
    _mav_put_float(buf, 12, yaw);
    _mav_put_float(buf, 16, rollspeed);
    _mav_put_float(buf, 20, pitchspeed);
    _mav_put_float(buf, 24, yawspeed);
    _mav_put_uint8_t(buf, 28, attitude_id);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_ATTITUDE2_LEN);
#else
    mavlink_attitude2_t packet;
    packet.time_boot_ms = time_boot_ms;
    packet.roll = roll;
    packet.pitch = pitch;
    packet.yaw = yaw;
    packet.rollspeed = rollspeed;
    packet.pitchspeed = pitchspeed;
    packet.yawspeed = yawspeed;
    packet.attitude_id = attitude_id;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_ATTITUDE2_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_ATTITUDE2;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_ATTITUDE2_MIN_LEN, MAVLINK_MSG_ID_ATTITUDE2_LEN, MAVLINK_MSG_ID_ATTITUDE2_CRC);
}

/**
 * @brief Encode a attitude2 struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param attitude2 C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_attitude2_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_attitude2_t* attitude2)
{
    return mavlink_msg_attitude2_pack(system_id, component_id, msg, attitude2->time_boot_ms, attitude2->attitude_id, attitude2->roll, attitude2->pitch, attitude2->yaw, attitude2->rollspeed, attitude2->pitchspeed, attitude2->yawspeed);
}

/**
 * @brief Encode a attitude2 struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param attitude2 C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_attitude2_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_attitude2_t* attitude2)
{
    return mavlink_msg_attitude2_pack_chan(system_id, component_id, chan, msg, attitude2->time_boot_ms, attitude2->attitude_id, attitude2->roll, attitude2->pitch, attitude2->yaw, attitude2->rollspeed, attitude2->pitchspeed, attitude2->yawspeed);
}

/**
 * @brief Send a attitude2 message
 * @param chan MAVLink channel to send the message
 *
 * @param time_boot_ms [ms] Timestamp (time since system boot).
 * @param attitude_id  Attitude Id.
 * @param roll [rad] Roll angle (-pi..+pi)
 * @param pitch [rad] Pitch angle (-pi..+pi)
 * @param yaw [rad] Yaw angle (-pi..+pi)
 * @param rollspeed [rad/s] Roll angular speed
 * @param pitchspeed [rad/s] Pitch angular speed
 * @param yawspeed [rad/s] Yaw angular speed
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_attitude2_send(mavlink_channel_t chan, uint32_t time_boot_ms, uint8_t attitude_id, float roll, float pitch, float yaw, float rollspeed, float pitchspeed, float yawspeed)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_ATTITUDE2_LEN];
    _mav_put_uint32_t(buf, 0, time_boot_ms);
    _mav_put_float(buf, 4, roll);
    _mav_put_float(buf, 8, pitch);
    _mav_put_float(buf, 12, yaw);
    _mav_put_float(buf, 16, rollspeed);
    _mav_put_float(buf, 20, pitchspeed);
    _mav_put_float(buf, 24, yawspeed);
    _mav_put_uint8_t(buf, 28, attitude_id);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ATTITUDE2, buf, MAVLINK_MSG_ID_ATTITUDE2_MIN_LEN, MAVLINK_MSG_ID_ATTITUDE2_LEN, MAVLINK_MSG_ID_ATTITUDE2_CRC);
#else
    mavlink_attitude2_t packet;
    packet.time_boot_ms = time_boot_ms;
    packet.roll = roll;
    packet.pitch = pitch;
    packet.yaw = yaw;
    packet.rollspeed = rollspeed;
    packet.pitchspeed = pitchspeed;
    packet.yawspeed = yawspeed;
    packet.attitude_id = attitude_id;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ATTITUDE2, (const char *)&packet, MAVLINK_MSG_ID_ATTITUDE2_MIN_LEN, MAVLINK_MSG_ID_ATTITUDE2_LEN, MAVLINK_MSG_ID_ATTITUDE2_CRC);
#endif
}

/**
 * @brief Send a attitude2 message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_attitude2_send_struct(mavlink_channel_t chan, const mavlink_attitude2_t* attitude2)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_attitude2_send(chan, attitude2->time_boot_ms, attitude2->attitude_id, attitude2->roll, attitude2->pitch, attitude2->yaw, attitude2->rollspeed, attitude2->pitchspeed, attitude2->yawspeed);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ATTITUDE2, (const char *)attitude2, MAVLINK_MSG_ID_ATTITUDE2_MIN_LEN, MAVLINK_MSG_ID_ATTITUDE2_LEN, MAVLINK_MSG_ID_ATTITUDE2_CRC);
#endif
}

#if MAVLINK_MSG_ID_ATTITUDE2_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_attitude2_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint32_t time_boot_ms, uint8_t attitude_id, float roll, float pitch, float yaw, float rollspeed, float pitchspeed, float yawspeed)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint32_t(buf, 0, time_boot_ms);
    _mav_put_float(buf, 4, roll);
    _mav_put_float(buf, 8, pitch);
    _mav_put_float(buf, 12, yaw);
    _mav_put_float(buf, 16, rollspeed);
    _mav_put_float(buf, 20, pitchspeed);
    _mav_put_float(buf, 24, yawspeed);
    _mav_put_uint8_t(buf, 28, attitude_id);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ATTITUDE2, buf, MAVLINK_MSG_ID_ATTITUDE2_MIN_LEN, MAVLINK_MSG_ID_ATTITUDE2_LEN, MAVLINK_MSG_ID_ATTITUDE2_CRC);
#else
    mavlink_attitude2_t *packet = (mavlink_attitude2_t *)msgbuf;
    packet->time_boot_ms = time_boot_ms;
    packet->roll = roll;
    packet->pitch = pitch;
    packet->yaw = yaw;
    packet->rollspeed = rollspeed;
    packet->pitchspeed = pitchspeed;
    packet->yawspeed = yawspeed;
    packet->attitude_id = attitude_id;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ATTITUDE2, (const char *)packet, MAVLINK_MSG_ID_ATTITUDE2_MIN_LEN, MAVLINK_MSG_ID_ATTITUDE2_LEN, MAVLINK_MSG_ID_ATTITUDE2_CRC);
#endif
}
#endif

#endif

// MESSAGE ATTITUDE2 UNPACKING


/**
 * @brief Get field time_boot_ms from attitude2 message
 *
 * @return [ms] Timestamp (time since system boot).
 */
static inline uint32_t mavlink_msg_attitude2_get_time_boot_ms(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint32_t(msg,  0);
}

/**
 * @brief Get field attitude_id from attitude2 message
 *
 * @return  Attitude Id.
 */
static inline uint8_t mavlink_msg_attitude2_get_attitude_id(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  28);
}

/**
 * @brief Get field roll from attitude2 message
 *
 * @return [rad] Roll angle (-pi..+pi)
 */
static inline float mavlink_msg_attitude2_get_roll(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  4);
}

/**
 * @brief Get field pitch from attitude2 message
 *
 * @return [rad] Pitch angle (-pi..+pi)
 */
static inline float mavlink_msg_attitude2_get_pitch(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  8);
}

/**
 * @brief Get field yaw from attitude2 message
 *
 * @return [rad] Yaw angle (-pi..+pi)
 */
static inline float mavlink_msg_attitude2_get_yaw(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  12);
}

/**
 * @brief Get field rollspeed from attitude2 message
 *
 * @return [rad/s] Roll angular speed
 */
static inline float mavlink_msg_attitude2_get_rollspeed(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  16);
}

/**
 * @brief Get field pitchspeed from attitude2 message
 *
 * @return [rad/s] Pitch angular speed
 */
static inline float mavlink_msg_attitude2_get_pitchspeed(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  20);
}

/**
 * @brief Get field yawspeed from attitude2 message
 *
 * @return [rad/s] Yaw angular speed
 */
static inline float mavlink_msg_attitude2_get_yawspeed(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  24);
}

/**
 * @brief Decode a attitude2 message into a struct
 *
 * @param msg The message to decode
 * @param attitude2 C-struct to decode the message contents into
 */
static inline void mavlink_msg_attitude2_decode(const mavlink_message_t* msg, mavlink_attitude2_t* attitude2)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    attitude2->time_boot_ms = mavlink_msg_attitude2_get_time_boot_ms(msg);
    attitude2->roll = mavlink_msg_attitude2_get_roll(msg);
    attitude2->pitch = mavlink_msg_attitude2_get_pitch(msg);
    attitude2->yaw = mavlink_msg_attitude2_get_yaw(msg);
    attitude2->rollspeed = mavlink_msg_attitude2_get_rollspeed(msg);
    attitude2->pitchspeed = mavlink_msg_attitude2_get_pitchspeed(msg);
    attitude2->yawspeed = mavlink_msg_attitude2_get_yawspeed(msg);
    attitude2->attitude_id = mavlink_msg_attitude2_get_attitude_id(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_ATTITUDE2_LEN? msg->len : MAVLINK_MSG_ID_ATTITUDE2_LEN;
        memset(attitude2, 0, MAVLINK_MSG_ID_ATTITUDE2_LEN);
    memcpy(attitude2, _MAV_PAYLOAD(msg), len);
#endif
}
