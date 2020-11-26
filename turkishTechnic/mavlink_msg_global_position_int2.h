#pragma once
// MESSAGE GLOBAL_POSITION_INT2 PACKING

#define MAVLINK_MSG_ID_GLOBAL_POSITION_INT2 13002


typedef struct __mavlink_global_position_int2_t {
 uint32_t time_boot_ms; /*< [ms] Timestamp (time since system boot).*/
 int32_t lat; /*< [degE7] Latitude, expressed*/
 int32_t lon; /*< [degE7] Longitude, expressed*/
 int32_t alt; /*< [mm] Altitude (MSL). Note that virtually all GPS modules provide both WGS84 and MSL.*/
 int32_t relative_alt; /*< [mm] Altitude above ground*/
 int16_t vx; /*< [cm/s] Ground X Speed (Latitude, positive north)*/
 int16_t vy; /*< [cm/s] Ground Y Speed (Longitude, positive east)*/
 int16_t vz; /*< [cm/s] Ground Z Speed (Altitude, positive down)*/
 uint16_t hdg; /*< [cdeg] Vehicle heading (yaw angle), 0.0..359.99 degrees. If unknown, set to: UINT16_MAX*/
 uint8_t gpos_id; /*<  global position source id, 1:EKF2, 2=INSP, 3=Other FCS */
} mavlink_global_position_int2_t;

#define MAVLINK_MSG_ID_GLOBAL_POSITION_INT2_LEN 29
#define MAVLINK_MSG_ID_GLOBAL_POSITION_INT2_MIN_LEN 29
#define MAVLINK_MSG_ID_13002_LEN 29
#define MAVLINK_MSG_ID_13002_MIN_LEN 29

#define MAVLINK_MSG_ID_GLOBAL_POSITION_INT2_CRC 206
#define MAVLINK_MSG_ID_13002_CRC 206



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_GLOBAL_POSITION_INT2 { \
    13002, \
    "GLOBAL_POSITION_INT2", \
    10, \
    {  { "time_boot_ms", NULL, MAVLINK_TYPE_UINT32_T, 0, 0, offsetof(mavlink_global_position_int2_t, time_boot_ms) }, \
         { "gpos_id", NULL, MAVLINK_TYPE_UINT8_T, 0, 28, offsetof(mavlink_global_position_int2_t, gpos_id) }, \
         { "lat", NULL, MAVLINK_TYPE_INT32_T, 0, 4, offsetof(mavlink_global_position_int2_t, lat) }, \
         { "lon", NULL, MAVLINK_TYPE_INT32_T, 0, 8, offsetof(mavlink_global_position_int2_t, lon) }, \
         { "alt", NULL, MAVLINK_TYPE_INT32_T, 0, 12, offsetof(mavlink_global_position_int2_t, alt) }, \
         { "relative_alt", NULL, MAVLINK_TYPE_INT32_T, 0, 16, offsetof(mavlink_global_position_int2_t, relative_alt) }, \
         { "vx", NULL, MAVLINK_TYPE_INT16_T, 0, 20, offsetof(mavlink_global_position_int2_t, vx) }, \
         { "vy", NULL, MAVLINK_TYPE_INT16_T, 0, 22, offsetof(mavlink_global_position_int2_t, vy) }, \
         { "vz", NULL, MAVLINK_TYPE_INT16_T, 0, 24, offsetof(mavlink_global_position_int2_t, vz) }, \
         { "hdg", NULL, MAVLINK_TYPE_UINT16_T, 0, 26, offsetof(mavlink_global_position_int2_t, hdg) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_GLOBAL_POSITION_INT2 { \
    "GLOBAL_POSITION_INT2", \
    10, \
    {  { "time_boot_ms", NULL, MAVLINK_TYPE_UINT32_T, 0, 0, offsetof(mavlink_global_position_int2_t, time_boot_ms) }, \
         { "gpos_id", NULL, MAVLINK_TYPE_UINT8_T, 0, 28, offsetof(mavlink_global_position_int2_t, gpos_id) }, \
         { "lat", NULL, MAVLINK_TYPE_INT32_T, 0, 4, offsetof(mavlink_global_position_int2_t, lat) }, \
         { "lon", NULL, MAVLINK_TYPE_INT32_T, 0, 8, offsetof(mavlink_global_position_int2_t, lon) }, \
         { "alt", NULL, MAVLINK_TYPE_INT32_T, 0, 12, offsetof(mavlink_global_position_int2_t, alt) }, \
         { "relative_alt", NULL, MAVLINK_TYPE_INT32_T, 0, 16, offsetof(mavlink_global_position_int2_t, relative_alt) }, \
         { "vx", NULL, MAVLINK_TYPE_INT16_T, 0, 20, offsetof(mavlink_global_position_int2_t, vx) }, \
         { "vy", NULL, MAVLINK_TYPE_INT16_T, 0, 22, offsetof(mavlink_global_position_int2_t, vy) }, \
         { "vz", NULL, MAVLINK_TYPE_INT16_T, 0, 24, offsetof(mavlink_global_position_int2_t, vz) }, \
         { "hdg", NULL, MAVLINK_TYPE_UINT16_T, 0, 26, offsetof(mavlink_global_position_int2_t, hdg) }, \
         } \
}
#endif

/**
 * @brief Pack a global_position_int2 message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param time_boot_ms [ms] Timestamp (time since system boot).
 * @param gpos_id  global position source id, 1:EKF2, 2=INSP, 3=Other FCS 
 * @param lat [degE7] Latitude, expressed
 * @param lon [degE7] Longitude, expressed
 * @param alt [mm] Altitude (MSL). Note that virtually all GPS modules provide both WGS84 and MSL.
 * @param relative_alt [mm] Altitude above ground
 * @param vx [cm/s] Ground X Speed (Latitude, positive north)
 * @param vy [cm/s] Ground Y Speed (Longitude, positive east)
 * @param vz [cm/s] Ground Z Speed (Altitude, positive down)
 * @param hdg [cdeg] Vehicle heading (yaw angle), 0.0..359.99 degrees. If unknown, set to: UINT16_MAX
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_global_position_int2_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint32_t time_boot_ms, uint8_t gpos_id, int32_t lat, int32_t lon, int32_t alt, int32_t relative_alt, int16_t vx, int16_t vy, int16_t vz, uint16_t hdg)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_GLOBAL_POSITION_INT2_LEN];
    _mav_put_uint32_t(buf, 0, time_boot_ms);
    _mav_put_int32_t(buf, 4, lat);
    _mav_put_int32_t(buf, 8, lon);
    _mav_put_int32_t(buf, 12, alt);
    _mav_put_int32_t(buf, 16, relative_alt);
    _mav_put_int16_t(buf, 20, vx);
    _mav_put_int16_t(buf, 22, vy);
    _mav_put_int16_t(buf, 24, vz);
    _mav_put_uint16_t(buf, 26, hdg);
    _mav_put_uint8_t(buf, 28, gpos_id);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_GLOBAL_POSITION_INT2_LEN);
#else
    mavlink_global_position_int2_t packet;
    packet.time_boot_ms = time_boot_ms;
    packet.lat = lat;
    packet.lon = lon;
    packet.alt = alt;
    packet.relative_alt = relative_alt;
    packet.vx = vx;
    packet.vy = vy;
    packet.vz = vz;
    packet.hdg = hdg;
    packet.gpos_id = gpos_id;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_GLOBAL_POSITION_INT2_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_GLOBAL_POSITION_INT2;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_GLOBAL_POSITION_INT2_MIN_LEN, MAVLINK_MSG_ID_GLOBAL_POSITION_INT2_LEN, MAVLINK_MSG_ID_GLOBAL_POSITION_INT2_CRC);
}

/**
 * @brief Pack a global_position_int2 message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param time_boot_ms [ms] Timestamp (time since system boot).
 * @param gpos_id  global position source id, 1:EKF2, 2=INSP, 3=Other FCS 
 * @param lat [degE7] Latitude, expressed
 * @param lon [degE7] Longitude, expressed
 * @param alt [mm] Altitude (MSL). Note that virtually all GPS modules provide both WGS84 and MSL.
 * @param relative_alt [mm] Altitude above ground
 * @param vx [cm/s] Ground X Speed (Latitude, positive north)
 * @param vy [cm/s] Ground Y Speed (Longitude, positive east)
 * @param vz [cm/s] Ground Z Speed (Altitude, positive down)
 * @param hdg [cdeg] Vehicle heading (yaw angle), 0.0..359.99 degrees. If unknown, set to: UINT16_MAX
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_global_position_int2_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint32_t time_boot_ms,uint8_t gpos_id,int32_t lat,int32_t lon,int32_t alt,int32_t relative_alt,int16_t vx,int16_t vy,int16_t vz,uint16_t hdg)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_GLOBAL_POSITION_INT2_LEN];
    _mav_put_uint32_t(buf, 0, time_boot_ms);
    _mav_put_int32_t(buf, 4, lat);
    _mav_put_int32_t(buf, 8, lon);
    _mav_put_int32_t(buf, 12, alt);
    _mav_put_int32_t(buf, 16, relative_alt);
    _mav_put_int16_t(buf, 20, vx);
    _mav_put_int16_t(buf, 22, vy);
    _mav_put_int16_t(buf, 24, vz);
    _mav_put_uint16_t(buf, 26, hdg);
    _mav_put_uint8_t(buf, 28, gpos_id);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_GLOBAL_POSITION_INT2_LEN);
#else
    mavlink_global_position_int2_t packet;
    packet.time_boot_ms = time_boot_ms;
    packet.lat = lat;
    packet.lon = lon;
    packet.alt = alt;
    packet.relative_alt = relative_alt;
    packet.vx = vx;
    packet.vy = vy;
    packet.vz = vz;
    packet.hdg = hdg;
    packet.gpos_id = gpos_id;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_GLOBAL_POSITION_INT2_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_GLOBAL_POSITION_INT2;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_GLOBAL_POSITION_INT2_MIN_LEN, MAVLINK_MSG_ID_GLOBAL_POSITION_INT2_LEN, MAVLINK_MSG_ID_GLOBAL_POSITION_INT2_CRC);
}

/**
 * @brief Encode a global_position_int2 struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param global_position_int2 C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_global_position_int2_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_global_position_int2_t* global_position_int2)
{
    return mavlink_msg_global_position_int2_pack(system_id, component_id, msg, global_position_int2->time_boot_ms, global_position_int2->gpos_id, global_position_int2->lat, global_position_int2->lon, global_position_int2->alt, global_position_int2->relative_alt, global_position_int2->vx, global_position_int2->vy, global_position_int2->vz, global_position_int2->hdg);
}

/**
 * @brief Encode a global_position_int2 struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param global_position_int2 C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_global_position_int2_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_global_position_int2_t* global_position_int2)
{
    return mavlink_msg_global_position_int2_pack_chan(system_id, component_id, chan, msg, global_position_int2->time_boot_ms, global_position_int2->gpos_id, global_position_int2->lat, global_position_int2->lon, global_position_int2->alt, global_position_int2->relative_alt, global_position_int2->vx, global_position_int2->vy, global_position_int2->vz, global_position_int2->hdg);
}

/**
 * @brief Send a global_position_int2 message
 * @param chan MAVLink channel to send the message
 *
 * @param time_boot_ms [ms] Timestamp (time since system boot).
 * @param gpos_id  global position source id, 1:EKF2, 2=INSP, 3=Other FCS 
 * @param lat [degE7] Latitude, expressed
 * @param lon [degE7] Longitude, expressed
 * @param alt [mm] Altitude (MSL). Note that virtually all GPS modules provide both WGS84 and MSL.
 * @param relative_alt [mm] Altitude above ground
 * @param vx [cm/s] Ground X Speed (Latitude, positive north)
 * @param vy [cm/s] Ground Y Speed (Longitude, positive east)
 * @param vz [cm/s] Ground Z Speed (Altitude, positive down)
 * @param hdg [cdeg] Vehicle heading (yaw angle), 0.0..359.99 degrees. If unknown, set to: UINT16_MAX
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_global_position_int2_send(mavlink_channel_t chan, uint32_t time_boot_ms, uint8_t gpos_id, int32_t lat, int32_t lon, int32_t alt, int32_t relative_alt, int16_t vx, int16_t vy, int16_t vz, uint16_t hdg)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_GLOBAL_POSITION_INT2_LEN];
    _mav_put_uint32_t(buf, 0, time_boot_ms);
    _mav_put_int32_t(buf, 4, lat);
    _mav_put_int32_t(buf, 8, lon);
    _mav_put_int32_t(buf, 12, alt);
    _mav_put_int32_t(buf, 16, relative_alt);
    _mav_put_int16_t(buf, 20, vx);
    _mav_put_int16_t(buf, 22, vy);
    _mav_put_int16_t(buf, 24, vz);
    _mav_put_uint16_t(buf, 26, hdg);
    _mav_put_uint8_t(buf, 28, gpos_id);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_GLOBAL_POSITION_INT2, buf, MAVLINK_MSG_ID_GLOBAL_POSITION_INT2_MIN_LEN, MAVLINK_MSG_ID_GLOBAL_POSITION_INT2_LEN, MAVLINK_MSG_ID_GLOBAL_POSITION_INT2_CRC);
#else
    mavlink_global_position_int2_t packet;
    packet.time_boot_ms = time_boot_ms;
    packet.lat = lat;
    packet.lon = lon;
    packet.alt = alt;
    packet.relative_alt = relative_alt;
    packet.vx = vx;
    packet.vy = vy;
    packet.vz = vz;
    packet.hdg = hdg;
    packet.gpos_id = gpos_id;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_GLOBAL_POSITION_INT2, (const char *)&packet, MAVLINK_MSG_ID_GLOBAL_POSITION_INT2_MIN_LEN, MAVLINK_MSG_ID_GLOBAL_POSITION_INT2_LEN, MAVLINK_MSG_ID_GLOBAL_POSITION_INT2_CRC);
#endif
}

/**
 * @brief Send a global_position_int2 message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_global_position_int2_send_struct(mavlink_channel_t chan, const mavlink_global_position_int2_t* global_position_int2)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_global_position_int2_send(chan, global_position_int2->time_boot_ms, global_position_int2->gpos_id, global_position_int2->lat, global_position_int2->lon, global_position_int2->alt, global_position_int2->relative_alt, global_position_int2->vx, global_position_int2->vy, global_position_int2->vz, global_position_int2->hdg);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_GLOBAL_POSITION_INT2, (const char *)global_position_int2, MAVLINK_MSG_ID_GLOBAL_POSITION_INT2_MIN_LEN, MAVLINK_MSG_ID_GLOBAL_POSITION_INT2_LEN, MAVLINK_MSG_ID_GLOBAL_POSITION_INT2_CRC);
#endif
}

#if MAVLINK_MSG_ID_GLOBAL_POSITION_INT2_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_global_position_int2_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint32_t time_boot_ms, uint8_t gpos_id, int32_t lat, int32_t lon, int32_t alt, int32_t relative_alt, int16_t vx, int16_t vy, int16_t vz, uint16_t hdg)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint32_t(buf, 0, time_boot_ms);
    _mav_put_int32_t(buf, 4, lat);
    _mav_put_int32_t(buf, 8, lon);
    _mav_put_int32_t(buf, 12, alt);
    _mav_put_int32_t(buf, 16, relative_alt);
    _mav_put_int16_t(buf, 20, vx);
    _mav_put_int16_t(buf, 22, vy);
    _mav_put_int16_t(buf, 24, vz);
    _mav_put_uint16_t(buf, 26, hdg);
    _mav_put_uint8_t(buf, 28, gpos_id);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_GLOBAL_POSITION_INT2, buf, MAVLINK_MSG_ID_GLOBAL_POSITION_INT2_MIN_LEN, MAVLINK_MSG_ID_GLOBAL_POSITION_INT2_LEN, MAVLINK_MSG_ID_GLOBAL_POSITION_INT2_CRC);
#else
    mavlink_global_position_int2_t *packet = (mavlink_global_position_int2_t *)msgbuf;
    packet->time_boot_ms = time_boot_ms;
    packet->lat = lat;
    packet->lon = lon;
    packet->alt = alt;
    packet->relative_alt = relative_alt;
    packet->vx = vx;
    packet->vy = vy;
    packet->vz = vz;
    packet->hdg = hdg;
    packet->gpos_id = gpos_id;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_GLOBAL_POSITION_INT2, (const char *)packet, MAVLINK_MSG_ID_GLOBAL_POSITION_INT2_MIN_LEN, MAVLINK_MSG_ID_GLOBAL_POSITION_INT2_LEN, MAVLINK_MSG_ID_GLOBAL_POSITION_INT2_CRC);
#endif
}
#endif

#endif

// MESSAGE GLOBAL_POSITION_INT2 UNPACKING


/**
 * @brief Get field time_boot_ms from global_position_int2 message
 *
 * @return [ms] Timestamp (time since system boot).
 */
static inline uint32_t mavlink_msg_global_position_int2_get_time_boot_ms(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint32_t(msg,  0);
}

/**
 * @brief Get field gpos_id from global_position_int2 message
 *
 * @return  global position source id, 1:EKF2, 2=INSP, 3=Other FCS 
 */
static inline uint8_t mavlink_msg_global_position_int2_get_gpos_id(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  28);
}

/**
 * @brief Get field lat from global_position_int2 message
 *
 * @return [degE7] Latitude, expressed
 */
static inline int32_t mavlink_msg_global_position_int2_get_lat(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int32_t(msg,  4);
}

/**
 * @brief Get field lon from global_position_int2 message
 *
 * @return [degE7] Longitude, expressed
 */
static inline int32_t mavlink_msg_global_position_int2_get_lon(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int32_t(msg,  8);
}

/**
 * @brief Get field alt from global_position_int2 message
 *
 * @return [mm] Altitude (MSL). Note that virtually all GPS modules provide both WGS84 and MSL.
 */
static inline int32_t mavlink_msg_global_position_int2_get_alt(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int32_t(msg,  12);
}

/**
 * @brief Get field relative_alt from global_position_int2 message
 *
 * @return [mm] Altitude above ground
 */
static inline int32_t mavlink_msg_global_position_int2_get_relative_alt(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int32_t(msg,  16);
}

/**
 * @brief Get field vx from global_position_int2 message
 *
 * @return [cm/s] Ground X Speed (Latitude, positive north)
 */
static inline int16_t mavlink_msg_global_position_int2_get_vx(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int16_t(msg,  20);
}

/**
 * @brief Get field vy from global_position_int2 message
 *
 * @return [cm/s] Ground Y Speed (Longitude, positive east)
 */
static inline int16_t mavlink_msg_global_position_int2_get_vy(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int16_t(msg,  22);
}

/**
 * @brief Get field vz from global_position_int2 message
 *
 * @return [cm/s] Ground Z Speed (Altitude, positive down)
 */
static inline int16_t mavlink_msg_global_position_int2_get_vz(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int16_t(msg,  24);
}

/**
 * @brief Get field hdg from global_position_int2 message
 *
 * @return [cdeg] Vehicle heading (yaw angle), 0.0..359.99 degrees. If unknown, set to: UINT16_MAX
 */
static inline uint16_t mavlink_msg_global_position_int2_get_hdg(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  26);
}

/**
 * @brief Decode a global_position_int2 message into a struct
 *
 * @param msg The message to decode
 * @param global_position_int2 C-struct to decode the message contents into
 */
static inline void mavlink_msg_global_position_int2_decode(const mavlink_message_t* msg, mavlink_global_position_int2_t* global_position_int2)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    global_position_int2->time_boot_ms = mavlink_msg_global_position_int2_get_time_boot_ms(msg);
    global_position_int2->lat = mavlink_msg_global_position_int2_get_lat(msg);
    global_position_int2->lon = mavlink_msg_global_position_int2_get_lon(msg);
    global_position_int2->alt = mavlink_msg_global_position_int2_get_alt(msg);
    global_position_int2->relative_alt = mavlink_msg_global_position_int2_get_relative_alt(msg);
    global_position_int2->vx = mavlink_msg_global_position_int2_get_vx(msg);
    global_position_int2->vy = mavlink_msg_global_position_int2_get_vy(msg);
    global_position_int2->vz = mavlink_msg_global_position_int2_get_vz(msg);
    global_position_int2->hdg = mavlink_msg_global_position_int2_get_hdg(msg);
    global_position_int2->gpos_id = mavlink_msg_global_position_int2_get_gpos_id(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_GLOBAL_POSITION_INT2_LEN? msg->len : MAVLINK_MSG_ID_GLOBAL_POSITION_INT2_LEN;
        memset(global_position_int2, 0, MAVLINK_MSG_ID_GLOBAL_POSITION_INT2_LEN);
    memcpy(global_position_int2, _MAV_PAYLOAD(msg), len);
#endif
}
