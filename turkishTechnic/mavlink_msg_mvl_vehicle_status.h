#pragma once
// MESSAGE MVL_VEHICLE_STATUS PACKING

#define MAVLINK_MSG_ID_MVL_VEHICLE_STATUS 13000

MAVPACKED(
typedef struct __mavlink_mvl_vehicle_status_t {
 uint64_t timestamp; /*<  Timestamp iste*/
 uint32_t onboard_control_sensors_present; /*<  present sensors*/
 uint32_t onboard_control_sensors_enabled; /*<  enabled sensors*/
 uint32_t onboard_control_sensors_health; /*<  sensor health*/
 float arspd_check_level; /*<  airspeed check level*/
 float load_factor_ratio; /*<  load factor ratio*/
 uint8_t nav_state; /*<  Navigator state*/
 uint8_t arming_state; /*<  Arming state*/
 uint8_t hil_state; /*<  hil state*/
 uint8_t red_state; /*<  redundancy state*/
 uint8_t failsafe; /*<  failsafe*/
 uint8_t system_type; /*<  system_type*/
 uint8_t vehicle_type; /*<  vehicle_type*/
 uint8_t is_vtol; /*<  is_vtol*/
 uint8_t vtol_fw_permanent_stab; /*<  vtol_fw_permanent_stab*/
 uint8_t in_transition_mode; /*<  in_transition_mode*/
 uint8_t in_transition_to_fw; /*<  in_transition_to_fw*/
 uint8_t rc_signal_lost; /*<  rc_signal_lost*/
 uint8_t rc_input_mode; /*<  rc_input_mode*/
 uint8_t data_link_lost; /*<  data_link_lost*/
 uint8_t data_link_lost_counter; /*<  data_link_lost_counter*/
 uint8_t high_latency_data_link_lost; /*<  high_latency_data_link_lost*/
 uint8_t engine_failure; /*<  engine_failure*/
 uint8_t mission_failure; /*<  mission_failure*/
 uint8_t failure_detector_status; /*<  failure_detector_status*/
 uint8_t aspd_check_failing; /*<  aspd_check_failing*/
 uint8_t aspd_fault_declared; /*<  aspd_fault_declared*/
 uint8_t aspd_use_inhibit; /*<  aspd_use_inhibit*/
 uint8_t aspd_fail_rtl; /*<  aspd_fail_rtl*/
}) mavlink_mvl_vehicle_status_t;

#define MAVLINK_MSG_ID_MVL_VEHICLE_STATUS_LEN 51
#define MAVLINK_MSG_ID_MVL_VEHICLE_STATUS_MIN_LEN 51
#define MAVLINK_MSG_ID_13000_LEN 51
#define MAVLINK_MSG_ID_13000_MIN_LEN 51

#define MAVLINK_MSG_ID_MVL_VEHICLE_STATUS_CRC 68
#define MAVLINK_MSG_ID_13000_CRC 68



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_MVL_VEHICLE_STATUS { \
    13000, \
    "MVL_VEHICLE_STATUS", \
    29, \
    {  { "timestamp", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_mvl_vehicle_status_t, timestamp) }, \
         { "onboard_control_sensors_present", NULL, MAVLINK_TYPE_UINT32_T, 0, 8, offsetof(mavlink_mvl_vehicle_status_t, onboard_control_sensors_present) }, \
         { "onboard_control_sensors_enabled", NULL, MAVLINK_TYPE_UINT32_T, 0, 12, offsetof(mavlink_mvl_vehicle_status_t, onboard_control_sensors_enabled) }, \
         { "onboard_control_sensors_health", NULL, MAVLINK_TYPE_UINT32_T, 0, 16, offsetof(mavlink_mvl_vehicle_status_t, onboard_control_sensors_health) }, \
         { "arspd_check_level", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_mvl_vehicle_status_t, arspd_check_level) }, \
         { "load_factor_ratio", NULL, MAVLINK_TYPE_FLOAT, 0, 24, offsetof(mavlink_mvl_vehicle_status_t, load_factor_ratio) }, \
         { "nav_state", NULL, MAVLINK_TYPE_UINT8_T, 0, 28, offsetof(mavlink_mvl_vehicle_status_t, nav_state) }, \
         { "arming_state", NULL, MAVLINK_TYPE_UINT8_T, 0, 29, offsetof(mavlink_mvl_vehicle_status_t, arming_state) }, \
         { "hil_state", NULL, MAVLINK_TYPE_UINT8_T, 0, 30, offsetof(mavlink_mvl_vehicle_status_t, hil_state) }, \
         { "red_state", NULL, MAVLINK_TYPE_UINT8_T, 0, 31, offsetof(mavlink_mvl_vehicle_status_t, red_state) }, \
         { "failsafe", NULL, MAVLINK_TYPE_UINT8_T, 0, 32, offsetof(mavlink_mvl_vehicle_status_t, failsafe) }, \
         { "system_type", NULL, MAVLINK_TYPE_UINT8_T, 0, 33, offsetof(mavlink_mvl_vehicle_status_t, system_type) }, \
         { "vehicle_type", NULL, MAVLINK_TYPE_UINT8_T, 0, 34, offsetof(mavlink_mvl_vehicle_status_t, vehicle_type) }, \
         { "is_vtol", NULL, MAVLINK_TYPE_UINT8_T, 0, 35, offsetof(mavlink_mvl_vehicle_status_t, is_vtol) }, \
         { "vtol_fw_permanent_stab", NULL, MAVLINK_TYPE_UINT8_T, 0, 36, offsetof(mavlink_mvl_vehicle_status_t, vtol_fw_permanent_stab) }, \
         { "in_transition_mode", NULL, MAVLINK_TYPE_UINT8_T, 0, 37, offsetof(mavlink_mvl_vehicle_status_t, in_transition_mode) }, \
         { "in_transition_to_fw", NULL, MAVLINK_TYPE_UINT8_T, 0, 38, offsetof(mavlink_mvl_vehicle_status_t, in_transition_to_fw) }, \
         { "rc_signal_lost", NULL, MAVLINK_TYPE_UINT8_T, 0, 39, offsetof(mavlink_mvl_vehicle_status_t, rc_signal_lost) }, \
         { "rc_input_mode", NULL, MAVLINK_TYPE_UINT8_T, 0, 40, offsetof(mavlink_mvl_vehicle_status_t, rc_input_mode) }, \
         { "data_link_lost", NULL, MAVLINK_TYPE_UINT8_T, 0, 41, offsetof(mavlink_mvl_vehicle_status_t, data_link_lost) }, \
         { "data_link_lost_counter", NULL, MAVLINK_TYPE_UINT8_T, 0, 42, offsetof(mavlink_mvl_vehicle_status_t, data_link_lost_counter) }, \
         { "high_latency_data_link_lost", NULL, MAVLINK_TYPE_UINT8_T, 0, 43, offsetof(mavlink_mvl_vehicle_status_t, high_latency_data_link_lost) }, \
         { "engine_failure", NULL, MAVLINK_TYPE_UINT8_T, 0, 44, offsetof(mavlink_mvl_vehicle_status_t, engine_failure) }, \
         { "mission_failure", NULL, MAVLINK_TYPE_UINT8_T, 0, 45, offsetof(mavlink_mvl_vehicle_status_t, mission_failure) }, \
         { "failure_detector_status", NULL, MAVLINK_TYPE_UINT8_T, 0, 46, offsetof(mavlink_mvl_vehicle_status_t, failure_detector_status) }, \
         { "aspd_check_failing", NULL, MAVLINK_TYPE_UINT8_T, 0, 47, offsetof(mavlink_mvl_vehicle_status_t, aspd_check_failing) }, \
         { "aspd_fault_declared", NULL, MAVLINK_TYPE_UINT8_T, 0, 48, offsetof(mavlink_mvl_vehicle_status_t, aspd_fault_declared) }, \
         { "aspd_use_inhibit", NULL, MAVLINK_TYPE_UINT8_T, 0, 49, offsetof(mavlink_mvl_vehicle_status_t, aspd_use_inhibit) }, \
         { "aspd_fail_rtl", NULL, MAVLINK_TYPE_UINT8_T, 0, 50, offsetof(mavlink_mvl_vehicle_status_t, aspd_fail_rtl) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_MVL_VEHICLE_STATUS { \
    "MVL_VEHICLE_STATUS", \
    29, \
    {  { "timestamp", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_mvl_vehicle_status_t, timestamp) }, \
         { "onboard_control_sensors_present", NULL, MAVLINK_TYPE_UINT32_T, 0, 8, offsetof(mavlink_mvl_vehicle_status_t, onboard_control_sensors_present) }, \
         { "onboard_control_sensors_enabled", NULL, MAVLINK_TYPE_UINT32_T, 0, 12, offsetof(mavlink_mvl_vehicle_status_t, onboard_control_sensors_enabled) }, \
         { "onboard_control_sensors_health", NULL, MAVLINK_TYPE_UINT32_T, 0, 16, offsetof(mavlink_mvl_vehicle_status_t, onboard_control_sensors_health) }, \
         { "arspd_check_level", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_mvl_vehicle_status_t, arspd_check_level) }, \
         { "load_factor_ratio", NULL, MAVLINK_TYPE_FLOAT, 0, 24, offsetof(mavlink_mvl_vehicle_status_t, load_factor_ratio) }, \
         { "nav_state", NULL, MAVLINK_TYPE_UINT8_T, 0, 28, offsetof(mavlink_mvl_vehicle_status_t, nav_state) }, \
         { "arming_state", NULL, MAVLINK_TYPE_UINT8_T, 0, 29, offsetof(mavlink_mvl_vehicle_status_t, arming_state) }, \
         { "hil_state", NULL, MAVLINK_TYPE_UINT8_T, 0, 30, offsetof(mavlink_mvl_vehicle_status_t, hil_state) }, \
         { "red_state", NULL, MAVLINK_TYPE_UINT8_T, 0, 31, offsetof(mavlink_mvl_vehicle_status_t, red_state) }, \
         { "failsafe", NULL, MAVLINK_TYPE_UINT8_T, 0, 32, offsetof(mavlink_mvl_vehicle_status_t, failsafe) }, \
         { "system_type", NULL, MAVLINK_TYPE_UINT8_T, 0, 33, offsetof(mavlink_mvl_vehicle_status_t, system_type) }, \
         { "vehicle_type", NULL, MAVLINK_TYPE_UINT8_T, 0, 34, offsetof(mavlink_mvl_vehicle_status_t, vehicle_type) }, \
         { "is_vtol", NULL, MAVLINK_TYPE_UINT8_T, 0, 35, offsetof(mavlink_mvl_vehicle_status_t, is_vtol) }, \
         { "vtol_fw_permanent_stab", NULL, MAVLINK_TYPE_UINT8_T, 0, 36, offsetof(mavlink_mvl_vehicle_status_t, vtol_fw_permanent_stab) }, \
         { "in_transition_mode", NULL, MAVLINK_TYPE_UINT8_T, 0, 37, offsetof(mavlink_mvl_vehicle_status_t, in_transition_mode) }, \
         { "in_transition_to_fw", NULL, MAVLINK_TYPE_UINT8_T, 0, 38, offsetof(mavlink_mvl_vehicle_status_t, in_transition_to_fw) }, \
         { "rc_signal_lost", NULL, MAVLINK_TYPE_UINT8_T, 0, 39, offsetof(mavlink_mvl_vehicle_status_t, rc_signal_lost) }, \
         { "rc_input_mode", NULL, MAVLINK_TYPE_UINT8_T, 0, 40, offsetof(mavlink_mvl_vehicle_status_t, rc_input_mode) }, \
         { "data_link_lost", NULL, MAVLINK_TYPE_UINT8_T, 0, 41, offsetof(mavlink_mvl_vehicle_status_t, data_link_lost) }, \
         { "data_link_lost_counter", NULL, MAVLINK_TYPE_UINT8_T, 0, 42, offsetof(mavlink_mvl_vehicle_status_t, data_link_lost_counter) }, \
         { "high_latency_data_link_lost", NULL, MAVLINK_TYPE_UINT8_T, 0, 43, offsetof(mavlink_mvl_vehicle_status_t, high_latency_data_link_lost) }, \
         { "engine_failure", NULL, MAVLINK_TYPE_UINT8_T, 0, 44, offsetof(mavlink_mvl_vehicle_status_t, engine_failure) }, \
         { "mission_failure", NULL, MAVLINK_TYPE_UINT8_T, 0, 45, offsetof(mavlink_mvl_vehicle_status_t, mission_failure) }, \
         { "failure_detector_status", NULL, MAVLINK_TYPE_UINT8_T, 0, 46, offsetof(mavlink_mvl_vehicle_status_t, failure_detector_status) }, \
         { "aspd_check_failing", NULL, MAVLINK_TYPE_UINT8_T, 0, 47, offsetof(mavlink_mvl_vehicle_status_t, aspd_check_failing) }, \
         { "aspd_fault_declared", NULL, MAVLINK_TYPE_UINT8_T, 0, 48, offsetof(mavlink_mvl_vehicle_status_t, aspd_fault_declared) }, \
         { "aspd_use_inhibit", NULL, MAVLINK_TYPE_UINT8_T, 0, 49, offsetof(mavlink_mvl_vehicle_status_t, aspd_use_inhibit) }, \
         { "aspd_fail_rtl", NULL, MAVLINK_TYPE_UINT8_T, 0, 50, offsetof(mavlink_mvl_vehicle_status_t, aspd_fail_rtl) }, \
         } \
}
#endif

/**
 * @brief Pack a mvl_vehicle_status message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param timestamp  Timestamp iste
 * @param onboard_control_sensors_present  present sensors
 * @param onboard_control_sensors_enabled  enabled sensors
 * @param onboard_control_sensors_health  sensor health
 * @param arspd_check_level  airspeed check level
 * @param load_factor_ratio  load factor ratio
 * @param nav_state  Navigator state
 * @param arming_state  Arming state
 * @param hil_state  hil state
 * @param red_state  redundancy state
 * @param failsafe  failsafe
 * @param system_type  system_type
 * @param vehicle_type  vehicle_type
 * @param is_vtol  is_vtol
 * @param vtol_fw_permanent_stab  vtol_fw_permanent_stab
 * @param in_transition_mode  in_transition_mode
 * @param in_transition_to_fw  in_transition_to_fw
 * @param rc_signal_lost  rc_signal_lost
 * @param rc_input_mode  rc_input_mode
 * @param data_link_lost  data_link_lost
 * @param data_link_lost_counter  data_link_lost_counter
 * @param high_latency_data_link_lost  high_latency_data_link_lost
 * @param engine_failure  engine_failure
 * @param mission_failure  mission_failure
 * @param failure_detector_status  failure_detector_status
 * @param aspd_check_failing  aspd_check_failing
 * @param aspd_fault_declared  aspd_fault_declared
 * @param aspd_use_inhibit  aspd_use_inhibit
 * @param aspd_fail_rtl  aspd_fail_rtl
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_mvl_vehicle_status_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint64_t timestamp, uint32_t onboard_control_sensors_present, uint32_t onboard_control_sensors_enabled, uint32_t onboard_control_sensors_health, float arspd_check_level, float load_factor_ratio, uint8_t nav_state, uint8_t arming_state, uint8_t hil_state, uint8_t red_state, uint8_t failsafe, uint8_t system_type, uint8_t vehicle_type, uint8_t is_vtol, uint8_t vtol_fw_permanent_stab, uint8_t in_transition_mode, uint8_t in_transition_to_fw, uint8_t rc_signal_lost, uint8_t rc_input_mode, uint8_t data_link_lost, uint8_t data_link_lost_counter, uint8_t high_latency_data_link_lost, uint8_t engine_failure, uint8_t mission_failure, uint8_t failure_detector_status, uint8_t aspd_check_failing, uint8_t aspd_fault_declared, uint8_t aspd_use_inhibit, uint8_t aspd_fail_rtl)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_MVL_VEHICLE_STATUS_LEN];
    _mav_put_uint64_t(buf, 0, timestamp);
    _mav_put_uint32_t(buf, 8, onboard_control_sensors_present);
    _mav_put_uint32_t(buf, 12, onboard_control_sensors_enabled);
    _mav_put_uint32_t(buf, 16, onboard_control_sensors_health);
    _mav_put_float(buf, 20, arspd_check_level);
    _mav_put_float(buf, 24, load_factor_ratio);
    _mav_put_uint8_t(buf, 28, nav_state);
    _mav_put_uint8_t(buf, 29, arming_state);
    _mav_put_uint8_t(buf, 30, hil_state);
    _mav_put_uint8_t(buf, 31, red_state);
    _mav_put_uint8_t(buf, 32, failsafe);
    _mav_put_uint8_t(buf, 33, system_type);
    _mav_put_uint8_t(buf, 34, vehicle_type);
    _mav_put_uint8_t(buf, 35, is_vtol);
    _mav_put_uint8_t(buf, 36, vtol_fw_permanent_stab);
    _mav_put_uint8_t(buf, 37, in_transition_mode);
    _mav_put_uint8_t(buf, 38, in_transition_to_fw);
    _mav_put_uint8_t(buf, 39, rc_signal_lost);
    _mav_put_uint8_t(buf, 40, rc_input_mode);
    _mav_put_uint8_t(buf, 41, data_link_lost);
    _mav_put_uint8_t(buf, 42, data_link_lost_counter);
    _mav_put_uint8_t(buf, 43, high_latency_data_link_lost);
    _mav_put_uint8_t(buf, 44, engine_failure);
    _mav_put_uint8_t(buf, 45, mission_failure);
    _mav_put_uint8_t(buf, 46, failure_detector_status);
    _mav_put_uint8_t(buf, 47, aspd_check_failing);
    _mav_put_uint8_t(buf, 48, aspd_fault_declared);
    _mav_put_uint8_t(buf, 49, aspd_use_inhibit);
    _mav_put_uint8_t(buf, 50, aspd_fail_rtl);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_MVL_VEHICLE_STATUS_LEN);
#else
    mavlink_mvl_vehicle_status_t packet;
    packet.timestamp = timestamp;
    packet.onboard_control_sensors_present = onboard_control_sensors_present;
    packet.onboard_control_sensors_enabled = onboard_control_sensors_enabled;
    packet.onboard_control_sensors_health = onboard_control_sensors_health;
    packet.arspd_check_level = arspd_check_level;
    packet.load_factor_ratio = load_factor_ratio;
    packet.nav_state = nav_state;
    packet.arming_state = arming_state;
    packet.hil_state = hil_state;
    packet.red_state = red_state;
    packet.failsafe = failsafe;
    packet.system_type = system_type;
    packet.vehicle_type = vehicle_type;
    packet.is_vtol = is_vtol;
    packet.vtol_fw_permanent_stab = vtol_fw_permanent_stab;
    packet.in_transition_mode = in_transition_mode;
    packet.in_transition_to_fw = in_transition_to_fw;
    packet.rc_signal_lost = rc_signal_lost;
    packet.rc_input_mode = rc_input_mode;
    packet.data_link_lost = data_link_lost;
    packet.data_link_lost_counter = data_link_lost_counter;
    packet.high_latency_data_link_lost = high_latency_data_link_lost;
    packet.engine_failure = engine_failure;
    packet.mission_failure = mission_failure;
    packet.failure_detector_status = failure_detector_status;
    packet.aspd_check_failing = aspd_check_failing;
    packet.aspd_fault_declared = aspd_fault_declared;
    packet.aspd_use_inhibit = aspd_use_inhibit;
    packet.aspd_fail_rtl = aspd_fail_rtl;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_MVL_VEHICLE_STATUS_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_MVL_VEHICLE_STATUS;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_MVL_VEHICLE_STATUS_MIN_LEN, MAVLINK_MSG_ID_MVL_VEHICLE_STATUS_LEN, MAVLINK_MSG_ID_MVL_VEHICLE_STATUS_CRC);
}

/**
 * @brief Pack a mvl_vehicle_status message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param timestamp  Timestamp iste
 * @param onboard_control_sensors_present  present sensors
 * @param onboard_control_sensors_enabled  enabled sensors
 * @param onboard_control_sensors_health  sensor health
 * @param arspd_check_level  airspeed check level
 * @param load_factor_ratio  load factor ratio
 * @param nav_state  Navigator state
 * @param arming_state  Arming state
 * @param hil_state  hil state
 * @param red_state  redundancy state
 * @param failsafe  failsafe
 * @param system_type  system_type
 * @param vehicle_type  vehicle_type
 * @param is_vtol  is_vtol
 * @param vtol_fw_permanent_stab  vtol_fw_permanent_stab
 * @param in_transition_mode  in_transition_mode
 * @param in_transition_to_fw  in_transition_to_fw
 * @param rc_signal_lost  rc_signal_lost
 * @param rc_input_mode  rc_input_mode
 * @param data_link_lost  data_link_lost
 * @param data_link_lost_counter  data_link_lost_counter
 * @param high_latency_data_link_lost  high_latency_data_link_lost
 * @param engine_failure  engine_failure
 * @param mission_failure  mission_failure
 * @param failure_detector_status  failure_detector_status
 * @param aspd_check_failing  aspd_check_failing
 * @param aspd_fault_declared  aspd_fault_declared
 * @param aspd_use_inhibit  aspd_use_inhibit
 * @param aspd_fail_rtl  aspd_fail_rtl
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_mvl_vehicle_status_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint64_t timestamp,uint32_t onboard_control_sensors_present,uint32_t onboard_control_sensors_enabled,uint32_t onboard_control_sensors_health,float arspd_check_level,float load_factor_ratio,uint8_t nav_state,uint8_t arming_state,uint8_t hil_state,uint8_t red_state,uint8_t failsafe,uint8_t system_type,uint8_t vehicle_type,uint8_t is_vtol,uint8_t vtol_fw_permanent_stab,uint8_t in_transition_mode,uint8_t in_transition_to_fw,uint8_t rc_signal_lost,uint8_t rc_input_mode,uint8_t data_link_lost,uint8_t data_link_lost_counter,uint8_t high_latency_data_link_lost,uint8_t engine_failure,uint8_t mission_failure,uint8_t failure_detector_status,uint8_t aspd_check_failing,uint8_t aspd_fault_declared,uint8_t aspd_use_inhibit,uint8_t aspd_fail_rtl)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_MVL_VEHICLE_STATUS_LEN];
    _mav_put_uint64_t(buf, 0, timestamp);
    _mav_put_uint32_t(buf, 8, onboard_control_sensors_present);
    _mav_put_uint32_t(buf, 12, onboard_control_sensors_enabled);
    _mav_put_uint32_t(buf, 16, onboard_control_sensors_health);
    _mav_put_float(buf, 20, arspd_check_level);
    _mav_put_float(buf, 24, load_factor_ratio);
    _mav_put_uint8_t(buf, 28, nav_state);
    _mav_put_uint8_t(buf, 29, arming_state);
    _mav_put_uint8_t(buf, 30, hil_state);
    _mav_put_uint8_t(buf, 31, red_state);
    _mav_put_uint8_t(buf, 32, failsafe);
    _mav_put_uint8_t(buf, 33, system_type);
    _mav_put_uint8_t(buf, 34, vehicle_type);
    _mav_put_uint8_t(buf, 35, is_vtol);
    _mav_put_uint8_t(buf, 36, vtol_fw_permanent_stab);
    _mav_put_uint8_t(buf, 37, in_transition_mode);
    _mav_put_uint8_t(buf, 38, in_transition_to_fw);
    _mav_put_uint8_t(buf, 39, rc_signal_lost);
    _mav_put_uint8_t(buf, 40, rc_input_mode);
    _mav_put_uint8_t(buf, 41, data_link_lost);
    _mav_put_uint8_t(buf, 42, data_link_lost_counter);
    _mav_put_uint8_t(buf, 43, high_latency_data_link_lost);
    _mav_put_uint8_t(buf, 44, engine_failure);
    _mav_put_uint8_t(buf, 45, mission_failure);
    _mav_put_uint8_t(buf, 46, failure_detector_status);
    _mav_put_uint8_t(buf, 47, aspd_check_failing);
    _mav_put_uint8_t(buf, 48, aspd_fault_declared);
    _mav_put_uint8_t(buf, 49, aspd_use_inhibit);
    _mav_put_uint8_t(buf, 50, aspd_fail_rtl);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_MVL_VEHICLE_STATUS_LEN);
#else
    mavlink_mvl_vehicle_status_t packet;
    packet.timestamp = timestamp;
    packet.onboard_control_sensors_present = onboard_control_sensors_present;
    packet.onboard_control_sensors_enabled = onboard_control_sensors_enabled;
    packet.onboard_control_sensors_health = onboard_control_sensors_health;
    packet.arspd_check_level = arspd_check_level;
    packet.load_factor_ratio = load_factor_ratio;
    packet.nav_state = nav_state;
    packet.arming_state = arming_state;
    packet.hil_state = hil_state;
    packet.red_state = red_state;
    packet.failsafe = failsafe;
    packet.system_type = system_type;
    packet.vehicle_type = vehicle_type;
    packet.is_vtol = is_vtol;
    packet.vtol_fw_permanent_stab = vtol_fw_permanent_stab;
    packet.in_transition_mode = in_transition_mode;
    packet.in_transition_to_fw = in_transition_to_fw;
    packet.rc_signal_lost = rc_signal_lost;
    packet.rc_input_mode = rc_input_mode;
    packet.data_link_lost = data_link_lost;
    packet.data_link_lost_counter = data_link_lost_counter;
    packet.high_latency_data_link_lost = high_latency_data_link_lost;
    packet.engine_failure = engine_failure;
    packet.mission_failure = mission_failure;
    packet.failure_detector_status = failure_detector_status;
    packet.aspd_check_failing = aspd_check_failing;
    packet.aspd_fault_declared = aspd_fault_declared;
    packet.aspd_use_inhibit = aspd_use_inhibit;
    packet.aspd_fail_rtl = aspd_fail_rtl;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_MVL_VEHICLE_STATUS_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_MVL_VEHICLE_STATUS;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_MVL_VEHICLE_STATUS_MIN_LEN, MAVLINK_MSG_ID_MVL_VEHICLE_STATUS_LEN, MAVLINK_MSG_ID_MVL_VEHICLE_STATUS_CRC);
}

/**
 * @brief Encode a mvl_vehicle_status struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param mvl_vehicle_status C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_mvl_vehicle_status_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_mvl_vehicle_status_t* mvl_vehicle_status)
{
    return mavlink_msg_mvl_vehicle_status_pack(system_id, component_id, msg, mvl_vehicle_status->timestamp, mvl_vehicle_status->onboard_control_sensors_present, mvl_vehicle_status->onboard_control_sensors_enabled, mvl_vehicle_status->onboard_control_sensors_health, mvl_vehicle_status->arspd_check_level, mvl_vehicle_status->load_factor_ratio, mvl_vehicle_status->nav_state, mvl_vehicle_status->arming_state, mvl_vehicle_status->hil_state, mvl_vehicle_status->red_state, mvl_vehicle_status->failsafe, mvl_vehicle_status->system_type, mvl_vehicle_status->vehicle_type, mvl_vehicle_status->is_vtol, mvl_vehicle_status->vtol_fw_permanent_stab, mvl_vehicle_status->in_transition_mode, mvl_vehicle_status->in_transition_to_fw, mvl_vehicle_status->rc_signal_lost, mvl_vehicle_status->rc_input_mode, mvl_vehicle_status->data_link_lost, mvl_vehicle_status->data_link_lost_counter, mvl_vehicle_status->high_latency_data_link_lost, mvl_vehicle_status->engine_failure, mvl_vehicle_status->mission_failure, mvl_vehicle_status->failure_detector_status, mvl_vehicle_status->aspd_check_failing, mvl_vehicle_status->aspd_fault_declared, mvl_vehicle_status->aspd_use_inhibit, mvl_vehicle_status->aspd_fail_rtl);
}

/**
 * @brief Encode a mvl_vehicle_status struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param mvl_vehicle_status C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_mvl_vehicle_status_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_mvl_vehicle_status_t* mvl_vehicle_status)
{
    return mavlink_msg_mvl_vehicle_status_pack_chan(system_id, component_id, chan, msg, mvl_vehicle_status->timestamp, mvl_vehicle_status->onboard_control_sensors_present, mvl_vehicle_status->onboard_control_sensors_enabled, mvl_vehicle_status->onboard_control_sensors_health, mvl_vehicle_status->arspd_check_level, mvl_vehicle_status->load_factor_ratio, mvl_vehicle_status->nav_state, mvl_vehicle_status->arming_state, mvl_vehicle_status->hil_state, mvl_vehicle_status->red_state, mvl_vehicle_status->failsafe, mvl_vehicle_status->system_type, mvl_vehicle_status->vehicle_type, mvl_vehicle_status->is_vtol, mvl_vehicle_status->vtol_fw_permanent_stab, mvl_vehicle_status->in_transition_mode, mvl_vehicle_status->in_transition_to_fw, mvl_vehicle_status->rc_signal_lost, mvl_vehicle_status->rc_input_mode, mvl_vehicle_status->data_link_lost, mvl_vehicle_status->data_link_lost_counter, mvl_vehicle_status->high_latency_data_link_lost, mvl_vehicle_status->engine_failure, mvl_vehicle_status->mission_failure, mvl_vehicle_status->failure_detector_status, mvl_vehicle_status->aspd_check_failing, mvl_vehicle_status->aspd_fault_declared, mvl_vehicle_status->aspd_use_inhibit, mvl_vehicle_status->aspd_fail_rtl);
}

/**
 * @brief Send a mvl_vehicle_status message
 * @param chan MAVLink channel to send the message
 *
 * @param timestamp  Timestamp iste
 * @param onboard_control_sensors_present  present sensors
 * @param onboard_control_sensors_enabled  enabled sensors
 * @param onboard_control_sensors_health  sensor health
 * @param arspd_check_level  airspeed check level
 * @param load_factor_ratio  load factor ratio
 * @param nav_state  Navigator state
 * @param arming_state  Arming state
 * @param hil_state  hil state
 * @param red_state  redundancy state
 * @param failsafe  failsafe
 * @param system_type  system_type
 * @param vehicle_type  vehicle_type
 * @param is_vtol  is_vtol
 * @param vtol_fw_permanent_stab  vtol_fw_permanent_stab
 * @param in_transition_mode  in_transition_mode
 * @param in_transition_to_fw  in_transition_to_fw
 * @param rc_signal_lost  rc_signal_lost
 * @param rc_input_mode  rc_input_mode
 * @param data_link_lost  data_link_lost
 * @param data_link_lost_counter  data_link_lost_counter
 * @param high_latency_data_link_lost  high_latency_data_link_lost
 * @param engine_failure  engine_failure
 * @param mission_failure  mission_failure
 * @param failure_detector_status  failure_detector_status
 * @param aspd_check_failing  aspd_check_failing
 * @param aspd_fault_declared  aspd_fault_declared
 * @param aspd_use_inhibit  aspd_use_inhibit
 * @param aspd_fail_rtl  aspd_fail_rtl
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_mvl_vehicle_status_send(mavlink_channel_t chan, uint64_t timestamp, uint32_t onboard_control_sensors_present, uint32_t onboard_control_sensors_enabled, uint32_t onboard_control_sensors_health, float arspd_check_level, float load_factor_ratio, uint8_t nav_state, uint8_t arming_state, uint8_t hil_state, uint8_t red_state, uint8_t failsafe, uint8_t system_type, uint8_t vehicle_type, uint8_t is_vtol, uint8_t vtol_fw_permanent_stab, uint8_t in_transition_mode, uint8_t in_transition_to_fw, uint8_t rc_signal_lost, uint8_t rc_input_mode, uint8_t data_link_lost, uint8_t data_link_lost_counter, uint8_t high_latency_data_link_lost, uint8_t engine_failure, uint8_t mission_failure, uint8_t failure_detector_status, uint8_t aspd_check_failing, uint8_t aspd_fault_declared, uint8_t aspd_use_inhibit, uint8_t aspd_fail_rtl)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_MVL_VEHICLE_STATUS_LEN];
    _mav_put_uint64_t(buf, 0, timestamp);
    _mav_put_uint32_t(buf, 8, onboard_control_sensors_present);
    _mav_put_uint32_t(buf, 12, onboard_control_sensors_enabled);
    _mav_put_uint32_t(buf, 16, onboard_control_sensors_health);
    _mav_put_float(buf, 20, arspd_check_level);
    _mav_put_float(buf, 24, load_factor_ratio);
    _mav_put_uint8_t(buf, 28, nav_state);
    _mav_put_uint8_t(buf, 29, arming_state);
    _mav_put_uint8_t(buf, 30, hil_state);
    _mav_put_uint8_t(buf, 31, red_state);
    _mav_put_uint8_t(buf, 32, failsafe);
    _mav_put_uint8_t(buf, 33, system_type);
    _mav_put_uint8_t(buf, 34, vehicle_type);
    _mav_put_uint8_t(buf, 35, is_vtol);
    _mav_put_uint8_t(buf, 36, vtol_fw_permanent_stab);
    _mav_put_uint8_t(buf, 37, in_transition_mode);
    _mav_put_uint8_t(buf, 38, in_transition_to_fw);
    _mav_put_uint8_t(buf, 39, rc_signal_lost);
    _mav_put_uint8_t(buf, 40, rc_input_mode);
    _mav_put_uint8_t(buf, 41, data_link_lost);
    _mav_put_uint8_t(buf, 42, data_link_lost_counter);
    _mav_put_uint8_t(buf, 43, high_latency_data_link_lost);
    _mav_put_uint8_t(buf, 44, engine_failure);
    _mav_put_uint8_t(buf, 45, mission_failure);
    _mav_put_uint8_t(buf, 46, failure_detector_status);
    _mav_put_uint8_t(buf, 47, aspd_check_failing);
    _mav_put_uint8_t(buf, 48, aspd_fault_declared);
    _mav_put_uint8_t(buf, 49, aspd_use_inhibit);
    _mav_put_uint8_t(buf, 50, aspd_fail_rtl);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MVL_VEHICLE_STATUS, buf, MAVLINK_MSG_ID_MVL_VEHICLE_STATUS_MIN_LEN, MAVLINK_MSG_ID_MVL_VEHICLE_STATUS_LEN, MAVLINK_MSG_ID_MVL_VEHICLE_STATUS_CRC);
#else
    mavlink_mvl_vehicle_status_t packet;
    packet.timestamp = timestamp;
    packet.onboard_control_sensors_present = onboard_control_sensors_present;
    packet.onboard_control_sensors_enabled = onboard_control_sensors_enabled;
    packet.onboard_control_sensors_health = onboard_control_sensors_health;
    packet.arspd_check_level = arspd_check_level;
    packet.load_factor_ratio = load_factor_ratio;
    packet.nav_state = nav_state;
    packet.arming_state = arming_state;
    packet.hil_state = hil_state;
    packet.red_state = red_state;
    packet.failsafe = failsafe;
    packet.system_type = system_type;
    packet.vehicle_type = vehicle_type;
    packet.is_vtol = is_vtol;
    packet.vtol_fw_permanent_stab = vtol_fw_permanent_stab;
    packet.in_transition_mode = in_transition_mode;
    packet.in_transition_to_fw = in_transition_to_fw;
    packet.rc_signal_lost = rc_signal_lost;
    packet.rc_input_mode = rc_input_mode;
    packet.data_link_lost = data_link_lost;
    packet.data_link_lost_counter = data_link_lost_counter;
    packet.high_latency_data_link_lost = high_latency_data_link_lost;
    packet.engine_failure = engine_failure;
    packet.mission_failure = mission_failure;
    packet.failure_detector_status = failure_detector_status;
    packet.aspd_check_failing = aspd_check_failing;
    packet.aspd_fault_declared = aspd_fault_declared;
    packet.aspd_use_inhibit = aspd_use_inhibit;
    packet.aspd_fail_rtl = aspd_fail_rtl;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MVL_VEHICLE_STATUS, (const char *)&packet, MAVLINK_MSG_ID_MVL_VEHICLE_STATUS_MIN_LEN, MAVLINK_MSG_ID_MVL_VEHICLE_STATUS_LEN, MAVLINK_MSG_ID_MVL_VEHICLE_STATUS_CRC);
#endif
}

/**
 * @brief Send a mvl_vehicle_status message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_mvl_vehicle_status_send_struct(mavlink_channel_t chan, const mavlink_mvl_vehicle_status_t* mvl_vehicle_status)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_mvl_vehicle_status_send(chan, mvl_vehicle_status->timestamp, mvl_vehicle_status->onboard_control_sensors_present, mvl_vehicle_status->onboard_control_sensors_enabled, mvl_vehicle_status->onboard_control_sensors_health, mvl_vehicle_status->arspd_check_level, mvl_vehicle_status->load_factor_ratio, mvl_vehicle_status->nav_state, mvl_vehicle_status->arming_state, mvl_vehicle_status->hil_state, mvl_vehicle_status->red_state, mvl_vehicle_status->failsafe, mvl_vehicle_status->system_type, mvl_vehicle_status->vehicle_type, mvl_vehicle_status->is_vtol, mvl_vehicle_status->vtol_fw_permanent_stab, mvl_vehicle_status->in_transition_mode, mvl_vehicle_status->in_transition_to_fw, mvl_vehicle_status->rc_signal_lost, mvl_vehicle_status->rc_input_mode, mvl_vehicle_status->data_link_lost, mvl_vehicle_status->data_link_lost_counter, mvl_vehicle_status->high_latency_data_link_lost, mvl_vehicle_status->engine_failure, mvl_vehicle_status->mission_failure, mvl_vehicle_status->failure_detector_status, mvl_vehicle_status->aspd_check_failing, mvl_vehicle_status->aspd_fault_declared, mvl_vehicle_status->aspd_use_inhibit, mvl_vehicle_status->aspd_fail_rtl);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MVL_VEHICLE_STATUS, (const char *)mvl_vehicle_status, MAVLINK_MSG_ID_MVL_VEHICLE_STATUS_MIN_LEN, MAVLINK_MSG_ID_MVL_VEHICLE_STATUS_LEN, MAVLINK_MSG_ID_MVL_VEHICLE_STATUS_CRC);
#endif
}

#if MAVLINK_MSG_ID_MVL_VEHICLE_STATUS_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_mvl_vehicle_status_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint64_t timestamp, uint32_t onboard_control_sensors_present, uint32_t onboard_control_sensors_enabled, uint32_t onboard_control_sensors_health, float arspd_check_level, float load_factor_ratio, uint8_t nav_state, uint8_t arming_state, uint8_t hil_state, uint8_t red_state, uint8_t failsafe, uint8_t system_type, uint8_t vehicle_type, uint8_t is_vtol, uint8_t vtol_fw_permanent_stab, uint8_t in_transition_mode, uint8_t in_transition_to_fw, uint8_t rc_signal_lost, uint8_t rc_input_mode, uint8_t data_link_lost, uint8_t data_link_lost_counter, uint8_t high_latency_data_link_lost, uint8_t engine_failure, uint8_t mission_failure, uint8_t failure_detector_status, uint8_t aspd_check_failing, uint8_t aspd_fault_declared, uint8_t aspd_use_inhibit, uint8_t aspd_fail_rtl)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint64_t(buf, 0, timestamp);
    _mav_put_uint32_t(buf, 8, onboard_control_sensors_present);
    _mav_put_uint32_t(buf, 12, onboard_control_sensors_enabled);
    _mav_put_uint32_t(buf, 16, onboard_control_sensors_health);
    _mav_put_float(buf, 20, arspd_check_level);
    _mav_put_float(buf, 24, load_factor_ratio);
    _mav_put_uint8_t(buf, 28, nav_state);
    _mav_put_uint8_t(buf, 29, arming_state);
    _mav_put_uint8_t(buf, 30, hil_state);
    _mav_put_uint8_t(buf, 31, red_state);
    _mav_put_uint8_t(buf, 32, failsafe);
    _mav_put_uint8_t(buf, 33, system_type);
    _mav_put_uint8_t(buf, 34, vehicle_type);
    _mav_put_uint8_t(buf, 35, is_vtol);
    _mav_put_uint8_t(buf, 36, vtol_fw_permanent_stab);
    _mav_put_uint8_t(buf, 37, in_transition_mode);
    _mav_put_uint8_t(buf, 38, in_transition_to_fw);
    _mav_put_uint8_t(buf, 39, rc_signal_lost);
    _mav_put_uint8_t(buf, 40, rc_input_mode);
    _mav_put_uint8_t(buf, 41, data_link_lost);
    _mav_put_uint8_t(buf, 42, data_link_lost_counter);
    _mav_put_uint8_t(buf, 43, high_latency_data_link_lost);
    _mav_put_uint8_t(buf, 44, engine_failure);
    _mav_put_uint8_t(buf, 45, mission_failure);
    _mav_put_uint8_t(buf, 46, failure_detector_status);
    _mav_put_uint8_t(buf, 47, aspd_check_failing);
    _mav_put_uint8_t(buf, 48, aspd_fault_declared);
    _mav_put_uint8_t(buf, 49, aspd_use_inhibit);
    _mav_put_uint8_t(buf, 50, aspd_fail_rtl);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MVL_VEHICLE_STATUS, buf, MAVLINK_MSG_ID_MVL_VEHICLE_STATUS_MIN_LEN, MAVLINK_MSG_ID_MVL_VEHICLE_STATUS_LEN, MAVLINK_MSG_ID_MVL_VEHICLE_STATUS_CRC);
#else
    mavlink_mvl_vehicle_status_t *packet = (mavlink_mvl_vehicle_status_t *)msgbuf;
    packet->timestamp = timestamp;
    packet->onboard_control_sensors_present = onboard_control_sensors_present;
    packet->onboard_control_sensors_enabled = onboard_control_sensors_enabled;
    packet->onboard_control_sensors_health = onboard_control_sensors_health;
    packet->arspd_check_level = arspd_check_level;
    packet->load_factor_ratio = load_factor_ratio;
    packet->nav_state = nav_state;
    packet->arming_state = arming_state;
    packet->hil_state = hil_state;
    packet->red_state = red_state;
    packet->failsafe = failsafe;
    packet->system_type = system_type;
    packet->vehicle_type = vehicle_type;
    packet->is_vtol = is_vtol;
    packet->vtol_fw_permanent_stab = vtol_fw_permanent_stab;
    packet->in_transition_mode = in_transition_mode;
    packet->in_transition_to_fw = in_transition_to_fw;
    packet->rc_signal_lost = rc_signal_lost;
    packet->rc_input_mode = rc_input_mode;
    packet->data_link_lost = data_link_lost;
    packet->data_link_lost_counter = data_link_lost_counter;
    packet->high_latency_data_link_lost = high_latency_data_link_lost;
    packet->engine_failure = engine_failure;
    packet->mission_failure = mission_failure;
    packet->failure_detector_status = failure_detector_status;
    packet->aspd_check_failing = aspd_check_failing;
    packet->aspd_fault_declared = aspd_fault_declared;
    packet->aspd_use_inhibit = aspd_use_inhibit;
    packet->aspd_fail_rtl = aspd_fail_rtl;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MVL_VEHICLE_STATUS, (const char *)packet, MAVLINK_MSG_ID_MVL_VEHICLE_STATUS_MIN_LEN, MAVLINK_MSG_ID_MVL_VEHICLE_STATUS_LEN, MAVLINK_MSG_ID_MVL_VEHICLE_STATUS_CRC);
#endif
}
#endif

#endif

// MESSAGE MVL_VEHICLE_STATUS UNPACKING


/**
 * @brief Get field timestamp from mvl_vehicle_status message
 *
 * @return  Timestamp iste
 */
static inline uint64_t mavlink_msg_mvl_vehicle_status_get_timestamp(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint64_t(msg,  0);
}

/**
 * @brief Get field onboard_control_sensors_present from mvl_vehicle_status message
 *
 * @return  present sensors
 */
static inline uint32_t mavlink_msg_mvl_vehicle_status_get_onboard_control_sensors_present(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint32_t(msg,  8);
}

/**
 * @brief Get field onboard_control_sensors_enabled from mvl_vehicle_status message
 *
 * @return  enabled sensors
 */
static inline uint32_t mavlink_msg_mvl_vehicle_status_get_onboard_control_sensors_enabled(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint32_t(msg,  12);
}

/**
 * @brief Get field onboard_control_sensors_health from mvl_vehicle_status message
 *
 * @return  sensor health
 */
static inline uint32_t mavlink_msg_mvl_vehicle_status_get_onboard_control_sensors_health(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint32_t(msg,  16);
}

/**
 * @brief Get field arspd_check_level from mvl_vehicle_status message
 *
 * @return  airspeed check level
 */
static inline float mavlink_msg_mvl_vehicle_status_get_arspd_check_level(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  20);
}

/**
 * @brief Get field load_factor_ratio from mvl_vehicle_status message
 *
 * @return  load factor ratio
 */
static inline float mavlink_msg_mvl_vehicle_status_get_load_factor_ratio(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  24);
}

/**
 * @brief Get field nav_state from mvl_vehicle_status message
 *
 * @return  Navigator state
 */
static inline uint8_t mavlink_msg_mvl_vehicle_status_get_nav_state(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  28);
}

/**
 * @brief Get field arming_state from mvl_vehicle_status message
 *
 * @return  Arming state
 */
static inline uint8_t mavlink_msg_mvl_vehicle_status_get_arming_state(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  29);
}

/**
 * @brief Get field hil_state from mvl_vehicle_status message
 *
 * @return  hil state
 */
static inline uint8_t mavlink_msg_mvl_vehicle_status_get_hil_state(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  30);
}

/**
 * @brief Get field red_state from mvl_vehicle_status message
 *
 * @return  redundancy state
 */
static inline uint8_t mavlink_msg_mvl_vehicle_status_get_red_state(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  31);
}

/**
 * @brief Get field failsafe from mvl_vehicle_status message
 *
 * @return  failsafe
 */
static inline uint8_t mavlink_msg_mvl_vehicle_status_get_failsafe(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  32);
}

/**
 * @brief Get field system_type from mvl_vehicle_status message
 *
 * @return  system_type
 */
static inline uint8_t mavlink_msg_mvl_vehicle_status_get_system_type(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  33);
}

/**
 * @brief Get field vehicle_type from mvl_vehicle_status message
 *
 * @return  vehicle_type
 */
static inline uint8_t mavlink_msg_mvl_vehicle_status_get_vehicle_type(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  34);
}

/**
 * @brief Get field is_vtol from mvl_vehicle_status message
 *
 * @return  is_vtol
 */
static inline uint8_t mavlink_msg_mvl_vehicle_status_get_is_vtol(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  35);
}

/**
 * @brief Get field vtol_fw_permanent_stab from mvl_vehicle_status message
 *
 * @return  vtol_fw_permanent_stab
 */
static inline uint8_t mavlink_msg_mvl_vehicle_status_get_vtol_fw_permanent_stab(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  36);
}

/**
 * @brief Get field in_transition_mode from mvl_vehicle_status message
 *
 * @return  in_transition_mode
 */
static inline uint8_t mavlink_msg_mvl_vehicle_status_get_in_transition_mode(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  37);
}

/**
 * @brief Get field in_transition_to_fw from mvl_vehicle_status message
 *
 * @return  in_transition_to_fw
 */
static inline uint8_t mavlink_msg_mvl_vehicle_status_get_in_transition_to_fw(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  38);
}

/**
 * @brief Get field rc_signal_lost from mvl_vehicle_status message
 *
 * @return  rc_signal_lost
 */
static inline uint8_t mavlink_msg_mvl_vehicle_status_get_rc_signal_lost(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  39);
}

/**
 * @brief Get field rc_input_mode from mvl_vehicle_status message
 *
 * @return  rc_input_mode
 */
static inline uint8_t mavlink_msg_mvl_vehicle_status_get_rc_input_mode(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  40);
}

/**
 * @brief Get field data_link_lost from mvl_vehicle_status message
 *
 * @return  data_link_lost
 */
static inline uint8_t mavlink_msg_mvl_vehicle_status_get_data_link_lost(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  41);
}

/**
 * @brief Get field data_link_lost_counter from mvl_vehicle_status message
 *
 * @return  data_link_lost_counter
 */
static inline uint8_t mavlink_msg_mvl_vehicle_status_get_data_link_lost_counter(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  42);
}

/**
 * @brief Get field high_latency_data_link_lost from mvl_vehicle_status message
 *
 * @return  high_latency_data_link_lost
 */
static inline uint8_t mavlink_msg_mvl_vehicle_status_get_high_latency_data_link_lost(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  43);
}

/**
 * @brief Get field engine_failure from mvl_vehicle_status message
 *
 * @return  engine_failure
 */
static inline uint8_t mavlink_msg_mvl_vehicle_status_get_engine_failure(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  44);
}

/**
 * @brief Get field mission_failure from mvl_vehicle_status message
 *
 * @return  mission_failure
 */
static inline uint8_t mavlink_msg_mvl_vehicle_status_get_mission_failure(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  45);
}

/**
 * @brief Get field failure_detector_status from mvl_vehicle_status message
 *
 * @return  failure_detector_status
 */
static inline uint8_t mavlink_msg_mvl_vehicle_status_get_failure_detector_status(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  46);
}

/**
 * @brief Get field aspd_check_failing from mvl_vehicle_status message
 *
 * @return  aspd_check_failing
 */
static inline uint8_t mavlink_msg_mvl_vehicle_status_get_aspd_check_failing(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  47);
}

/**
 * @brief Get field aspd_fault_declared from mvl_vehicle_status message
 *
 * @return  aspd_fault_declared
 */
static inline uint8_t mavlink_msg_mvl_vehicle_status_get_aspd_fault_declared(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  48);
}

/**
 * @brief Get field aspd_use_inhibit from mvl_vehicle_status message
 *
 * @return  aspd_use_inhibit
 */
static inline uint8_t mavlink_msg_mvl_vehicle_status_get_aspd_use_inhibit(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  49);
}

/**
 * @brief Get field aspd_fail_rtl from mvl_vehicle_status message
 *
 * @return  aspd_fail_rtl
 */
static inline uint8_t mavlink_msg_mvl_vehicle_status_get_aspd_fail_rtl(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  50);
}

/**
 * @brief Decode a mvl_vehicle_status message into a struct
 *
 * @param msg The message to decode
 * @param mvl_vehicle_status C-struct to decode the message contents into
 */
static inline void mavlink_msg_mvl_vehicle_status_decode(const mavlink_message_t* msg, mavlink_mvl_vehicle_status_t* mvl_vehicle_status)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mvl_vehicle_status->timestamp = mavlink_msg_mvl_vehicle_status_get_timestamp(msg);
    mvl_vehicle_status->onboard_control_sensors_present = mavlink_msg_mvl_vehicle_status_get_onboard_control_sensors_present(msg);
    mvl_vehicle_status->onboard_control_sensors_enabled = mavlink_msg_mvl_vehicle_status_get_onboard_control_sensors_enabled(msg);
    mvl_vehicle_status->onboard_control_sensors_health = mavlink_msg_mvl_vehicle_status_get_onboard_control_sensors_health(msg);
    mvl_vehicle_status->arspd_check_level = mavlink_msg_mvl_vehicle_status_get_arspd_check_level(msg);
    mvl_vehicle_status->load_factor_ratio = mavlink_msg_mvl_vehicle_status_get_load_factor_ratio(msg);
    mvl_vehicle_status->nav_state = mavlink_msg_mvl_vehicle_status_get_nav_state(msg);
    mvl_vehicle_status->arming_state = mavlink_msg_mvl_vehicle_status_get_arming_state(msg);
    mvl_vehicle_status->hil_state = mavlink_msg_mvl_vehicle_status_get_hil_state(msg);
    mvl_vehicle_status->red_state = mavlink_msg_mvl_vehicle_status_get_red_state(msg);
    mvl_vehicle_status->failsafe = mavlink_msg_mvl_vehicle_status_get_failsafe(msg);
    mvl_vehicle_status->system_type = mavlink_msg_mvl_vehicle_status_get_system_type(msg);
    mvl_vehicle_status->vehicle_type = mavlink_msg_mvl_vehicle_status_get_vehicle_type(msg);
    mvl_vehicle_status->is_vtol = mavlink_msg_mvl_vehicle_status_get_is_vtol(msg);
    mvl_vehicle_status->vtol_fw_permanent_stab = mavlink_msg_mvl_vehicle_status_get_vtol_fw_permanent_stab(msg);
    mvl_vehicle_status->in_transition_mode = mavlink_msg_mvl_vehicle_status_get_in_transition_mode(msg);
    mvl_vehicle_status->in_transition_to_fw = mavlink_msg_mvl_vehicle_status_get_in_transition_to_fw(msg);
    mvl_vehicle_status->rc_signal_lost = mavlink_msg_mvl_vehicle_status_get_rc_signal_lost(msg);
    mvl_vehicle_status->rc_input_mode = mavlink_msg_mvl_vehicle_status_get_rc_input_mode(msg);
    mvl_vehicle_status->data_link_lost = mavlink_msg_mvl_vehicle_status_get_data_link_lost(msg);
    mvl_vehicle_status->data_link_lost_counter = mavlink_msg_mvl_vehicle_status_get_data_link_lost_counter(msg);
    mvl_vehicle_status->high_latency_data_link_lost = mavlink_msg_mvl_vehicle_status_get_high_latency_data_link_lost(msg);
    mvl_vehicle_status->engine_failure = mavlink_msg_mvl_vehicle_status_get_engine_failure(msg);
    mvl_vehicle_status->mission_failure = mavlink_msg_mvl_vehicle_status_get_mission_failure(msg);
    mvl_vehicle_status->failure_detector_status = mavlink_msg_mvl_vehicle_status_get_failure_detector_status(msg);
    mvl_vehicle_status->aspd_check_failing = mavlink_msg_mvl_vehicle_status_get_aspd_check_failing(msg);
    mvl_vehicle_status->aspd_fault_declared = mavlink_msg_mvl_vehicle_status_get_aspd_fault_declared(msg);
    mvl_vehicle_status->aspd_use_inhibit = mavlink_msg_mvl_vehicle_status_get_aspd_use_inhibit(msg);
    mvl_vehicle_status->aspd_fail_rtl = mavlink_msg_mvl_vehicle_status_get_aspd_fail_rtl(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_MVL_VEHICLE_STATUS_LEN? msg->len : MAVLINK_MSG_ID_MVL_VEHICLE_STATUS_LEN;
        memset(mvl_vehicle_status, 0, MAVLINK_MSG_ID_MVL_VEHICLE_STATUS_LEN);
    memcpy(mvl_vehicle_status, _MAV_PAYLOAD(msg), len);
#endif
}
