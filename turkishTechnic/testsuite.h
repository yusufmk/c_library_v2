/** @file
 *    @brief MAVLink comm protocol testsuite generated from turkishTechnic.xml
 *    @see http://qgroundcontrol.org/mavlink/
 */
#pragma once
#ifndef TURKISHTECHNIC_TESTSUITE_H
#define TURKISHTECHNIC_TESTSUITE_H

#ifdef __cplusplus
extern "C" {
#endif

#ifndef MAVLINK_TEST_ALL
#define MAVLINK_TEST_ALL
static void mavlink_test_common(uint8_t, uint8_t, mavlink_message_t *last_msg);
static void mavlink_test_turkishTechnic(uint8_t, uint8_t, mavlink_message_t *last_msg);

static void mavlink_test_all(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
    mavlink_test_common(system_id, component_id, last_msg);
    mavlink_test_turkishTechnic(system_id, component_id, last_msg);
}
#endif

#include "../common/testsuite.h"


static void mavlink_test_mvl_vehicle_status(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_MVL_VEHICLE_STATUS >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_mvl_vehicle_status_t packet_in = {
        93372036854775807ULL,963497880,963498088,963498296,65,132,199,10,77,144,211,22,89,156,223,34,101,168,235,46,113,180
    };
    mavlink_mvl_vehicle_status_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.timestamp = packet_in.timestamp;
        packet1.onboard_control_sensors_present = packet_in.onboard_control_sensors_present;
        packet1.onboard_control_sensors_enabled = packet_in.onboard_control_sensors_enabled;
        packet1.onboard_control_sensors_health = packet_in.onboard_control_sensors_health;
        packet1.nav_state = packet_in.nav_state;
        packet1.arming_state = packet_in.arming_state;
        packet1.hil_state = packet_in.hil_state;
        packet1.red_state = packet_in.red_state;
        packet1.failsafe = packet_in.failsafe;
        packet1.system_type = packet_in.system_type;
        packet1.vehicle_type = packet_in.vehicle_type;
        packet1.is_vtol = packet_in.is_vtol;
        packet1.vtol_fw_permanent_stab = packet_in.vtol_fw_permanent_stab;
        packet1.in_transition_mode = packet_in.in_transition_mode;
        packet1.in_transition_to_fw = packet_in.in_transition_to_fw;
        packet1.rc_signal_lost = packet_in.rc_signal_lost;
        packet1.rc_input_mode = packet_in.rc_input_mode;
        packet1.data_link_lost = packet_in.data_link_lost;
        packet1.data_link_lost_counter = packet_in.data_link_lost_counter;
        packet1.high_latency_data_link_lost = packet_in.high_latency_data_link_lost;
        packet1.engine_failure = packet_in.engine_failure;
        packet1.mission_failure = packet_in.mission_failure;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_MVL_VEHICLE_STATUS_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_MVL_VEHICLE_STATUS_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_mvl_vehicle_status_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_mvl_vehicle_status_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_mvl_vehicle_status_pack(system_id, component_id, &msg , packet1.timestamp , packet1.onboard_control_sensors_present , packet1.onboard_control_sensors_enabled , packet1.onboard_control_sensors_health , packet1.nav_state , packet1.arming_state , packet1.hil_state , packet1.red_state , packet1.failsafe , packet1.system_type , packet1.vehicle_type , packet1.is_vtol , packet1.vtol_fw_permanent_stab , packet1.in_transition_mode , packet1.in_transition_to_fw , packet1.rc_signal_lost , packet1.rc_input_mode , packet1.data_link_lost , packet1.data_link_lost_counter , packet1.high_latency_data_link_lost , packet1.engine_failure , packet1.mission_failure );
    mavlink_msg_mvl_vehicle_status_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_mvl_vehicle_status_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.timestamp , packet1.onboard_control_sensors_present , packet1.onboard_control_sensors_enabled , packet1.onboard_control_sensors_health , packet1.nav_state , packet1.arming_state , packet1.hil_state , packet1.red_state , packet1.failsafe , packet1.system_type , packet1.vehicle_type , packet1.is_vtol , packet1.vtol_fw_permanent_stab , packet1.in_transition_mode , packet1.in_transition_to_fw , packet1.rc_signal_lost , packet1.rc_input_mode , packet1.data_link_lost , packet1.data_link_lost_counter , packet1.high_latency_data_link_lost , packet1.engine_failure , packet1.mission_failure );
    mavlink_msg_mvl_vehicle_status_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_mvl_vehicle_status_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_mvl_vehicle_status_send(MAVLINK_COMM_1 , packet1.timestamp , packet1.onboard_control_sensors_present , packet1.onboard_control_sensors_enabled , packet1.onboard_control_sensors_health , packet1.nav_state , packet1.arming_state , packet1.hil_state , packet1.red_state , packet1.failsafe , packet1.system_type , packet1.vehicle_type , packet1.is_vtol , packet1.vtol_fw_permanent_stab , packet1.in_transition_mode , packet1.in_transition_to_fw , packet1.rc_signal_lost , packet1.rc_input_mode , packet1.data_link_lost , packet1.data_link_lost_counter , packet1.high_latency_data_link_lost , packet1.engine_failure , packet1.mission_failure );
    mavlink_msg_mvl_vehicle_status_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_attitude2(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_ATTITUDE2 >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_attitude2_t packet_in = {
        963497464,45.0,73.0,101.0,129.0,157.0,185.0,89
    };
    mavlink_attitude2_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.time_boot_ms = packet_in.time_boot_ms;
        packet1.roll = packet_in.roll;
        packet1.pitch = packet_in.pitch;
        packet1.yaw = packet_in.yaw;
        packet1.rollspeed = packet_in.rollspeed;
        packet1.pitchspeed = packet_in.pitchspeed;
        packet1.yawspeed = packet_in.yawspeed;
        packet1.attitude_id = packet_in.attitude_id;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_ATTITUDE2_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_ATTITUDE2_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_attitude2_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_attitude2_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_attitude2_pack(system_id, component_id, &msg , packet1.time_boot_ms , packet1.attitude_id , packet1.roll , packet1.pitch , packet1.yaw , packet1.rollspeed , packet1.pitchspeed , packet1.yawspeed );
    mavlink_msg_attitude2_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_attitude2_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.time_boot_ms , packet1.attitude_id , packet1.roll , packet1.pitch , packet1.yaw , packet1.rollspeed , packet1.pitchspeed , packet1.yawspeed );
    mavlink_msg_attitude2_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_attitude2_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_attitude2_send(MAVLINK_COMM_1 , packet1.time_boot_ms , packet1.attitude_id , packet1.roll , packet1.pitch , packet1.yaw , packet1.rollspeed , packet1.pitchspeed , packet1.yawspeed );
    mavlink_msg_attitude2_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_global_position_int2(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_GLOBAL_POSITION_INT2 >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_global_position_int2_t packet_in = {
        963497464,963497672,963497880,963498088,963498296,18275,18379,18483,18587,89
    };
    mavlink_global_position_int2_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.time_boot_ms = packet_in.time_boot_ms;
        packet1.lat = packet_in.lat;
        packet1.lon = packet_in.lon;
        packet1.alt = packet_in.alt;
        packet1.relative_alt = packet_in.relative_alt;
        packet1.vx = packet_in.vx;
        packet1.vy = packet_in.vy;
        packet1.vz = packet_in.vz;
        packet1.hdg = packet_in.hdg;
        packet1.gpos_id = packet_in.gpos_id;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_GLOBAL_POSITION_INT2_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_GLOBAL_POSITION_INT2_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_global_position_int2_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_global_position_int2_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_global_position_int2_pack(system_id, component_id, &msg , packet1.time_boot_ms , packet1.gpos_id , packet1.lat , packet1.lon , packet1.alt , packet1.relative_alt , packet1.vx , packet1.vy , packet1.vz , packet1.hdg );
    mavlink_msg_global_position_int2_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_global_position_int2_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.time_boot_ms , packet1.gpos_id , packet1.lat , packet1.lon , packet1.alt , packet1.relative_alt , packet1.vx , packet1.vy , packet1.vz , packet1.hdg );
    mavlink_msg_global_position_int2_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_global_position_int2_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_global_position_int2_send(MAVLINK_COMM_1 , packet1.time_boot_ms , packet1.gpos_id , packet1.lat , packet1.lon , packet1.alt , packet1.relative_alt , packet1.vx , packet1.vy , packet1.vz , packet1.hdg );
    mavlink_msg_global_position_int2_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_turkishTechnic(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
    mavlink_test_mvl_vehicle_status(system_id, component_id, last_msg);
    mavlink_test_attitude2(system_id, component_id, last_msg);
    mavlink_test_global_position_int2(system_id, component_id, last_msg);
}

#ifdef __cplusplus
}
#endif // __cplusplus
#endif // TURKISHTECHNIC_TESTSUITE_H
