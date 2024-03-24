/*
 * Copyright (c) 2023 CogniPilot Foundation
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef SYNAPSE_TOPIC_LIST_H
#define SYNAPSE_TOPIC_LIST_H

#include <zros/zros_topic.h>

#include <synapse_protobuf/actuators.pb.h>
#include <synapse_protobuf/imu.pb.h>
#include <synapse_protobuf/joy.pb.h>
#include <synapse_protobuf/road_curve_angle.pb.h>
#include <synapse_protobuf/led_array.pb.h>
#include <synapse_protobuf/status.pb.h>
#include <synapse_protobuf/time.pb.h>
#include <synapse_protobuf/twist.pb.h>
#include <synapse_protobuf/twist_with_covariance.pb.h>
#include <synapse_protobuf/point.pb.h>
#include <synapse_protobuf/pose.pb.h>
#include <synapse_protobuf/pose_with_covariance.pb.h>
#include <synapse_protobuf/vector3.pb.h>
#include <synapse_protobuf/quaternion.pb.h>


/********************************************************************
 * helper
 ********************************************************************/
void stamp_header(synapse_msgs_Header* hdr, int64_t ticks);
const char* mode_str(synapse_msgs_Status_Mode mode);
const char* armed_str(synapse_msgs_Status_Arming arming);
const char* status_joy_str(synapse_msgs_Status_Joy joy);

enum {
    JOY_BUTTON_MANUAL = 0,
    JOY_BUTTON_AUTO = 1,
    JOY_BUTTON_CMD_VEL = 2,
    JOY_BUTTON_CALIBRATION = 3,
    JOY_BUTTON_LIGHTS_OFF = 4,
    JOY_BUTTON_LIGHTS_ON = 5,
    JOY_BUTTON_DISARM = 6,
    JOY_BUTTON_ARM = 7,
};

enum {
    JOY_AXES_THRUST = 1,
    JOY_AXES_PITCH = 2,
    JOY_AXES_ROLL = 3,
    JOY_AXES_YAW = 4,
};

/********************************************************************
 * topics
 ********************************************************************/

ZROS_TOPIC_DECLARE(topic_actuators_manual, synapse_msgs_Actuators); // Actuators from manual mode
ZROS_TOPIC_DECLARE(topic_actuators_auto, synapse_msgs_Actuators); // Actuators from auto mode
ZROS_TOPIC_DECLARE(topic_actuators, synapse_msgs_Actuators); // Actuators sent to motors (actuate/pwm node)

ZROS_TOPIC_DECLARE(topic_imu, synapse_msgs_Imu); // Actuators received from imu sensor
ZROS_TOPIC_DECLARE(topic_joy, synapse_msgs_Joy);
ZROS_TOPIC_DECLARE(topic_road_curve_angle, synapse_msgs_RoadCurveAngle);
ZROS_TOPIC_DECLARE(topic_led_array, synapse_msgs_LEDArray);
ZROS_TOPIC_DECLARE(topic_status, synapse_msgs_Status);

#endif // SYNAPSE_TOPIC_LIST_H_
// vi: ts=4 sw=4 et
