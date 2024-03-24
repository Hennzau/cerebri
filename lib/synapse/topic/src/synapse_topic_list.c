/*
 * Copyright (c) 2023 CogniPilot Foundation
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/init.h>
#include <zephyr/sys/slist.h>

#include <zros/private/zros_broker_struct.h>
#include <zros/private/zros_topic_struct.h>
#include <zros/zros_broker.h>

#include "synapse_topic_list.h"

//*******************************************************************
// helper functions
//*******************************************************************
static const char* unhandled = "UNHANDLED";

void stamp_header(synapse_msgs_Header* hdr, int64_t ticks)
{
    int64_t sec = ticks / CONFIG_SYS_CLOCK_TICKS_PER_SEC;
    int32_t nanosec = (ticks - sec * CONFIG_SYS_CLOCK_TICKS_PER_SEC) * 1e9 / CONFIG_SYS_CLOCK_TICKS_PER_SEC;
    hdr->has_stamp = true;
    hdr->stamp.sec = sec;
    hdr->stamp.nanosec = nanosec;
}

const char* mode_str(synapse_msgs_Status_Mode mode)
{
    if (mode == synapse_msgs_Status_Mode_MODE_UNKNOWN) {
        return "unknown";
    } else if (mode == synapse_msgs_Status_Mode_MODE_MANUAL) {
        return "manual";
    } else if (mode == synapse_msgs_Status_Mode_MODE_AUTO) {
        return "auto";
    } else if (mode == synapse_msgs_Status_Mode_MODE_CMD_VEL) {
        return "cmd_vel";
    }
    return unhandled;
}

const char* armed_str(synapse_msgs_Status_Arming arming)
{
    if (arming == synapse_msgs_Status_Arming_ARMING_UNKNOWN) {
        return "unknown";
    } else if (arming == synapse_msgs_Status_Arming_ARMING_ARMED) {
        return "armed";
    } else if (arming == synapse_msgs_Status_Arming_ARMING_DISARMED) {
        return "disarmed";
    }
    return unhandled;
}

const char* status_joy_str(synapse_msgs_Status_Joy joy)
{
    if (joy == synapse_msgs_Status_Joy_JOY_UNKNOWN) {
        return "unknown";
    } else if (joy == synapse_msgs_Status_Joy_JOY_NOMINAL) {
        return "nominal";
    } else if (joy == synapse_msgs_Status_Joy_JOY_LOSS) {
        return "loss";
    }
    return unhandled;
}

/********************************************************************
 * topics
 ********************************************************************/
ZROS_TOPIC_DEFINE(actuators_manual, synapse_msgs_Actuators);
ZROS_TOPIC_DEFINE(actuators_auto, synapse_msgs_Actuators);
ZROS_TOPIC_DEFINE(actuators, synapse_msgs_Actuators);

ZROS_TOPIC_DEFINE(status, synapse_msgs_Status);
ZROS_TOPIC_DEFINE(road_curve_angle, synapse_msgs_RoadCurveAngle);
ZROS_TOPIC_DEFINE(imu, synapse_msgs_Imu);
ZROS_TOPIC_DEFINE(joy, synapse_msgs_Joy);
ZROS_TOPIC_DEFINE(led_array, synapse_msgs_LEDArray);

static struct zros_topic* topic_list[] = {
    &topic_actuators_manual,
    &topic_actuators_auto,
    &topic_actuators,

    &topic_status,
    &topic_road_curve_angle,
    &topic_imu,
    &topic_joy,
    &topic_led_array,
};

static int set_topic_list()
{
    for (size_t i = 0; i < ARRAY_SIZE(topic_list); i++) {
        zros_broker_add_topic(topic_list[i]);
    }
    return 0;
}

SYS_INIT(set_topic_list, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);

// vi: ts=4 sw=4 et
