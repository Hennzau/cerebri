/*
 * Copyright (c) 2023 CogniPilot Foundation
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef SYNAPSE_snprint_H
#define SYNAPSE_snprint_H

#include <zephyr/kernel.h>
#include <zephyr/shell/shell.h>

#include <synapse_topic_list.h>

typedef int snprint_t(char* buf, size_t n, void* msg);
int snprint_actuators(char* buf, size_t n, synapse_msgs_Actuators* m);
int snprint_header(char* buf, size_t n, synapse_msgs_Header* m);
int snprint_imu(char* buf, size_t n, synapse_msgs_Imu* m);
int snprint_joy(char* buf, size_t n, synapse_msgs_Joy* m);
int snprint_ledarray(char* buf, size_t n, synapse_msgs_LEDArray* m);
int snprint_status(char* buf, size_t n, synapse_msgs_Status* m);

#endif // SYNAPSE_snprint_LIST_H
// vi: ts=4 sw=4 et
