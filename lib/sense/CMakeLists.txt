# SPDX-License-Identifier: Apache-2.0

add_subdirectory_ifdef(CONFIG_CEREBRI_SENSE_IMU imu)
add_subdirectory_ifdef(CONFIG_CEREBRI_SYNAPSE_VESC_CAN_STATUS vesc_can_status)
