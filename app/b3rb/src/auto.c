//
//    Created by Enzo Le Van
//    Ce code reprend la structure du noeud "manual" mais au lieu de suivre le joystick, il suit la commande
//    obtenue à partir du noeud "vision" de la NavQ
//

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include <zros/private/zros_node_struct.h>
#include <zros/private/zros_pub_struct.h>
#include <zros/private/zros_sub_struct.h>
#include <zros/zros_node.h>
#include <zros/zros_pub.h>
#include <zros/zros_sub.h>

#include <synapse_topic_list.h>

#include "mixing.h"

#define MY_STACK_SIZE 1024
#define MY_PRIORITY 4

LOG_MODULE_REGISTER(b3rb_auto, CONFIG_CEREBRI_B3RB_LOG_LEVEL);

typedef struct _context {
    struct zros_node node;

    struct zros_pub pub_actuators;

    synapse_msgs_Actuators actuators;

    const double wheel_radius;
    const double max_turn_angle;
    const double max_velocity;
} context;

static context g_ctx = {
    .node = {},
    .joy = synapse_msgs_Joy_init_default,
    .actuators = synapse_msgs_Actuators_init_default,
    .sub_joy = {},
    .pub_actuators = {},
    .wheel_radius = CONFIG_CEREBRI_B3RB_WHEEL_RADIUS_MM / 1000.0,
    .max_turn_angle = CONFIG_CEREBRI_B3RB_MAX_TURN_ANGLE_MRAD / 1000.0,
    .max_velocity = CONFIG_CEREBRI_B3RB_MAX_VELOCITY_MM_S / 1000.0,
};

static void init(context* ctx)
{
    zros_node_init(&ctx->node, "b3rb_auto");
    zros_pub_init(&ctx->pub_actuators, &ctx->node,
        &topic_actuators_auto, &ctx->actuators);
}

static void b3rb_auto_entry_point(void* p0, void* p1, void* p2)
{
    LOG_INF("init");
    context* ctx = p0;
    ARG_UNUSED(p1);
    ARG_UNUSED(p2);

    init(ctx);

    while (true) {

    }
}

K_THREAD_DEFINE(b3rb_auto, MY_STACK_SIZE,
    b3rb_auto_entry_point, (void*)&g_ctx, NULL, NULL,
    MY_PRIORITY, 0, 1000);

/* vi: ts=4 sw=4 et */
