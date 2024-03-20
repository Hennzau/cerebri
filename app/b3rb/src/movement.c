//
//    Created by Enzo Le Van
//    Ce code récupère les différents actuators, ainsi que le status du rover : il envoie alors le bon actuator aux moteurs
//

#include "casadi/gen/b3rb.h"
#include "math.h"

#include <zephyr/logging/log.h>

#include <zros/private/zros_node_struct.h>
#include <zros/private/zros_pub_struct.h>
#include <zros/private/zros_sub_struct.h>
#include <zros/zros_node.h>
#include <zros/zros_pub.h>
#include <zros/zros_sub.h>

#include <cerebri/core/casadi.h>

#include "mixing.h"

#define MY_STACK_SIZE 3072
#define MY_PRIORITY 4

LOG_MODULE_REGISTER(b3rb_movement, CONFIG_CEREBRI_B3RB_LOG_LEVEL);

typedef struct _context {
    struct zros_node node;

    struct zros_sub sub_status, sub_actuators_manual, sub_actuators_auto;

    synapse_msgs_Status status;
    synapse_msgs_Actuators actuators;
    synapse_msgs_Actuators actuators_manual;
    synapse_msgs_Actuators actuators_auto;

    struct zros_pub pub_actuators;

    const double wheel_radius;
    const double wheel_base;
} context;

static context g_ctx = {
    .node = {},
    .status = synapse_msgs_Status_init_default,

    .sub_status = {},
    .sub_actuators_manual = {},
    .sub_actuators_auto = {},

    .actuators = synapse_msgs_Actuators_init_default,
    .actuators_manual = synapse_msgs_Actuators_init_default,
    .actuators_auto = synapse_msgs_Actuators_init_default,

    .pub_actuators = {},

    .wheel_radius = CONFIG_CEREBRI_B3RB_WHEEL_RADIUS_MM / 1000.0,
    .wheel_base = CONFIG_CEREBRI_B3RB_WHEEL_BASE_MM / 1000.0,
};

static void init(context* ctx)
{
    LOG_DBG("init movement");

    zros_node_init(&ctx->node, "b3rb_movement");
    zros_sub_init(&ctx->sub_status, &ctx->node, &topic_status, &ctx->status, 10);

    zros_sub_init(&ctx->sub_actuators_manual, &ctx->node,
        &topic_actuators_manual, &ctx->actuators_manual, 10);

    zros_sub_init(&ctx->sub_actuators_auto, &ctx->node,
        &topic_actuators_auto, &ctx->actuators_auto, 10);

    zros_pub_init(&ctx->pub_actuators, &ctx->node, &topic_actuators, &ctx->actuators);
}

static void stop(context* ctx)
{
    b3rb_set_actuators(&ctx->actuators, 0, 0);
}

static void b3rb_movement_entry_point(void* p0, void* p1, void* p2)
{
    LOG_INF("init");
    context* ctx = p0;
    ARG_UNUSED(p1);
    ARG_UNUSED(p2);

    init(ctx);

    struct k_poll_event events[] = {
        *zros_sub_get_event(&ctx->sub_status),
        *zros_sub_get_event(&ctx->sub_actuators_manual),
        *zros_sub_get_event(&ctx->sub_actuators_auto),
    };

    while (true) {
        k_poll(events, ARRAY_SIZE(events), K_MSEC(1000));

        if (zros_sub_update_available(&ctx->sub_status)) {
            zros_sub_update(&ctx->sub_status);
        }

        if (zros_sub_update_available(&ctx->sub_actuators_manual)) {
            zros_sub_update(&ctx->sub_actuators_manual);
        }

        if (zros_sub_update_available(&ctx->sub_actuators_auto)) {
            zros_sub_update(&ctx->sub_actuators_auto);
        }

        if (ctx->status.arming != synapse_msgs_Status_Arming_ARMING_ARMED) {
            stop(ctx);
            LOG_DBG("not armed, stopped");
        } else if (ctx->status.mode == synapse_msgs_Status_Mode_MODE_MANUAL) {
            LOG_DBG("manual mode");
            ctx->actuators = ctx->actuators_manual;
        } else if(ctx->status.mode == synapse_msgs_Status_Mode_MODE_AUTO){
            LOG_DBG("auto mode");
            ctx->actuators = ctx->actuators_auto;
        }

        // publish

        zros_pub_update(&ctx->pub_actuators);
    }
}

K_THREAD_DEFINE(b3rb_movement, MY_STACK_SIZE,
    b3rb_movement_entry_point, &g_ctx, NULL, NULL,
    MY_PRIORITY, 0, 1000);

/* vi: ts=4 sw=4 et */
