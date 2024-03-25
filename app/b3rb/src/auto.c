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

    struct zros_sub sub_road_curve_angle;
    struct zros_pub pub_actuators;

    synapse_msgs_RoadCurveAngle road_curve_angle;
    synapse_msgs_Actuators actuators;

    const double wheel_radius;
    const double max_turn_angle;
    const double max_velocity;
} context;

static context g_ctx = {
    .node = {},

    .sub_road_curve_angle = {},
    .pub_actuators = {},

    .road_curve_angle = synapse_msgs_RoadCurveAngle_init_default,
    .actuators = synapse_msgs_Actuators_init_default,

    .wheel_radius = CONFIG_CEREBRI_B3RB_WHEEL_RADIUS_MM / 1000.0,
    .max_turn_angle = CONFIG_CEREBRI_B3RB_MAX_TURN_ANGLE_MRAD / 1000.0,
    .max_velocity = CONFIG_CEREBRI_B3RB_MAX_VELOCITY_MM_S / 1000.0,
};

static void init(context* ctx)
{
    zros_node_init(&ctx->node, "b3rb_auto");
    zros_sub_init(&ctx->sub_road_curve_angle, &ctx->node, &topic_road_curve_angle, &ctx->road_curve_angle, 10);
    zros_pub_init(&ctx->pub_actuators, &ctx->node, &topic_actuators_auto, &ctx->actuators);
}

static double compute_velocity (context* ctx, double angle) {
    ARG_UNUSED(angle);

    return ctx->max_velocity / 2;
}

static void b3rb_auto_entry_point(void* p0, void* p1, void* p2)
{
    LOG_INF("init");
    context* ctx = p0;
    ARG_UNUSED(p1);
    ARG_UNUSED(p2);

    init(ctx);

    struct k_poll_event events[] = {
        *zros_sub_get_event(&ctx->sub_road_curve_angle),
    };

    while (true) {
        k_poll(events, ARRAY_SIZE(events), K_MSEC(1000));

        if (zros_sub_update_available(&ctx->sub_road_curve_angle)) {
            zros_sub_update(&ctx->sub_road_curve_angle);
        }

        /*
            Compute actuator knowing road curve angle
        */

        double turn_angle = 0;
        double road_curve_angle = ctx->road_curve_angle.angle;

        if (road_curve_angle > ctx->max_turn_angle) {
            turn_angle = ctx->max_turn_angle;
        } else if (road_curve_angle < -ctx->max_turn_angle) {
            turn_angle = -ctx->max_turn_angle;
        } else {
            turn_angle = road_curve_angle;
        }

        double omega_fwd = compute_velocity (ctx, turn_angle);

        b3rb_set_actuators(&ctx->actuators, turn_angle, omega_fwd);

        zros_pub_update(&ctx->pub_actuators);
    }
}

K_THREAD_DEFINE(b3rb_auto, MY_STACK_SIZE,
    b3rb_auto_entry_point, (void*)&g_ctx, NULL, NULL,
    MY_PRIORITY, 0, 1000);

/* vi: ts=4 sw=4 et */
