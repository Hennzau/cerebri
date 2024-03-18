/*
 * Copyright CogniPilot Foundation 2023
 * SPDX-License-Identifier: Apache-2.0
 */
#include <zephyr/device.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/net/socket.h>
#include <zephyr/posix/fcntl.h>

#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <pb_decode.h>
#include <pb_encode.h>
#include <synapse_tinyframe/SynapseTopics.h>
#include <synapse_tinyframe/TinyFrame.h>
#include <synapse_tinyframe/utils.h>

#include <zros/private/zros_node_struct.h>
#include <zros/private/zros_sub_struct.h>
#include <zros/zros_node.h>
#include <zros/zros_sub.h>

#include <synapse_protobuf/actuators.pb.h>
#include <synapse_protobuf/status.pb.h>
#include <synapse_topic_list.h>

LOG_MODULE_REGISTER(synapse_ethernet, CONFIG_CEREBRI_SYNAPSE_ETHERNET_LOG_LEVEL);

#define MY_STACK_SIZE 8192
#define MY_PRIORITY 3

#define RX_BUF_SIZE 2048
#define BIND_PORT 4242

typedef struct context_s {
    struct zros_node node;
    synapse_msgs_Actuators actuators;
    synapse_msgs_Status status;
    struct zros_sub
        sub_actuators,
        sub_status;
    TinyFrame tf;
    int client;
    volatile bool reconnect;
    int error_count;
    uint8_t rx1_buf[RX_BUF_SIZE];
    int serv;
    struct sockaddr_in bind_addr;
    int counter;
} context_t;

static context_t g_ctx = {
    .node = {},
    .actuators = synapse_msgs_Actuators_init_default,
    .status = synapse_msgs_Status_init_default,
    .sub_actuators = {},
    .sub_status = {},
    .tf = {},
    .client = 0,
    .error_count = 0,
};

#define TOPIC_LISTENER(CHANNEL, CLASS)                                            \
    static TF_Result CHANNEL##_listener(TinyFrame* tf, TF_Msg* frame)             \
    {                                                                             \
        CLASS msg = CLASS##_init_default;                                         \
        pb_istream_t stream = pb_istream_from_buffer(frame->data, frame->len);    \
        int rc = pb_decode(&stream, CLASS##_fields, &msg);                        \
        if (rc) {                                                                 \
            zros_topic_publish(&topic_##CHANNEL, &msg);                           \
            LOG_DBG("%s decoding\n", #CHANNEL);                                   \
        } else {                                                                  \
            LOG_WRN("%s decoding failed: %s\n", #CHANNEL, PB_GET_ERROR(&stream)); \
        }                                                                         \
        return TF_STAY;                                                           \
    }

#define TOPIC_PUBLISHER(DATA, CLASS, TOPIC)                                   \
    {                                                                         \
        TF_Msg msg;                                                           \
        TF_ClearMsg(&msg);                                                    \
        uint8_t buf[CLASS##_size];                                            \
        pb_ostream_t stream = pb_ostream_from_buffer((pu8)buf, sizeof(buf));  \
        int rc = pb_encode(&stream, CLASS##_fields, DATA);                    \
        if (rc) {                                                             \
            msg.type = TOPIC;                                                 \
            msg.data = buf;                                                   \
            msg.len = stream.bytes_written;                                   \
            TF_Send(&ctx->tf, &msg);                                          \
        } else {                                                              \
            printf("%s encoding failed: %s\n", #DATA, PB_GET_ERROR(&stream)); \
        }                                                                     \
    }

static void write_ethernet(TinyFrame* tf, const uint8_t* buf, uint32_t len)
{
    context_t* ctx = tf->userdata;

    if (ctx->reconnect) {
        return;
    }

    int out_len;
    const char* p;
    p = buf;
    do {
        out_len = zsock_send(ctx->client, p, len, 0);
        if (out_len < 0) {
            LOG_ERR("send: %d\n", errno);
            if (ctx->error_count++ > 100) {
                // trigger reconnect
                ctx->reconnect = true;
            }
            return;
        } else {
            // reset error count
            ctx->error_count = 0;
        }
        p += out_len;
        len -= out_len;
    } while (len);
}

static TF_Result genericListener(TinyFrame* tf, TF_Msg* msg)
{
    LOG_WRN("unhandled tinyframe type: %4d", msg->type);
    // dumpFrameInfo(msg);
    return TF_STAY;
}

// ROS -> Cerebri
TOPIC_LISTENER(joy, synapse_msgs_Joy)
TOPIC_LISTENER(clock_offset, synapse_msgs_Time)

static bool set_blocking_enabled(int fd, bool blocking)
{
    if (fd < 0)
        return false;
    int flags = zsock_fcntl(fd, F_GETFL, 0);
    if (flags == -1)
        return false;
    flags = blocking ? (flags & ~O_NONBLOCK) : (flags | O_NONBLOCK);
    return (zsock_fcntl(fd, F_SETFL, flags) == 0) ? true : false;
}

static void synapse_ethernet_init(context_t* ctx)
{
    zros_node_init(&ctx->node, "synapse_ethernet");
    zros_sub_init(&ctx->sub_actuators, &ctx->node, &topic_actuators, &ctx->actuators, 1);
    zros_sub_init(&ctx->sub_status, &ctx->node, &topic_status, &ctx->status, 1);
}

static void send_uptime(context_t* ctx)
{
    TF_Msg msg;
    TF_ClearMsg(&msg);
    uint8_t buf[synapse_msgs_Time_size];
    pb_ostream_t stream = pb_ostream_from_buffer((pu8)buf, sizeof(buf));
    int64_t ticks = k_uptime_ticks();
    int64_t sec = ticks / CONFIG_SYS_CLOCK_TICKS_PER_SEC;
    int32_t nanosec = (ticks - sec * CONFIG_SYS_CLOCK_TICKS_PER_SEC) * 1e9 / CONFIG_SYS_CLOCK_TICKS_PER_SEC;
    synapse_msgs_Time message;
    message.sec = sec;
    message.nanosec = nanosec;
    int rc = pb_encode(&stream, synapse_msgs_Time_fields, &message);
    if (rc) {
        msg.type = SYNAPSE_UPTIME_TOPIC;
        msg.data = buf;
        msg.len = stream.bytes_written;
        TF_Send(&ctx->tf, &msg);
    } else {
        printf("uptime encoding failed: %s\n", PB_GET_ERROR(&stream));
    }
}

static void ethernet_entry_point(context_t* ctx)
{
    synapse_ethernet_init(ctx);

    TF_InitStatic(&ctx->tf, TF_MASTER, write_ethernet);
    ctx->tf.userdata = ctx;

    ctx->serv = zsock_socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    set_blocking_enabled(ctx->serv, true);

    if (ctx->serv < 0) {
        LOG_ERR("socket: %d", errno);
        exit(1);
    }

    ctx->bind_addr.sin_family = AF_INET;
    ctx->bind_addr.sin_addr.s_addr = htonl(INADDR_ANY);
    ctx->bind_addr.sin_port = htons(BIND_PORT);

    if (zsock_bind(ctx->serv, (struct sockaddr*)&ctx->bind_addr, sizeof(ctx->bind_addr)) < 0) {
        LOG_ERR("bind: %d", errno);
        exit(1);
    }

    if (zsock_listen(ctx->serv, 5) < 0) {
        LOG_ERR("listen: %d", errno);
        exit(1);
    }

    // ROS -> Cerebri
    TF_AddGenericListener(&ctx->tf, genericListener);
    TF_AddTypeListener(&ctx->tf, SYNAPSE_JOY_TOPIC, joy_listener);
    TF_AddTypeListener(&ctx->tf, SYNAPSE_CLOCK_OFFSET_TOPIC, clock_offset_listener);

    while (1) {
        LOG_INF("socket waiting for connection on port: %d", BIND_PORT);
        struct sockaddr_in client_addr;
        socklen_t client_addr_len = sizeof(client_addr);
        char addr_str[32];
        ctx->client = zsock_accept(ctx->serv, (struct sockaddr*)&client_addr,
            &client_addr_len);

        if (ctx->client < 0) {
            k_msleep(1000);
            continue;
        }
        ctx->reconnect = false;

        zsock_inet_ntop(client_addr.sin_family, &client_addr.sin_addr,
            addr_str, sizeof(addr_str));
        LOG_INF("connection #%d from %s", ctx->counter++, addr_str);

        struct k_poll_event events[] = {
            *zros_sub_get_event(&ctx->sub_status),
        };

        while (!ctx->reconnect) {
            int rc = 0;
            rc = k_poll(events, ARRAY_SIZE(events), K_MSEC(1000));
            if (rc != 0) {
                LOG_WRN("poll timeout");
            }

            if (zros_sub_update_available(&ctx->sub_status)) {
                zros_sub_update(&ctx->sub_status);
                TOPIC_PUBLISHER(&ctx->status, synapse_msgs_Status, SYNAPSE_STATUS_TOPIC);
            }

            send_uptime(ctx);
            if (ctx->client < 0) {
                LOG_WRN("no client, triggering reconnect");
                break;
            }

            bool do_polling = false;

            if (do_polling) {
                struct zsock_pollfd fds;
                fds.fd = ctx->client;
                rc = zsock_poll(&fds, 1u, 1);
                if (rc >= 0) {
                    if (fds.revents & ZSOCK_POLLIN) {
                        int len = zsock_recv(ctx->client, ctx->rx1_buf, sizeof(ctx->rx1_buf), 0);
                        TF_Accept(&ctx->tf, ctx->rx1_buf, len);
                    }
                    if (fds.revents & (ZSOCK_POLLHUP | ZSOCK_POLLERR)) {
                        LOG_ERR("Socket closed/error");
                        break;
                    }
                } else {
                    LOG_ERR("poll failed: %d", rc);
                }
            } else {
                int len = zsock_recv(ctx->client, ctx->rx1_buf, sizeof(ctx->rx1_buf), 0);
                TF_Accept(&ctx->tf, ctx->rx1_buf, len);
            }
            TF_Tick(&ctx->tf);
        }
    }
}

K_THREAD_DEFINE(synapse_ethernet, MY_STACK_SIZE, ethernet_entry_point,
    &g_ctx, NULL, NULL, MY_PRIORITY, 0, 0);

/* vi: ts=4 sw=4 et */
