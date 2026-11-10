/**
 * robot_web.h - 机器人 Web 控制台对外接口
 *
 * 设计要点：
 *   1. 所有业务逻辑（定时采集、WS 推送、HTTP 副作用）集中在一个 FreeRTOS 任务里。
 *   2. HTTP handler 只做轻量解析 + 投递队列，不阻塞。
 *   3. 共享数据用 mutex 保护快照一致性。
 */
#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "esp_err.h"
#include "esp_http_server.h"

/* ---------- 类型定义 ---------- */

/* IMU 六轴 + 姿态快照 */
typedef struct {
    float acc_x, acc_y, acc_z;       /* g */
    float gyr_x, gyr_y, gyr_z;       /* deg/s */
    float att_pitch, att_roll, att_yaw; /* deg */
    uint32_t tick_ms;                 /* 采集时刻（系统 tick） */
} imu_snapshot_t;

/* 单轮 PID */
typedef struct {
    float kp;
    float ki;
    float kd;
    float out_min;
    float out_max;
} wheel_pid_t;

/* 四个轮子 */
typedef struct {
    wheel_pid_t wheels[4]; /* FL, FR, RL, RR */
} pid_set_t;

/* 动作类型（由 HTTP handler 投递，主任务消费） */
typedef enum {
    ACT_SET_PID,           /* 改某一轮 PID */
    ACT_SET_ALL_PID,       /* 一键写四轮 */
    ACT_CALIBRATE_IMU,     /* 重新校准 IMU */
    ACT_REBOOT,            /* 预留 */
} action_type_t;

typedef struct {
    action_type_t type;
    uint8_t       wheel_idx; /* 0..3, 仅 PID 相关 */
    pid_set_t     pid;       /* 批量写四轮时用 */
    wheel_pid_t   one_pid;   /* 单轮写时用 */
} action_t;

/* ---------- 对外 API ---------- */

/* 启动 Web 控制台（内部创建 FreeRTOS 任务 + 启动 HTTP Server） */
esp_err_t robot_web_start(void);

/* ---- 以下四个函数是"操作函数打桩"，由用户自己补全硬件逻辑 ---- */

/* 读取一份 IMU 最新快照（打桩：内部可以先填假数据） */
void robot_imu_read(imu_snapshot_t *out);

/* 重新校准 IMU（打桩） */
esp_err_t robot_imu_calibrate(void);

/* 写入单轮 PID（打桩：下发到电机控制环） */
esp_err_t robot_pid_set_one(uint8_t wheel_idx, const wheel_pid_t *pid);

/* 读取电池电压，单位 V（打桩） */
float robot_battery_read_v(void);

/* OTA 固件上传入口（打桩：真正的 OTA 逻辑自己补） */
esp_err_t robot_ota_begin(size_t image_size);
esp_err_t robot_ota_write(const uint8_t *buf, size_t len);
esp_err_t robot_ota_end(void);

#ifdef __cplusplus
}
#endif
