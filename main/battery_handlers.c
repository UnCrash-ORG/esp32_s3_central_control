/**
 * battery_handlers.c - IMU 采集 / 校准 / 电池电压 打桩
 *
 * robot_imu_read：每 100ms 被主任务调用一次，应当返回"自上次 read 以来最新的一份快照"。
 *   真正实现时：从 I2C/SPI 读原始加速度 / 角速度寄存器，或直接读 IMU 驱动层已经
 *   解算好的姿态角。这里用正弦 + 噪声生成假数据，便于离线前端调通。
 *
 * robot_imu_calibrate：下发零偏校准。
 * robot_battery_read_v：通过 ADC 分压采样，返回转换后的电压。
 */

#include "robot_web.h"
#include "esp_log.h"
#include "esp_timer.h"
#include <math.h>
#include <stdlib.h>

static const char *TAG = "bat_imu_stub";

/* ---------- IMU 打桩 ---------- */

void robot_imu_read(imu_snapshot_t *out) {
    if (!out) return;

    /* ====== 打桩：伪造一些随时间变化的数据 ====== */
    /* 真正实现时请读取 I2C/SPI IMU，例如：
     *   qmi8658_read_acc(&ax, &ay, &az);
     *   qmi8658_read_gyr(&gx, &gy, &gz);
     *   mahony_update(...) 得到 pitch/roll/yaw;
     */
    static int64_t t0 = -1;
    if (t0 < 0) t0 = esp_timer_get_time();
    double t = (double)(esp_timer_get_time() - t0) / 1e6; /* 秒 */

    /* 模拟一个缓慢倾斜 + 轻微抖动的姿态 */
    out->acc_x = (float)(0.15 * sin(t * 0.7) + 0.02 * (rand() % 100 - 50) / 100.0);
    out->acc_y = (float)(0.10 * cos(t * 0.5) + 0.02 * (rand() % 100 - 50) / 100.0);
    out->acc_z = 1.0f; /* g */

    out->gyr_x = (float)(2.0 * cos(t * 0.6)  + 0.1 * (rand() % 100 - 50) / 100.0);
    out->gyr_y = (float)(3.0 * sin(t * 0.4)  + 0.1 * (rand() % 100 - 50) / 100.0);
    out->gyr_z = (float)(1.5 * cos(t * 0.2)  + 0.1 * (rand() % 100 - 50) / 100.0);

    out->att_pitch = (float)(5.0 * sin(t * 0.3));   /* deg */
    out->att_roll  = (float)(4.0 * cos(t * 0.35));
    out->att_yaw   = (float)(t * 20.0);             /* deg, 慢慢转 */

    out->tick_ms = (uint32_t)(t * 1000.0);
    /* ================================================= */
}

esp_err_t robot_imu_calibrate(void) {
    /* ====== 打桩：真正下发 IMU 零偏校准命令 ====== */
    ESP_LOGW(TAG, "IMU calibrate (stub) — 请保持机器人静止 2s");
    /* 示例：
     *   qmi8658_calibrate_accel();
     *   qmi8658_calibrate_gyro();
     */
    return ESP_OK;
    /* ============================================= */
}

/* ---------- 电池电压 打桩 ---------- */

float robot_battery_read_v(void) {
    /* ====== 打桩：真正读 ADC 并按分压换算 ====== */
    /* 示例：
     *   uint32_t raw = adc1_get_raw(ADC1_CHANNEL_5);
     *   // 分压比 R_up/(R_up+R_down)，参考电压 3.3V
     *   return 3.3f * raw / 4095.0f * (1.0f + 10.0f);
     */
    static int64_t t0 = -1;
    if (t0 < 0) t0 = esp_timer_get_time();
    double t = (double)(esp_timer_get_time() - t0) / 1e6;
    return 11.8f + 0.3f * (float)sin(t * 0.05); /* 慢慢飘 */
    /* ============================================ */
}
