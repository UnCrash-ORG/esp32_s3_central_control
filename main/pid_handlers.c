/**
 * pid_handlers.c - PID 控制接口打桩
 *
 * 真正的实现应当调用电机控制环（比如把参数写到一个共享的 control_loop_t 里，
 * control loop 任务在 ISR/周期性 tick 中读取并执行）。
 * 这里只打印日志，返回成功，便于前端联调。
 */

#include "robot_web.h"
#include "esp_log.h"
#include <stdio.h>

static const char *TAG = "pid_stub";

static const char *wheel_name(uint8_t idx) {
    static const char *n[] = {"FL", "FR", "RL", "RR"};
    return (idx <= 3) ? n[idx] : "??";
}

esp_err_t robot_pid_set_one(uint8_t wheel_idx, const wheel_pid_t *pid) {
    if (!pid || wheel_idx > 3) return ESP_ERR_INVALID_ARG;

    /* ====== 打桩：这里是你真正下发到电机控制环的地方 ====== */
    ESP_LOGI(TAG, "SET %s PID: kp=%.3f ki=%.3f kd=%.3f out=[%.2f, %.2f]",
             wheel_name(wheel_idx),
             pid->kp, pid->ki, pid->kd, pid->out_min, pid->out_max);
    /* ======================================================== */
    return ESP_OK;
}
