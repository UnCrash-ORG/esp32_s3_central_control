//
// Created by 35264 on 2026/11/7.
//

#ifndef ROBOT_MY_TASKS_H
#define ROBOT_MY_TASKS_H

#include "components/bdc_motor_pid/bdc_motor_pid.h"
#include "components/gy521/gy521.h"
#include "FastAccelStepper.h"
#include "freertos/task.h"
#include "iot_servo.h"
#include "atomic"

namespace robot
{
    constexpr int CONTROLLER_TASK_PRIORITY = configMAX_PRIORITIES - 1;
    constexpr int CONTROLLER_TASK_STACK_DEPTH = 8 * 1024;;

    constexpr int UART_TASK_PRIORITY = configMAX_PRIORITIES - 1;
    constexpr int UART_TASK_STACK_DEPTH = 8 * 1024;;

    constexpr int BAT_TASK_PRIORITY = configMAX_PRIORITIES - 2;
    constexpr int BAT_TASK_STACK_DEPTH = 4 * 1024;;

    struct sharing_contents //NOLINT
    {
        struct sharing_variables
        {
            float target_vx_m;
            float target_vy_m;
            float target_yaw_rad;
            std::atomic<float> battery_voltage_v;
            std::mutex target_speed_mutex;
        };
        struct devices
        {
            std::array<bdc_motor_pid, 4>& motors;
            std::array<FastAccelStepper*, 2>& steppers;
            std::array<servo_handle_t, 4>& servos;
            gy521& imu;
        };
        sharing_variables sharing_variables;
        devices devices;
    };
    namespace tasks
    {
        [[ noreturn ]] void controller_task(void* pvParameter);
        [[ noreturn ]] void uart_task(void* pvParameter);
        [[ noreturn ]] void battery_voltage_check(void* pvParameters);
    }
}

#endif //ROBOT_MY_TASKS_H
