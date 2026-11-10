//
// Created by 35264 on 2026/11/7.
//

#include "include/my_tasks.h"
#include "mutex"
#include "numbers"

constexpr float DIAMETER_WHEEL_M = 0.8;
constexpr float PERIMETER_WHEEL_M = DIAMETER_WHEEL_M * std::numbers::pi_v<float>;
constexpr float K = 1; //TODO: To be measured
constexpr TickType_t FREQ_MS = pdMS_TO_TICKS(10);

[[ noreturn ]] void robot::tasks::controller_task(void* pvParameter)
{
    TickType_t xLastWakeTime;

    while (true)
    {
        auto sharing_contents = static_cast<robot::sharing_contents*>(pvParameter); //NOLINT, because exactly I will modify these motors
        const auto pitch = sharing_contents->devices.imu.get_data().pitch;

        //Entering CRITICAL
        sharing_contents->sharing_variables.target_speed_mutex.lock();
        const float target_vx = sharing_contents->sharing_variables.target_vx_m;
        const float target_vy = sharing_contents->sharing_variables.target_vy_m;
        const float target_omega_rad = sharing_contents->sharing_variables.target_omega_rad;
        sharing_contents->sharing_variables.target_speed_mutex.unlock();
        //Leaving CRITICAL

        const float v_fl_rps = (target_vx - target_vy - target_omega_rad * K) / PERIMETER_WHEEL_M;
        const float v_fr_rps = (target_vx + target_vy + target_omega_rad * K) / PERIMETER_WHEEL_M;
        const float v_rl_rps = (target_vx + target_vy - target_omega_rad * K) / PERIMETER_WHEEL_M;
        const float v_rr_rps = (target_vx - target_vy + target_omega_rad * K) / PERIMETER_WHEEL_M;

        sharing_contents->devices.motors[0].control_task(v_fl_rps, pitch);
        sharing_contents->devices.motors[0].control_task(v_fr_rps, pitch);
        sharing_contents->devices.motors[0].control_task(v_rl_rps, pitch);
        sharing_contents->devices.motors[0].control_task(v_rr_rps, pitch);

        xTaskDelayUntil( &xLastWakeTime, FREQ_MS );
    }
}
