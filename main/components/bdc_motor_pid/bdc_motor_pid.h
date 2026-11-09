//
// Created by 35264 on 2026/11/7.
//

#ifndef ROBOT_BDC_MOTOR_H
#define ROBOT_BDC_MOTOR_H

#include <driver/pulse_cnt.h>

#include "bdc_motor.h"
#include "driver/gpio.h"
#include "pid.hpp"
#include "atomic"
#include "mutex"
#include <limits>

namespace robot
{

    constexpr short int PULSE_CNT_HIGH_LIMIT = std::numeric_limits<short int>::max();
    constexpr short int PULSE_CNT_LOW_LIMIT = std::numeric_limits<short int>::min();
    constexpr int PWM_FREQ_HZ = 200000;
    constexpr int MCPWM_RESOLUTION_HZ = 10000000;
    constexpr int MAX_GLITCH_NS = 1000;

    class bdc_motor_pid: private espp::Pid
    {
    public:
        struct motor_config_t{
            int mcpwm_group;
            int ticks_per_lap;
            gpio_num_t pwm_a;
            gpio_num_t pwm_b;
            gpio_num_t cnt_a;
            gpio_num_t cnt_b;
        };
        bdc_motor_pid(const motor_config_t& motor_config, const espp::Pid::Config& pid_config);
        void control_task(float target_rps, float pitch);
        void temporary_override_pid(const Config& config);
        float get_now_rps();

        float kg = 0;

    private:
        motor_config_t motor_config;
        bdc_motor_handle_t motor_handle{};
        pcnt_unit_handle_t pcnt_unit_handle{};
        pcnt_channel_handle_t pcnt_chan_a_handle{};
        pcnt_channel_handle_t pcnt_chan_b_handle{};

        std::atomic<float> now_rps;
        uint64_t last_seen= 0;
        std::mutex config_lock;
    };
}

#endif //ROBOT_BDC_MOTOR_H
