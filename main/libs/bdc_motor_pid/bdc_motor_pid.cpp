//
// Created by 35264 on 2026/11/7.
//

#include "bdc_motor_pid.h"
#include "driver/pulse_cnt.h"
#include "bdc_motor.h"
#include "esp_timer.h"

using namespace robot;

bdc_motor_pid::bdc_motor_pid(
    const motor_config_t& motor_config,
    const espp::Pid::Config& pid_config): Pid(pid_config), motor_config(motor_config)
{
    //Create PCNT Unit
    constexpr pcnt_unit_config_t pcnt_unit_config {
        .low_limit = PULSE_CNT_LOW_LIMIT,
        .high_limit = PULSE_CNT_HIGH_LIMIT,
    };
    constexpr pcnt_glitch_filter_config_t filter_config = {
        .max_glitch_ns = MAX_GLITCH_NS,
    };
    ESP_ERROR_CHECK(pcnt_new_unit(&pcnt_unit_config, &pcnt_unit_handle));
    ESP_ERROR_CHECK(pcnt_unit_set_glitch_filter(pcnt_unit_handle, &filter_config));

    //Create PCNT Channel
    const pcnt_chan_config_t pcnt_chan_config_a {
        .edge_gpio_num = motor_config.cnt_a,
        .level_gpio_num = motor_config.cnt_b,
    };
    const pcnt_chan_config_t pcnt_chan_config_b {
        .edge_gpio_num = motor_config.cnt_b,
        .level_gpio_num = motor_config.cnt_a
    };
    ESP_ERROR_CHECK(pcnt_new_channel( pcnt_unit_handle, &pcnt_chan_config_a, &pcnt_chan_a_handle ));
    ESP_ERROR_CHECK(pcnt_new_channel( pcnt_unit_handle, &pcnt_chan_config_b, &pcnt_chan_b_handle ));

    //Set PCNT ChannelA Actions
    ESP_ERROR_CHECK(pcnt_channel_set_edge_action(
    pcnt_chan_a_handle,
    PCNT_CHANNEL_EDGE_ACTION_DECREASE,
    PCNT_CHANNEL_EDGE_ACTION_INCREASE
    ));

    ESP_ERROR_CHECK(pcnt_channel_set_level_action(
        pcnt_chan_a_handle,
        PCNT_CHANNEL_LEVEL_ACTION_KEEP,
        PCNT_CHANNEL_LEVEL_ACTION_INVERSE
    ));

    //Set PCNT ChannelB Actions
    ESP_ERROR_CHECK(pcnt_channel_set_edge_action(
        pcnt_chan_b_handle,
        PCNT_CHANNEL_EDGE_ACTION_INCREASE,
        PCNT_CHANNEL_EDGE_ACTION_DECREASE
    ));
    ESP_ERROR_CHECK(pcnt_channel_set_level_action(
        pcnt_chan_b_handle,
        PCNT_CHANNEL_LEVEL_ACTION_KEEP,
        PCNT_CHANNEL_LEVEL_ACTION_INVERSE
    ));

    //Enable PCNT_Unit
    gpio_pullup_en(motor_config.cnt_a);
    gpio_pullup_en(motor_config.cnt_b);
    ESP_ERROR_CHECK(pcnt_unit_enable(pcnt_unit_handle));
    ESP_ERROR_CHECK(pcnt_unit_clear_count(pcnt_unit_handle));
    ESP_ERROR_CHECK(pcnt_unit_start(pcnt_unit_handle));


    //Create BDC Motor
    bdc_motor_config_t bdc_motor_config {
        .pwma_gpio_num = static_cast<uint32_t>(motor_config.pwm_a),
        .pwmb_gpio_num = static_cast<uint32_t>(motor_config.pwm_b),
        .pwm_freq_hz = PWM_FREQ_HZ
    };
    const bdc_motor_mcpwm_config_t bdc_motor_mcpwm_config {
        .group_id = motor_config.mcpwm_group,
        .resolution_hz = MCPWM_RESOLUTION_HZ,
    };
    ESP_ERROR_CHECK(
    bdc_motor_new_mcpwm_device(
        &bdc_motor_config,
        &bdc_motor_mcpwm_config,
        &motor_handle
    ));
    ESP_ERROR_CHECK(bdc_motor_enable(motor_handle));
}

void bdc_motor_pid::control_task(const float target_rps)
{
    //Read and reset ticks cnt
    int ticks;
    
    ESP_ERROR_CHECK(pcnt_unit_get_count(pcnt_unit_handle, &ticks));
    ESP_ERROR_CHECK(pcnt_unit_clear_count(pcnt_unit_handle));

    //Calc last rps
    if (last_seen == 0){last_seen = esp_timer_get_time(); return;}
    const auto now_time= esp_timer_get_time();
    const auto now_ticks_per_sec = static_cast<float>(ticks) / (static_cast<float>(now_time - last_seen) / 1e6f);
    now_rps = now_ticks_per_sec / static_cast<float>(motor_config.ticks_per_lap);

    last_seen = esp_timer_get_time();

    //Apply next rpm
    config_lock.lock();
    if (const auto next_speed = update(target_rps - now_rps); next_speed > 0)
    {
        config_lock.unlock();
        bdc_motor_forward(motor_handle);
        bdc_motor_set_speed(motor_handle, std::abs(std::lround(next_speed)));
    }else
    {
        config_lock.unlock();
        bdc_motor_reverse(motor_handle);
        bdc_motor_set_speed(motor_handle, std::abs(std::lround(next_speed)));
    }
}

void bdc_motor_pid::temporary_override_pid(const Config& config)
{
    std::lock_guard lock(config_lock);
    set_config(config);
}

float bdc_motor_pid::get_now_rps()
{
    return now_rps;
}
