//
// Created by 35264 on 2026/11/12.
//

#include "esp_adc/adc_oneshot.h"
#include "include/my_tasks.h"
#include "include/pin_config.h"

constexpr int BAT_CHK_PERIOD_MS = 1000;
constexpr float VOLT_RATIO = (56.0f + 10.0f) / 10.0f;

[[noreturn]] void robot::tasks::battery_voltage_check(void* pvParameters)
{
    const auto sharing_context = static_cast<sharing_contents*>(pvParameters);
    //Create ADC Unit
    adc_oneshot_unit_handle_t adc1_handle;
    constexpr adc_oneshot_unit_init_cfg_t init_config1 = {
        .unit_id = ADC_UNIT_1,
        .clk_src = ADC_RTC_CLK_SRC_DEFAULT,
        .ulp_mode = ADC_ULP_MODE_DISABLE
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config1, &adc1_handle));
    //Set ADC Channel
    constexpr adc_oneshot_chan_cfg_t config = {
        .atten = ADC_ATTEN_DB_6,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, pins::BAT_CHK_ADC, &config));
    //Apply ADC CAL
    adc_cali_handle_t cali_handle;
    constexpr adc_cali_curve_fitting_config_t cali_config = {
        .unit_id = ADC_UNIT_1,
        .chan = pins::BAT_CHK_ADC,
        .atten = ADC_ATTEN_DB_6,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };
    ESP_ERROR_CHECK(adc_cali_create_scheme_curve_fitting(&cali_config, &cali_handle));

    while (true)
    {
        int adc_raw;
        ESP_ERROR_CHECK(adc_oneshot_read(adc1_handle, pins::BAT_CHK_ADC, &adc_raw));
        int voltage_mv;
        ESP_ERROR_CHECK(adc_cali_raw_to_voltage(cali_handle, adc_raw, &voltage_mv));
        sharing_context->sharing_variables.battery_voltage_v = static_cast<float>(voltage_mv) / 1000.0f * VOLT_RATIO;
        vTaskDelay(pdMS_TO_TICKS(BAT_CHK_PERIOD_MS));
    }
}
