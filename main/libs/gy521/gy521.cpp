//
// Created by 35264 on 2026/11/7.
//

#include "gy521.h"
#include "esp_timer.h"
#include <driver/gpio.h>
#include "freertos/FreeRTOS.h"

namespace robot
{
    gy521::gy521(const gy521_config_t& config) : config(config)
    {
        const i2c_master_bus_config_t i2c_bus_config{
            .i2c_port = -1,
            .sda_io_num = config.gy521_sda,
            .scl_io_num = config.gy521_scl,
            .clk_source = I2C_CLK_SRC_DEFAULT,
            .glitch_ignore_cnt = 7,
            .intr_priority = -1,
            .flags = {
                .enable_internal_pullup = true,
            },
        };
        ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_bus_config, &i2c_handle));
        ESP_ERROR_CHECK(mpu6050_create(i2c_handle, MPU6050_DEFAULT_INFO(), &mpu6050_handle));

        //Set DLPF = 42 Hz
        constexpr uint8_t dlpf_data[2] = {0x1A, 0x03};
        i2c_master_transmit(mpu6050_handle.dev_handle, dlpf_data, sizeof(dlpf_data), 100);
        //Set output freq = 100 Hz
        constexpr uint8_t div_data[2] = {0x19, 0x09};
        i2c_master_transmit(mpu6050_handle.dev_handle, div_data, sizeof(div_data), 100);
        if (config.enable_interrupt)
        {
            //Set data ready interrupt parameters(Pull-Push, 50us)
            constexpr uint8_t int_pin_data[2] = {0x37, 0x00};
            i2c_master_transmit(mpu6050_handle.dev_handle, int_pin_data, sizeof(int_pin_data), 100);
            constexpr uint8_t int_en_data[2] = {0x38, 0x01};
            //Enable data ready interrupt
            i2c_master_transmit(mpu6050_handle.dev_handle, int_en_data, sizeof(int_en_data), 100);
        }

        for (int i = 0; i < CORRECTION_SAMPLES; i++)
        {
            vTaskDelay(pdMS_TO_TICKS(10));
            mpu6050_gyro_value_t gyro_value;
            ESP_ERROR_CHECK(mpu6050_get_gyro(mpu6050_handle, &gyro_value));
            correction_gx += gyro_value.gyro_x;
            correction_gy += gyro_value.gyro_y;
            correction_gz += gyro_value.gyro_z;
        }
        correction_gx /= CORRECTION_SAMPLES;
        correction_gy /= CORRECTION_SAMPLES;
        correction_gz /= CORRECTION_SAMPLES;
    }

    /**
     *
     * @return A tuple that contains ax, ay,az, gx, gy, gz
     */
    gy521_data_t gy521::get_data()
    {
        if (esp_timer_get_time() - last_seen > CACHE_LIFETIME_MS)
        {
            mpu6050_accel_value_t accel_value;
            mpu6050_gyro_value_t gyro_value;
            mpu6050_get_accel(mpu6050_handle, &accel_value);
            mpu6050_get_gyro(mpu6050_handle, &gyro_value);
            gyro_value.gyro_x -= correction_gx;
            gyro_value.gyro_y -= correction_gy;
            gyro_value.gyro_z -= correction_gz;
            const auto duration = static_cast<float>(esp_timer_get_time() - last_seen) / 1e6f;
            updateIMU(gyro_value.gyro_x, gyro_value.gyro_y, gyro_value.gyro_z, accel_value.accel_x, accel_value.accel_y, accel_value.accel_z, duration);
            const auto data = gy521_data_t{
                .ax = accel_value.accel_x,
                .ay = accel_value.accel_y,
                .az = accel_value.accel_z,
                .gx = gyro_value.gyro_x,
                .gy = gyro_value.gyro_y,
                .gz = gyro_value.gyro_z,
                .pitch = getPitch(),
                .roll = getRoll(),
                .yaw = getYaw()
            };
            last_seen = esp_timer_get_time();
            buffer = data;
            return data;
        }
        return buffer;
    }
} // robot
