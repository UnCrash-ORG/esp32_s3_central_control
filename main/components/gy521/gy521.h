//
// Created by 35264 on 2026/11/7.
//

#ifndef ROBOT_GY521_H
#define ROBOT_GY521_H



#include "soc/gpio_num.h"
#include "mpu6050.h"
#include "components/MahonyAHRS/MahonyAHRS.h"
#include "array"
#include "mutex"

namespace robot
{
    constexpr int CORRECTION_SAMPLES = 300;
    constexpr int CACHE_LIFETIME_MS = 10;

    struct gy521_data_t
    {
        float ax;
        float ay;
        float az;
        float gx;
        float gy;
        float gz;
        float yaw;
        float pitch;
        float roll;
    };

    class gy521: private Mahony
    {
    public:
        struct gy521_config_t //NOLINT
        {
            gpio_num_t gy521_scl;
            gpio_num_t gy521_sda ;
            bool enable_interrupt = true;
            float ratio_ax = 1.0;
            float ratio_ay = 1.0;
            float ratio_az = 1.0;
            float ratio_gx = 1.0;
            float ratio_gy = 1.0;
            float ratio_gz = 1.0;
        };
        gy521(const gy521_config_t& config);
        gy521_data_t get_data();

    private:
        gy521_config_t config;
        mpu6050_handle_t mpu6050_handle{};
        i2c_master_bus_handle_t i2c_handle{};
        uint64_t last_seen = 0;
        float correction_gx = 0;
        float correction_gy = 0;
        float correction_gz = 0;
        gy521_data_t buffer{};
    };
} // robot

#endif //ROBOT_GY521_H
