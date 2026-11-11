#include "include/pid_storage.h"

#include <array>
#include "nvs.h"
#include "nvs_flash.h"

namespace robot::pid_storage
{
    namespace
    {
        struct Values
        {
            float kp, ki, kd;
            float integrator_min, integrator_max;
            float output_min, output_max;
        };

        constexpr std::array keys = {"motor1", "motor2", "motor3", "motor4", "yaw"};

        // Fill in kp, ki, kd, integrator_min, integrator_max, output_min, output_max.
        // These defaults are written only when the corresponding NVS record is absent.
        constexpr std::array<Values, 5> defaults = {{
            {0, 0, 0, 0, 0, 0, 0}, // Motor 1
            {0, 0, 0, 0, 0, 0, 0}, // Motor 2
            {0, 0, 0, 0, 0, 0, 0}, // Motor 3
            {0, 0, 0, 0, 0, 0, 0}, // Motor 4
            {0, 0, 0, 0, 0, 0, 0}, // Yaw
        }};
    }

    esp_err_t initialize()
    {
        esp_err_t err = nvs_flash_init();
        if (err != ESP_OK) return err;

        nvs_handle_t handle;
        err = nvs_open("pid", NVS_READWRITE, &handle);
        if (err != ESP_OK) return err;

        for (size_t i = 0; i < keys.size() && err == ESP_OK; ++i)
        {
            size_t size = 0;
            err = nvs_get_blob(handle, keys[i], nullptr, &size);
            if (err == ESP_ERR_NVS_NOT_FOUND)
                err = nvs_set_blob(handle, keys[i], &defaults[i], sizeof(Values));
            else if (err == ESP_OK && size != sizeof(Values))
                err = ESP_ERR_NVS_INVALID_LENGTH;
        }
        if (err == ESP_OK) err = nvs_commit(handle);
        nvs_close(handle);
        return err;
    }

    esp_err_t load(const Controller controller, espp::Pid::Config& config)
    {
        const auto index = static_cast<size_t>(controller);
        if (index >= keys.size()) return ESP_ERR_INVALID_ARG;

        nvs_handle_t handle;
        esp_err_t err = nvs_open("pid", NVS_READONLY, &handle);
        if (err != ESP_OK) return err;

        Values values{};
        size_t size = sizeof(values);
        err = nvs_get_blob(handle, keys[index], &values, &size);
        nvs_close(handle);
        if (err != ESP_OK) return err;
        if (size != sizeof(values)) return ESP_ERR_NVS_INVALID_LENGTH;

        config.kp = values.kp;
        config.ki = values.ki;
        config.kd = values.kd;
        config.integrator_min = values.integrator_min;
        config.integrator_max = values.integrator_max;
        config.output_min = values.output_min;
        config.output_max = values.output_max;
        return ESP_OK;
    }
}
