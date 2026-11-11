#pragma once

#include "esp_err.h"
#include "pid.hpp"

namespace robot::pid_storage
{
    enum class Controller { motor1, motor2, motor3, motor4, yaw };

    // Initializes the default NVS partition and creates any missing PID records.
    esp_err_t initialize();
    esp_err_t load(Controller controller, espp::Pid::Config& config);
}
