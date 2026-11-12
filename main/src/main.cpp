#include <fstream>
#include "include/pin_config.h"
#include "freertos/FreeRTOS.h"
#include "include/my_tasks.h"
#include "include/pid_storage.h"

using namespace robot;

extern "C" void app_main()
{
    ESP_ERROR_CHECK(pid_storage::initialize());
    std::array<espp::Pid::Config, 4> motor_pid_configs{};
    ESP_ERROR_CHECK(pid_storage::load(pid_storage::Controller::motor1, motor_pid_configs[0]));
    ESP_ERROR_CHECK(pid_storage::load(pid_storage::Controller::motor2, motor_pid_configs[1]));
    ESP_ERROR_CHECK(pid_storage::load(pid_storage::Controller::motor3, motor_pid_configs[2]));
    ESP_ERROR_CHECK(pid_storage::load(pid_storage::Controller::motor4, motor_pid_configs[3]));

    // Initialize MPU6050
    gy521 imu = gy521({
        .gy521_scl = pins::MPU6050_SCL,
        .gy521_sda = pins::MPU6050_SDA,
        .enable_interrupt = true
    });

    //Initialize BDC
    std::array bdc_motors = {
        bdc_motor_pid(
            {
                .mcpwm_group = 0,
                .ticks_per_lap = 1540,
                .pwm_a = pins::MOTOR1_IN1_PIN,
                .pwm_b = pins::MOTOR1_IN2_PIN,
                .cnt_a = pins::ENC1_A_PIN,
                .cnt_b = pins::ENC1_B_PIN,
            }, motor_pid_configs[0]),
        bdc_motor_pid(
            {
                .mcpwm_group = 0,
                .ticks_per_lap = 1540,
                .pwm_a = pins::MOTOR2_IN1_PIN,
                .pwm_b = pins::MOTOR2_IN2_PIN,
                .cnt_a = pins::ENC2_A_PIN,
                .cnt_b = pins::ENC2_B_PIN,
            }, motor_pid_configs[1]),
        bdc_motor_pid(
            {
                .mcpwm_group = 1,
                .ticks_per_lap = 1540,
                .pwm_a = pins::MOTOR3_IN1_PIN,
                .pwm_b = pins::MOTOR3_IN2_PIN,
                .cnt_a = pins::ENC3_A_PIN,
                .cnt_b = pins::ENC3_B_PIN,
            }, motor_pid_configs[2]),
        bdc_motor_pid(
            {
                .mcpwm_group = 1,
                .ticks_per_lap = 1540,
                .pwm_a = pins::MOTOR4_IN1_PIN,
                .pwm_b = pins::MOTOR4_IN2_PIN,
                .cnt_a = pins::ENC4_A_PIN,
                .cnt_b = pins::ENC4_B_PIN,
            }, motor_pid_configs[3]),
    };

    //Initialize Servo
    std::array<servo_handle_t, 4> servos = {};
    constexpr std::array servo_pins = {pins::SERVO1_PIN, pins::SERVO2_PIN, pins::SERVO3_PIN, pins::SERVO4_PIN};
    constexpr std::array servo_pwm_channels = {LEDC_CHANNEL_0, LEDC_CHANNEL_1, LEDC_CHANNEL_2, LEDC_CHANNEL_3};
    for (int i = 0; i < sizeof(servo_pins); i++)
    {
        const servo_config_t servo_cfg = SERVO_CONFIG_DEFAULT(LEDC_LOW_SPEED_MODE, LEDC_TIMER_0, servo_pwm_channels[i],
                                                              servo_pins[i]);
        iot_servo_new(&servo_cfg, &servos[i]);
    }

    //Initialize Stepper
    std::array<FastAccelStepper*, 2> steppers = {};
    auto stepper_engine = FastAccelStepperEngine();
    stepper_engine.init();
    constexpr std::array step_pins = {pins::STEP1_STEP_PIN, pins::STEP2_STEP_PIN};
    constexpr std::array dir_pins = {pins::STEP1_DIR_PIN, pins::STEP2_DIR_PIN};
    constexpr std::array en_pins = {pins::STEP1_EN_PIN, pins::STEP2_EN_PIN};
    for (int i = 0; i < sizeof(step_pins); i++)
    {
        steppers[i] = stepper_engine.stepperConnectToPin(step_pins[i]);
        steppers[i]->setDirectionPin(dir_pins[i]);
        steppers[i]->setEnablePin(en_pins[i]);
        steppers[i]->setAutoEnable(true);
    }

    //Create sub tasks
    TaskHandle_t controller_task_handle = nullptr;
    TaskHandle_t uart_task_handle = nullptr;
    TaskHandle_t battery_voltage_task_handle = nullptr;
    auto share_contents = sharing_contents{
        .sharing_variables{},
        .devices = {
            .motors = bdc_motors,
            .steppers = steppers,
            .servos = servos,
            .imu = imu
        }
    };

    xTaskCreatePinnedToCore(
        tasks::controller_task,
        "ControllerTask",
        CONTROLLER_TASK_STACK_DEPTH,
        &share_contents,
        CONTROLLER_TASK_PRIORITY,
        &controller_task_handle,
        0
    );
    xTaskCreatePinnedToCore(
        tasks::uart_task,
        "UARTTask",
        UART_TASK_STACK_DEPTH,
        &share_contents,
        UART_TASK_PRIORITY,
        &uart_task_handle,
        1
    );
    xTaskCreatePinnedToCore(
        tasks::battery_voltage_check,
        "BATTask",
        BAT_TASK_STACK_DEPTH,
        &share_contents,
        BAT_TASK_PRIORITY,
        &battery_voltage_task_handle,
        1
    );
    while (true) vTaskDelay(portMAX_DELAY);
}
