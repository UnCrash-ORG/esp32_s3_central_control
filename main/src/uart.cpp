//
// Created by 35264 on 2026/11/8.
//


#include <cstring>
#include <mutex>
#include <esp_err.h>
#include "driver/uart.h"
#include "freertos/queue.h"
#include "../include/my_tasks.h"
#include "../include/pin_config.h"
#include "../include/uart.h"
#include "esp_crc.h"

static constexpr int UART_BUFFER_SIZE = 1024;
static constexpr uart_port_t UART_NUM = UART_NUM_0;
using namespace robot;

static size_t get_expected_payload_len(const uart::msg_type type)
{
    switch (type)
    {
    case uart::msg_type::ACQUIRE_IMU:
        return sizeof(uart::payload_acquire_imu);
    case uart::msg_type::ACQUIRE_SET_SPEED:
        return sizeof(uart::payload_acquire_set_speed);
    case uart::msg_type::ACQUIRE_SET_SERVO:
        return sizeof(uart::payload_acquire_set_servo);
    case uart::msg_type::ACQUIRE_SET_STEPPER_SPEED:
        return sizeof(uart::payload_acquire_set_stepper_speed);
    case uart::msg_type::ACQUIRE_SET_STEPPER_MOVE:
        return sizeof(uart::payload_acquire_set_stepper_move);
    case uart::msg_type::ACQUIRE_SET_STEPPER_MOVE_TO:
        return sizeof(uart::payload_acquire_set_stepper_move_to);
    default:
        return 0;
    }
}

static void reply_ok(const uart_port_t uart_num)
{
    uart::packet send_msg{};
    memcpy(send_msg.header.magic, uart::MAGIC_NUM, 2);
    send_msg.header.type = uart::msg_type::RESPOND_OK;
    send_msg.header.payload_len = sizeof(uart::payload_respond_ok);
    send_msg.payload.respond_ok.crc32 = esp_crc32_le(
        0,
        reinterpret_cast<const uint8_t*>(&send_msg),
        sizeof(uart::frame_header) + sizeof(uart::payload_respond_ok) - sizeof(uint32_t)
    );
    uart_write_bytes(uart_num, &send_msg, sizeof(uart::frame_header) + sizeof(uart::payload_respond_ok));
}


[[ noreturn ]] void tasks::uart_task(void* pvParameter)
{
    auto sharing_contents = static_cast<robot::sharing_contents*>(pvParameter);
    // Setup UART buffered IO with event queue
    QueueHandle_t uart_queue = nullptr;
    // Install UART driver using an event queue here
    ESP_ERROR_CHECK(uart_driver_install(UART_NUM, UART_BUFFER_SIZE, UART_BUFFER_SIZE, 10, &uart_queue, 0));
    constexpr uart_port_t uart_num = UART_NUM;
    constexpr uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_CTS_RTS,
        .rx_flow_ctrl_thresh = 122,
    };
    // Configure UART parameters
    ESP_ERROR_CHECK(uart_param_config(uart_num, &uart_config));
    //Set UART GPIO
    ESP_ERROR_CHECK(uart_set_pin(
        UART_NUM_0, pins::UART0_TXD_PIN, pins::UART0_RXD_PIN,
        UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

    // Configure a UART interrupt threshold and timeout
    constexpr uart_intr_config_t uart_intr = {
        .intr_enable_mask = UART_DATA,
        .rx_timeout_thresh = 10,
        .rxfifo_full_thresh = 100,
    };
    ESP_ERROR_CHECK(uart_intr_config(uart_num, &uart_intr));
    // Enable UART RX FIFO full threshold and timeout interrupts
    ESP_ERROR_CHECK(uart_enable_rx_intr(uart_num));


    while (true)
    {
        uart_event_t event{};
        if (xQueueReceive(uart_queue, &event, portMAX_DELAY) != pdTRUE || event.type != UART_DATA) continue;

        while (true)
        {
            uint8_t byte = 0;
            bool got_first_magic = false;
            bool header_synced = false;
            while (uart_read_bytes(uart_num, &byte, 1, got_first_magic ? pdMS_TO_TICKS(5) : 0) == 1)
            {
                if (got_first_magic)
                {
                    if (byte == uart::MAGIC_NUM[1])
                    {
                        header_synced = true;
                        break;
                    }
                    got_first_magic = (byte == uart::MAGIC_NUM[0]);
                }
                else if (byte == uart::MAGIC_NUM[0])
                {
                    got_first_magic = true;
                }
            }
            if (!header_synced) break;

            uart::packet recv_msg{};
            recv_msg.header.magic[0] = uart::MAGIC_NUM[0];
            recv_msg.header.magic[1] = uart::MAGIC_NUM[1];

            // Read remaining 2 bytes of frame_header (type + payload_len)
            if (uart_read_bytes(
                uart_num,
                reinterpret_cast<uint8_t*>(&recv_msg.header) + 2,
                sizeof(uart::frame_header) - 2,
                pdMS_TO_TICKS(10)
            ) != static_cast<int>(sizeof(uart::frame_header) - 2))
            {
                break;
            }

            const size_t expected_payload_len = get_expected_payload_len(recv_msg.header.type);
            if (expected_payload_len == 0) continue;

            if (uart_read_bytes(
                uart_num,
                recv_msg.payload.raw_bytes,
                static_cast<uint32_t>(expected_payload_len),
                pdMS_TO_TICKS(10)
            ) != static_cast<int>(expected_payload_len))
            {
                break;
            }

            // Check recv crc32
            const uint32_t true_crc32 = esp_crc32_le(
                0,
                reinterpret_cast<const uint8_t*>(&recv_msg),
                sizeof(uart::frame_header) + expected_payload_len - sizeof(uint32_t)
            );

            switch (recv_msg.header.type)
            {
            case uart::msg_type::ACQUIRE_IMU:
                {
                    if (true_crc32 != recv_msg.payload.acquire_imu.crc32) continue;

                    const auto imu_data = sharing_contents->devices.imu.get_data();

                    auto send_msg = uart::packet{};
                    memcpy(send_msg.header.magic, uart::MAGIC_NUM, 2);
                    send_msg.header.type = uart::msg_type::RESPOND_IMU;
                    send_msg.header.payload_len = sizeof(uart::payload_respond_imu);
                    memcpy(&send_msg.payload.respond_imu, &imu_data, sizeof(imu_data));
                    send_msg.payload.respond_imu.crc32 = esp_crc32_le(
                        0,
                        reinterpret_cast<const uint8_t*>(&send_msg),
                        sizeof(uart::frame_header) + sizeof(uart::payload_respond_imu) - sizeof(uint32_t)
                    );
                    uart_write_bytes(uart_num, &send_msg,
                                     sizeof(uart::frame_header) + sizeof(uart::payload_respond_imu));
                    break;
                }

            case uart::msg_type::ACQUIRE_SET_SPEED:
                {
                    if (true_crc32 != recv_msg.payload.set_speed.crc32) continue;

                    const auto target_vx = recv_msg.payload.set_speed.vx_m;
                    const auto target_vy = recv_msg.payload.set_speed.vy_m;
                    const auto target_yaw = recv_msg.payload.set_speed.yaw_rad;
                    // Enter sharing variables critical
                    {
                        auto lock = std::lock_guard(sharing_contents->sharing_variables.target_speed_mutex);
                        sharing_contents->sharing_variables.target_vx_m = target_vx;
                        sharing_contents->sharing_variables.target_vy_m = target_vy;
                        sharing_contents->sharing_variables.target_yaw_rad = target_yaw;
                    }
                    // Leave sharing variables critical
                    reply_ok(uart_num);
                    break;
                }

            case uart::msg_type::ACQUIRE_SET_STEPPER_SPEED:
                {
                    if (true_crc32 != recv_msg.payload.set_stepper_speed.crc32) continue;

                    for (int i = 0; i < uart::NUM_OF_STEPPER; i++)
                    {
                        sharing_contents->devices.steppers[i]->setSpeedInHz(
                            recv_msg.payload.set_stepper_speed.steppers_speed[i]);
                        sharing_contents->devices.steppers[i]->setAcceleration(
                            recv_msg.payload.set_stepper_speed.steppers_acc[i]);
                    }

                    reply_ok(uart_num);
                    break;
                }

            case uart::msg_type::ACQUIRE_SET_STEPPER_MOVE:
                {
                    if (true_crc32 != recv_msg.payload.set_stepper_move.crc32) continue;

                    for (int i = 0; i < uart::NUM_OF_STEPPER; i++)
                    {
                        sharing_contents->devices.steppers[i]->move(recv_msg.payload.set_stepper_move.steppers_move[i],
                                                                    false);
                    }

                    reply_ok(uart_num);
                    break;
                }

            case uart::msg_type::ACQUIRE_SET_STEPPER_MOVE_TO:
                {
                    if (true_crc32 != recv_msg.payload.set_stepper_move_to.crc32) continue;

                    for (int i = 0; i < uart::NUM_OF_STEPPER; i++)
                    {
                        if ((1U << i) & recv_msg.payload.set_stepper_move_to.select_stepper)
                        {
                            sharing_contents->devices.steppers[i]->moveTo(
                                recv_msg.payload.set_stepper_move_to.steppers_move_to[i],
                                false
                            );
                        }
                    }

                    reply_ok(uart_num);
                    break;
                }

            case uart::msg_type::ACQUIRE_SET_SERVO:
                {
                    if (true_crc32 != recv_msg.payload.set_servo.crc32) continue;

                    for (int i = 0; i < uart::NUM_OF_SERVOS; i++)
                    {
                        iot_servo_write_angle(
                            sharing_contents->devices.servos[i],
                            recv_msg.payload.set_servo.servo_angles[i]
                        );
                    }

                    reply_ok(uart_num);
                    break;
                }

            default:
                break;
            }
        }
    }
}
