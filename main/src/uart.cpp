//
// Created by 35264 on 2026/11/8.
//


#include <esp_err.h>
#include "driver/uart.h"
#include "freertos/queue.h"
#include "../include/my_tasks.h"
#include "../include/pin_config.h"
#include "../include/uart.h"
#include "esp_crc.h"

static constexpr int UART_BUFFER_SIZE = 1024;
static constexpr int UART_MAX_PACKET_LENGTH = 44;
static constexpr uart_port_t UART_NUM = UART_NUM_0;
using namespace robot;

[[ noreturn ]] void tasks::uart_task(void* pvParameter)
{
    auto sharing_contents = static_cast<robot::sharing_contents*>(pvParameter);
    // Setup UART buffered IO with event queue
    QueueHandle_t uart_queue;
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
        uart_event_t event;
        xQueueReceive(uart_queue, &event, portMAX_DELAY);
        if (event.type != UART_DATA) continue;

        uint8_t rx_buffer[UART_MAX_PACKET_LENGTH];
        int length = 0;
        ESP_ERROR_CHECK(uart_get_buffered_data_len(UART_NUM, reinterpret_cast<size_t*>(&length)));
        length = uart_read_bytes(uart_num, rx_buffer, length, 0);
        const uart::packet* recv_msg = reinterpret_cast<uart::packet*>(rx_buffer);

        //Make sure Packet header is right
        if (recv_msg != nullptr and memcmp(recv_msg->header.magic, uart::MAGIC_NUM, 2) != 0) continue;
        uart::packet send_msg{};
        memcpy(send_msg.header.magic, uart::MAGIC_NUM, 2);
        switch (recv_msg->header.type)
        {
        case uart::msg_type::ACQUIRE_IMU:
            {
                //Check recv crc32
                const auto true_crc32 = esp_crc32_le(
                    0,
                    reinterpret_cast<uint8_t*>(&recv_msg),
                    sizeof(uart::frame_header) + sizeof(uart::payload_acquire_imu) - 4
                );
                if (true_crc32 != recv_msg->payload.acquire_imu.crc32) continue;

                const auto imu_data = sharing_contents->devices.imu.get_data();
                send_msg.header.type = uart::msg_type::RESPOND_IMU;
                send_msg.header.payload_len = sizeof(uart::payload_respond_imu);
                memcpy(&send_msg.payload.respond_imu, &imu_data, sizeof(imu_data));
                send_msg.payload.respond_imu.crc32 = esp_crc32_le(
                    0,
                    reinterpret_cast<uint8_t*>(&send_msg),
                    sizeof(uart::frame_header) + sizeof(
                        uart::payload_respond_imu) - 4);
                uart_write_bytes(uart_num, &send_msg, sizeof(uart::frame_header) + sizeof(uart::payload_respond_imu));
                break;
            }
        case uart::msg_type::ACQUIRE_SET_SPEED:
            {
                //Check recv crc32
                const auto true_crc32 = esp_crc32_le(
                    0,
                    reinterpret_cast<uint8_t*>(&recv_msg),
                    sizeof(uart::frame_header) + sizeof(uart::payload_acquire_set_speed) - 4
                );
                if (true_crc32 != recv_msg->payload.set_speed.crc32) continue;

                const auto target_vx = recv_msg->payload.set_speed.vx_m;
                const auto target_vy = recv_msg->payload.set_speed.vy_m;
                const auto target_yaw = recv_msg->payload.set_speed.yaw_rad;
                // Enter sharing variables critical
                sharing_contents->sharing_variables.target_speed_mutex.lock();
                sharing_contents->sharing_variables.target_vx_m = target_vx;
                sharing_contents->sharing_variables.target_vy_m = target_vy;
                sharing_contents->sharing_variables.target_yaw_rad = target_yaw;
                sharing_contents->sharing_variables.target_speed_mutex.unlock();
                // Leave sharing variables critical
                send_msg.header.type = uart::msg_type::RESPOND_OK;
                send_msg.header.payload_len = sizeof(uart::payload_respond_ok);
                send_msg.payload.respond_ok.crc32 = esp_crc32_le(
                    0,
                    reinterpret_cast<uint8_t*>(&send_msg),
                    sizeof(uart::frame_header) + sizeof(uart::payload_respond_ok) - 4
                );
                uart_write_bytes(uart_num, &send_msg, sizeof(uart::frame_header) + sizeof(uart::payload_respond_ok));
                break;
            }
        case uart::msg_type::ACQUIRE_SET_STEPPER:
            {
                //Check recv crc32
                const auto true_crc32 = esp_crc32_le(
                    0,
                    reinterpret_cast<uint8_t*>(&recv_msg),
                    sizeof(uart::frame_header) + sizeof(uart::payload_acquire_set_stepper) - 4
                );
                if (true_crc32 != recv_msg->payload.set_stepper.crc32) continue;

                const auto stepper1_speed = recv_msg->payload.set_stepper.stepper1_speed;

                break;
            }
        case uart::msg_type::ACQUIRE_SET_SERVO:
            {
                //Check recv crc32
                const auto true_crc32 = esp_crc32_le(
                    0,
                    reinterpret_cast<uint8_t*>(&recv_msg),
                    sizeof(uart::frame_header) + sizeof(uart::payload_acquire_set_servo) - 4
                );
                if (true_crc32 != recv_msg->payload.set_servo.crc32) continue;


                break;
            }
        default: break;
        }
    }
}
