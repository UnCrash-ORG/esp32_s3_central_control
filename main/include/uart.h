//
// Created by 35264 on 2026/11/8.
//

#ifndef ROBOT_UART_H
#define ROBOT_UART_H

#include <cstdint>
#include <cstddef>

namespace robot::uart
{

    constexpr uint8_t MAGIC_NUM[2] = {0xAA, 0x55};

    // Frame synchronization magic words
    constexpr uint8_t FRAME_HEADER_0 = 0xAA;
    constexpr uint8_t FRAME_HEADER_1 = 0x55;

    enum class msg_type : uint8_t
    {
        ACQUIRE_IMU = 0,
        ACQUIRE_SET_SPEED = 1,
        ACQUIRE_SET_SERVO = 2,
        ACQUIRE_SET_STEPPER = 3,

        RESPOND_IMU = 255,
        RESPOND_OK = 254
    };

    // Fixed frame header (4 bytes, naturally 4-byte aligned)
    // Layout: [0xAA] [0x55] [msg_type] [payload_len]
    struct alignas(4) frame_header
    {
        uint8_t magic[2]; // Frame start delimiter: {0xAA, 0x55}
        msg_type type; // Command/Message identifier
        uint8_t payload_len; // Payload length in bytes (including payload data + crc32)
    };

    // ======================== Payload Definitions (with CRC32) ========================

    // Payload for ACQUIRE_IMU (4 bytes total: only CRC32)
    struct alignas(4) payload_acquire_imu
    {
        uint32_t crc32; // CRC-32 calculated over header
    };

    // Payload for RESPOND_OK (4 bytes total: only CRC32)
    struct alignas(4) payload_respond_ok
    {
        uint32_t crc32; // CRC-32 calculated over header
    };

    // Payload for ACQUIRE_SET_SPEED (16 bytes total: 12B data + 4B CRC32)
    struct alignas(4) payload_acquire_set_speed
    {
        float vx_m; // Linear velocity X (m/s)
        float vy_m; // Linear velocity Y (m/s)
        float yaw_rad; // Yaw (rad)
        uint32_t crc32; // CRC-32 calculated over header + data
    };

    // Payload for ACQUIRE_SET_SERVO (20 bytes total: 16B data + 4B CRC32)
    struct alignas(4) payload_acquire_set_servo
    {
        float servo1_degree; // Target angle for Servo 1
        float servo2_degree; // Target angle for Servo 2
        float servo3_degree; // Target angle for Servo 3
        float servo4_degree; // Target angle for Servo 4
        uint32_t crc32; // CRC-32 calculated over header + data
    };

    // Payload for ACQUIRE_SET_STEPPER (12 bytes total: 8B data + 4B CRC32)
    struct alignas(4) payload_acquire_set_stepper
    {
        uint8_t stepper1_speed; // Steps per second or PWM rate
        uint8_t stepper2_speed; // Steps per second or PWM rate
        int16_t stepper1_steps; // Target step displacement
        int16_t stepper2_steps; // Target step displacement
        uint32_t crc32; // CRC-32 calculated over header + data
    };

    // Payload for RESPOND_IMU (40 bytes total: 36B data + 4B CRC32)
    struct alignas(4) payload_respond_imu
    {
        float ax; // Acceleration X (m/s^2)
        float ay; // Acceleration Y (m/s^2)
        float az; // Acceleration Z (m/s^2)
        float gx; // Gyroscope X (rad/s)
        float gy; // Gyroscope Y (rad/s)
        float gz; // Gyroscope Z (rad/s)
        float yaw; // Orientation Yaw (rad)
        float pitch; // Orientation Pitch (rad)
        float roll; // Orientation Roll (rad)
        uint32_t crc32; // CRC-32 calculated over header + data
    };

    // ======================== Unified Payload Union ========================
    // Sized to the largest payload (payload_respond_imu = 40 bytes)
    union alignas(4) packet_payload
    {
        payload_acquire_imu acquire_imu;
        payload_respond_ok respond_ok;
        payload_acquire_set_speed set_speed;
        payload_acquire_set_servo set_servo;
        payload_acquire_set_stepper set_stepper;
        payload_respond_imu respond_imu;
        uint8_t raw_bytes[40];
    };

    // ======================== Unified Packet Container ========================
    // Total size: 44 bytes (4B Header + 40B Max Payload)
    struct alignas(4) packet
    {
        frame_header header; // 4 bytes
        packet_payload payload; // 40 bytes
    };

    // ======================== Compile-Time Assertions ========================
    static_assert(sizeof(frame_header) == 4, "frame_header size must be 4 bytes");
    static_assert(sizeof(payload_acquire_imu) == 4, "payload_acquire_imu size must be 4 bytes");
    static_assert(sizeof(payload_respond_ok) == 4, "payload_respond_ok size must be 4 bytes");
    static_assert(sizeof(payload_acquire_set_speed) == 16, "payload_acquire_set_speed size must be 16 bytes");
    static_assert(sizeof(payload_acquire_set_servo) == 20, "payload_acquire_set_servo size must be 20 bytes");
    static_assert(sizeof(payload_acquire_set_stepper) == 12, "payload_acquire_set_stepper size must be 12 bytes");
    static_assert(sizeof(payload_respond_imu) == 40, "payload_respond_imu size must be 40 bytes");
    static_assert(sizeof(packet) == 44, "packet container size must be 44 bytes");
    static_assert(offsetof(packet, payload) == 4, "payload offset must be 4");
}

#endif // ROBOT_UART_H
