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
    constexpr uint8_t NUM_OF_STEPPER = 2;
    constexpr uint8_t NUM_OF_SERVOS = 4;

    enum class msg_type : uint8_t
    {
        ACQUIRE_IMU = 0,
        ACQUIRE_SET_SPEED = 1,
        ACQUIRE_SET_SERVO = 2,
        ACQUIRE_SET_STEPPER_SPEED = 3,
        ACQUIRE_SET_STEPPER_MOVE = 4,
        ACQUIRE_SET_STEPPER_MOVE_TO = 5,

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
        float servo_angles[NUM_OF_SERVOS];
        uint32_t crc32; // CRC-32 calculated over header + data
    };

    // Payload for ACQUIRE_SET_STEPPER (8 bytes total: 4B data + 4B CRC32)
    struct alignas(4) payload_acquire_set_stepper_speed
    {
        uint8_t steppers_speed[NUM_OF_STEPPER]; // Steps per second
        uint8_t steppers_acc[NUM_OF_STEPPER]; // Steps per second^2
        uint32_t crc32; // CRC-32 calculated over header + data
    };

    struct alignas(4) payload_acquire_set_stepper_move
    {
        int16_t steppers_move[NUM_OF_STEPPER];
        uint32_t crc32;
    };

    struct alignas(4) payload_acquire_set_stepper_move_to
    {
        uint16_t select_stepper;
        uint16_t steppers_move_to[NUM_OF_STEPPER];
        uint16_t zero;
        uint32_t crc32;
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
        payload_acquire_set_stepper_speed set_stepper_speed;
        payload_acquire_set_stepper_move set_stepper_move;
        payload_acquire_set_stepper_move_to set_stepper_move_to;
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
    static_assert(sizeof(payload_acquire_set_stepper_speed) == 8,
                  "payload_acquire_set_stepper_speed size must be 8 bytes");
    static_assert(sizeof(payload_acquire_set_stepper_move) == 8,
                  "payload_acquire_set_stepper_move size must be 8 bytes");
    static_assert(sizeof(payload_acquire_set_stepper_move_to) == 12,
                  "payload_acquire_set_stepper_move_to size must be 12 bytes");
    static_assert(sizeof(payload_respond_imu) == 40, "payload_respond_imu size must be 40 bytes");
    static_assert(sizeof(packet) == 44, "packet container size must be 44 bytes");
    static_assert(offsetof(packet, payload) == 4, "payload offset must be 4");
}

#endif // ROBOT_UART_H
