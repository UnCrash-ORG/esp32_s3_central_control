//
// Created by 35264 on 2026/11/7.
//

#ifndef ROBOT_PIN_CONFIG_H
#define ROBOT_PIN_CONFIG_H
#include "driver/gpio.h"
#include "esp_adc/adc_oneshot.h"

namespace robot::pins
{
    //MPU6050
    constexpr gpio_num_t MPU6050_SCL = GPIO_NUM_41;
    constexpr gpio_num_t MPU6050_SDA = GPIO_NUM_40;
    constexpr gpio_num_t MPU6050_INT = GPIO_NUM_39;

    //STEPPER
    constexpr gpio_num_t STEP1_DIR_PIN = GPIO_NUM_7;
    constexpr gpio_num_t STEP1_STEP_PIN = GPIO_NUM_15;
    constexpr gpio_num_t STEP1_EN_PIN = GPIO_NUM_16;

    constexpr gpio_num_t STEP2_DIR_PIN = GPIO_NUM_17;
    constexpr gpio_num_t STEP2_STEP_PIN = GPIO_NUM_18;
    constexpr gpio_num_t STEP2_EN_PIN = GPIO_NUM_8;

    //Motor PWN
    constexpr gpio_num_t MOTOR1_IN1_PIN = GPIO_NUM_21;
    constexpr gpio_num_t MOTOR1_IN2_PIN = GPIO_NUM_47;

    constexpr gpio_num_t MOTOR2_IN1_PIN = GPIO_NUM_48;
    constexpr gpio_num_t MOTOR2_IN2_PIN = GPIO_NUM_45;

    constexpr gpio_num_t MOTOR3_IN1_PIN = GPIO_NUM_35;
    constexpr gpio_num_t MOTOR3_IN2_PIN = GPIO_NUM_36;

    constexpr gpio_num_t MOTOR4_IN1_PIN = GPIO_NUM_37;
    constexpr gpio_num_t MOTOR4_IN2_PIN = GPIO_NUM_38;

    //Motor PCNT
    constexpr gpio_num_t ENC1_A_PIN = GPIO_NUM_46;
    constexpr gpio_num_t ENC1_B_PIN = GPIO_NUM_3;

    constexpr gpio_num_t ENC2_A_PIN = GPIO_NUM_10;
    constexpr gpio_num_t ENC2_B_PIN = GPIO_NUM_9;

    constexpr gpio_num_t ENC3_A_PIN = GPIO_NUM_12;
    constexpr gpio_num_t ENC3_B_PIN = GPIO_NUM_11;

    constexpr gpio_num_t ENC4_A_PIN = GPIO_NUM_14;
    constexpr gpio_num_t ENC4_B_PIN = GPIO_NUM_13;

    //Servo
    constexpr gpio_num_t SERVO1_PIN = GPIO_NUM_5;
    constexpr gpio_num_t SERVO2_PIN = GPIO_NUM_4;
    constexpr gpio_num_t SERVO3_PIN = GPIO_NUM_1;
    constexpr gpio_num_t SERVO4_PIN = GPIO_NUM_2;

    //SYSTEM
    constexpr adc_channel_t BAT_CHK_ADC = ADC_CHANNEL_5;
    constexpr gpio_num_t BOOT_KEY_PIN = GPIO_NUM_0;

    // UART
    constexpr gpio_num_t UART0_TXD_PIN = GPIO_NUM_43; // TXD0
    constexpr gpio_num_t UART0_RXD_PIN = GPIO_NUM_44; // RXD0

    // USB
    constexpr gpio_num_t USB_DM_PIN = GPIO_NUM_19; // DN
    constexpr gpio_num_t USB_DP_PIN = GPIO_NUM_20; // DP

}

#endif //ROBOT_PIN_CONFIG_H
