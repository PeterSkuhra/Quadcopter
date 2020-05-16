#ifndef DEVICES_WIRING_HPP
#define DEVICES_WIRING_HPP

#include <Arduino.h>
#include <ArduinoSTL.h>

namespace wiring
{

/******************************************************************************
 *  FlySky receiver
 *****************************************************************************/
#define CHANNEL_COUNT   10

#define CHANNEL1_PIN    A8      //  Thrust
#define CHANNEL2_PIN    A9      //  Yaw
#define CHANNEL3_PIN    A10     //  Pitch
#define CHANNEL4_PIN    A11     //  Roll
#define CHANNEL5_PIN    A12
#define CHANNEL6_PIN    A13
#define CHANNEL7_PIN    A14
#define CHANNEL8_PIN    A15
#define CHANNEL9_PIN    14
#define CHANNEL10_PIN   15

static const std::vector<uint8_t> kReceiverPins {
    CHANNEL1_PIN,
    CHANNEL2_PIN,
    CHANNEL3_PIN,
    CHANNEL4_PIN,
    CHANNEL5_PIN,       // Kratsi cyklus
    CHANNEL6_PIN,
    CHANNEL7_PIN,       // 5
    CHANNEL8_PIN,       // 6
    CHANNEL9_PIN,
    CHANNEL10_PIN
};


/*
 *  I2C
 *
 *  20 - SDA
 *  21 - SCL
 */
/******************************************************************************
 *  MPU6050 Gyroscope & Accelerometer
 *****************************************************************************/
#define MPU6050_I2C_ADDRESS     0x68
#define MPU6050_INT_PIN         2

/******************************************************************************
 *  Ultrasonic sensor
 *****************************************************************************/
// #define 23
// #define 25

/******************************************************************************
 *  BMP180 Barometer
 *****************************************************************************/
#define BMP180_I2C_ADDRESS      0x77

/******************************************************************************
 *  Battery voltage sensor
 *****************************************************************************/
#define BATTERY_VOLTAGE_SENSOR_PIN          A0
#define BATTERY_VOLTAGE_SENSOR_RESOLUTION   (0.024414)      // 25V / 1024

/******************************************************************************
 *  Brushless motors
 *****************************************************************************/
#define MOTOR_COUNT             4

#define MOTOR_FRONT_RIGHT_PIN   6
#define MOTOR_FRONT_LEFT_PIN    7
#define MOTOR_BACK_RIGHT_PIN    8
#define MOTOR_BACK_LEFT_PIN     11

static const std::vector<uint8_t> kESCPins {
    MOTOR_FRONT_RIGHT_PIN,
    MOTOR_FRONT_LEFT_PIN,
    MOTOR_BACK_RIGHT_PIN,
    MOTOR_BACK_LEFT_PIN
};

// Motor position
#define FRONT_RIGHT     0
#define FRONT_LEFT      1
#define BACK_RIGHT      2
#define BACK_LEFT       3

/******************************************************************************
 *  LED Indicators
 *****************************************************************************/
#define LED_RED_PIN         22
#define LED_ORANGE_PIN      24
#define LED_GREEN_PIN       26


}

#endif
