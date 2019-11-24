#ifndef DEVICES_WIRING_HPP
#define DEVICES_WIRING_HPP

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
#define CHANNEL9_PIN    10
#define CHANNEL10_PIN   11

const uint8_t kReceiverPins[CHANNEL_COUNT] = {
    CHANNEL1_PIN,
    CHANNEL2_PIN,
    CHANNEL3_PIN,
    CHANNEL4_PIN,
    CHANNEL5_PIN,
    CHANNEL6_PIN,
    CHANNEL7_PIN,
    CHANNEL8_PIN,
    CHANNEL9_PIN,
    CHANNEL10_PIN
};

/******************************************************************************
 *  MPU6050 Gyroscope & Accelerometer
 *****************************************************************************/
#define MPU6050_I2C_ADDRESS     0x68

/******************************************************************************
 *  BMP180 Barometer
 *****************************************************************************/
#define BMP180_I2C_ADDRESS      0x77

/******************************************************************************
 *  Battery voltage sensor
 *****************************************************************************/
#define BATTERY_VOLTAGE_SENSOR_PIN          A0
#define BATTERY_VOLTAGE_SENSOR_RESOLUTION   0.048828

/******************************************************************************
 *  Brushless motors
 *****************************************************************************/
#define MOTOR_FRONT_RIGHT_PIN   4
#define MOTOR_FRONT_LEFT_PIN    5
#define MOTOR_BACK_RIGHT_PIN    6
#define MOTOR_BACK_LEFT_PIN     7

#endif
