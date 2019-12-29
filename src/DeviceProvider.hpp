#ifndef DEVICE_PROVIDER_HPP
#define DEVICE_PROVIDER_HPP

#include <Arduino.h>

#include "control/IController.hpp"
#include "command/IReceiver.hpp"
#include "display/ISender.hpp"
#include "sensing/imu/IIMU.hpp"
#include "sensing/pressure/IBarometer.hpp"
#include "sensing/ISensor.hpp"
#include "sensing/temperature/IThermometer.hpp"
#include "motor/IMotor.hpp"
#include "display/IDisplayManager.hpp"

enum MotorPosition {
    FRONT_RIGHT = 0,
    FRONT_LEFT  = 1,
    BACK_RIGHT  = 2,
    BACK_LEFT   = 3
};

class DeviceProvider
{
public:

    static DeviceProvider* GetInstance();

    ~DeviceProvider();

    command::IReceiver* GetReceiver() const;

    display::ISender* GetSender() const;

    sensing::imu::IIMU* GetIMU() const;

    sensing::pressure::IBarometer* GetBarometer() const;

    sensing::temperature::IThermometer* GetThermometer() const;

    sensing::ISensor* GetBatteryVoltageSensor() const;

    motor::IMotor* GetMotor(MotorPosition position) const;

    motor::IMotor** GetMotors() const;

    display::IDisplayManager* GetDisplayManager() const;


private:

    DeviceProvider();

private:

    command::IReceiver* receiver_;

    display::ISender* sender_;

    sensing::imu::IIMU* imu_;

    sensing::pressure::IBarometer* barometer_;

    sensing::temperature::IThermometer* thermometer_;

    sensing::ISensor* battery_voltage_sensor_;

    motor::IMotor** motors_;

    display::IDisplayManager* display_manager_;
};

#endif
