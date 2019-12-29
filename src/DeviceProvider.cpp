#include "DeviceProvider.hpp"

#include "DevicesWiring.hpp"

#include "command/PWMReceiver.hpp"
#include "display/IBusSender.hpp"
#include "sensing/imu/MPU6050.hpp"
#include "sensing/pressure/BMP180.hpp"
#include "sensing/VoltageSensor.hpp"
#include "motor/BrushlessMotor.hpp"
#include "display/OLEDManager.hpp"


DeviceProvider* DeviceProvider::GetInstance()
{
    static DeviceProvider instance;
    return &instance;
}

DeviceProvider::~DeviceProvider()
{
    delete receiver_;
    delete sender_;
    delete imu_;
    delete barometer_;
    delete thermometer_;
    delete battery_voltage_sensor_;

    for (uint8_t i = 0; i < MOTOR_COUNT; ++i) {
        delete [] motors_[i];
    }
    delete [] motors_;

    delete display_manager_;
}

command::IReceiver* DeviceProvider::GetReceiver() const
{
    return receiver_;
}

display::ISender* DeviceProvider::GetSender() const
{
    return sender_;
}

sensing::imu::IIMU* DeviceProvider::GetIMU() const
{
    return imu_;
}

sensing::pressure::IBarometer* DeviceProvider::GetBarometer() const
{
    return barometer_;
}

sensing::temperature::IThermometer* DeviceProvider::GetThermometer() const
{
    return thermometer_;
}

sensing::ISensor* DeviceProvider::GetBatteryVoltageSensor() const
{
    return battery_voltage_sensor_;
}

motor::IMotor* DeviceProvider::GetMotor(MotorPosition position) const
{
    return motors_[position];
}

motor::IMotor** DeviceProvider::GetMotors() const
{
    return motors_;
}

display::IDisplayManager* DeviceProvider::GetDisplayManager() const
{
    return display_manager_;
}

DeviceProvider::DeviceProvider()
{
    receiver_ = new command::PWMReceiver(kReceiverPins);

    sender_ = new display::IBusSender(Serial2);

    imu_ = new sensing::imu::MPU6050(MPU6050_I2C_ADDRESS);

    barometer_ = new sensing::pressure::BMP180(BMP180_I2C_ADDRESS);

    thermometer_ = (sensing::temperature::IThermometer*)imu_;

    battery_voltage_sensor_ =
        new sensing::VoltageSensor(BATTERY_VOLTAGE_SENSOR_PIN,
                                   BATTERY_VOLTAGE_SENSOR_RESOLUTION);

    motors_ = new motor::IMotor*[MOTOR_COUNT];

    motors_[FRONT_RIGHT] = new motor::BrushlessMotor(MOTOR_FRONT_RIGHT_PIN);
    motors_[FRONT_LEFT] = new motor::BrushlessMotor(MOTOR_FRONT_LEFT_PIN);
    motors_[BACK_RIGHT] = new motor::BrushlessMotor(MOTOR_BACK_RIGHT_PIN);
    motors_[BACK_LEFT] = new motor::BrushlessMotor(MOTOR_BACK_LEFT_PIN);

    display_manager_ = new display::OLEDManager();
}
