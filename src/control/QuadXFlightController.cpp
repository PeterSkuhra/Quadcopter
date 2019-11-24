#include "QuadXFlightController.hpp"

using namespace control;


QuadXFlightController::QuadXFlightController(
    sensing::imu::IIMU* imu,
    sensing::ISensor* battery_voltage_sensor,
    command::IReceiver* receiver,
    display::ISender* sender,
    motor::IMotor** motors)
{
    QuadXFlightController(imu,
                          nullptr,
                          nullptr,
                          battery_voltage_sensor,
                          receiver,
                          sender,
                          motors);
}

QuadXFlightController::QuadXFlightController(
    sensing::imu::IIMU* imu,
    sensing::temperature::IThermometer* thermometer,
    sensing::ISensor* battery_voltage_sensor,
    command::IReceiver* receiver,
    display::ISender* sender,
    motor::IMotor** motors)
{
    QuadXFlightController(imu,
                          thermometer,
                          nullptr,
                          battery_voltage_sensor,
                          receiver,
                          sender,
                          motors);
}

QuadXFlightController::QuadXFlightController(
    sensing::imu::IIMU* imu,
    sensing::temperature::IThermometer* thermometer,
    sensing::pressure::IBarometer* barometer,
    sensing::ISensor* battery_voltage_sensor,
    command::IReceiver* receiver,
    display::ISender* sender,
    motor::IMotor** motors) :

    imu_(imu),
    thermometer_(thermometer),
    barometer_(barometer),
    battery_voltage_sensor_(battery_voltage_sensor),
    receiver_(receiver),
    sender_(sender),
    motors_(motors)
{
    imu_->Begin();

    if (barometer_ != nullptr) {
        barometer_->Begin();
    }
}

QuadXFlightController::~QuadXFlightController()
{

}

void QuadXFlightController::Control()
{
    imu_->Update();
    barometer_->Update();
}
