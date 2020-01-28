#include "QuadXFlightController.hpp"

using namespace control;

QuadXFlightController::QuadXFlightController(
    command::IReceiver* receiver,
    motor::IMotor** motors)
{
    QuadXFlightController(nullptr,
                          nullptr,
                          nullptr,
                          nullptr,
                          receiver,
                          nullptr,
                          motors);
}


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


    //========TEST=============================================================
    UpdateChannels();
    CalculateMotorsSpeeds();
    WriteMotorsSpeeds();
    //=========================================================================
}

void QuadXFlightController::UpdateChannels()
{
    receiver_data_.thrust = receiver_->ReadChannel(1);
    receiver_data_.roll = receiver_->ReadChannel(4);
    receiver_data_.pitch = receiver_->ReadChannel(3);
    receiver_data_.yaw = receiver_->ReadChannel(2);
}

void QuadXFlightController::CalculateMotorsSpeeds()
{
    // Front right
    motors_speeds_[0] = receiver_data_.thrust + receiver_data_.yaw +
                       receiver_data_.pitch + receiver_data_.roll;
    // Front left
    motors_speeds_[1] = receiver_data_.thrust - receiver_data_.yaw +
                       receiver_data_.pitch - receiver_data_.roll;

    // Back right
    motors_speeds_[2] = receiver_data_.thrust - receiver_data_.yaw -
                       receiver_data_.pitch + receiver_data_.roll;

    // Back left
    motors_speeds_[3] = receiver_data_.thrust + receiver_data_.yaw -
                       receiver_data_.pitch - receiver_data_.roll;
}

void QuadXFlightController::WriteMotorsSpeeds()
{
    for (uint8_t i = 0; i < 4; ++i) {
        motors_[i]->SetSpeed(motors_speeds_[i]);
    }
}
