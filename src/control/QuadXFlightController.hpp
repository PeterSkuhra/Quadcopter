#ifndef QUADX_FLIGHT_CONTROLLER_HPP
#define QUADX_FLIGHT_CONTROLLER_HPP

#include "IController.hpp"

#include "DeviceProvider.hpp"


namespace control
{

class QuadXFlightController : public IController
{
public:

    QuadXFlightController(sensing::imu::IIMU* imu,
                          sensing::ISensor* battery_voltage_sensor,
                          command::IReceiver* receiver,
                          display::ISender* sender,
                          motor::IMotor** motors);

    QuadXFlightController(sensing::imu::IIMU* imu,
                          sensing::temperature::IThermometer* thermometer,
                          sensing::ISensor* battery_voltage_sensor,
                          command::IReceiver* receiver,
                          display::ISender* sender,
                          motor::IMotor** motors);

    QuadXFlightController(sensing::imu::IIMU* imu,
                          sensing::temperature::IThermometer* thermometer,
                          sensing::pressure::IBarometer* barometer,
                          sensing::ISensor* battery_voltage_sensor,
                          command::IReceiver* receiver,
                          display::ISender* sender,
                          motor::IMotor** motors);

    ~QuadXFlightController();   // TODO!!!

    void Control() override;


private:

    // void UpdateChannels();

private:

    sensing::imu::IIMU* imu_;
    sensing::temperature::IThermometer* thermometer_;
    sensing::pressure::IBarometer* barometer_;
    sensing::ISensor* battery_voltage_sensor_;

    command::IReceiver* receiver_;

    display::ISender* sender_;

    motor::IMotor** motors_;

};

}

#endif
