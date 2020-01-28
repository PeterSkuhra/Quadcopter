#include "Multicopter.hpp"

#include "IExecutable.hpp"
#include "DeviceProvider.hpp"
#include "control/QuadXFlightController.hpp"
#include "control/CalibrationController.hpp"

IExecutable* Multicopter::GetInstance()
{
    static Multicopter instance;
    return &instance;
}

void Multicopter::Run()
{
    Once();

    flight_controller_->Control();
    display_manager_->Run();
}

void Multicopter::Once()
{
    if (!first_launched_) {
        first_launched_ = true;

        // CODE...
        calibration_controller_->Control();
    }
}

Multicopter::Multicopter() :
    first_launched_(false)
{
    // flight_controller_ = new control::QuadXFlightController(...);
    // calibration_controller_ = new control::CalibrationController(...);
    // display_manager_ = DeviceProvider::GetInstance()->GetDisplayManager();

    DeviceProvider* device_provider = DeviceProvider::GetInstance();

    // flight_controller_ = new control::QuadXFlightController(
    //     device_provider->GetIMU(),
    //     device_provider->GetThermometer(),
    //     device_provider->GetBarometer(),
    //     device_provider->GetBatteryVoltageSensor(),
    //     device_provider->GetReceiver(),
    //     device_provider->GetSender(),
    //     device_provider->GetMotors()
    // );

    //========TEST==================
    flight_controller_ = new control::QuadXFlightController(
        device_provider->GetReceiver(),
        device_provider->GetMotors()
        );
}
