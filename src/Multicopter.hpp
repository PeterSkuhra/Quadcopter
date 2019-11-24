#ifndef MULTICOPTER_HPP
#define MULTICOPTER_HPP

#include "IExecutable.hpp"
#include "control/IController.hpp"
#include "display/IDisplayManager.hpp"

#include "sensing/imu/MPU6050.hpp"
#include "command/IReceiver.hpp"
#include "display/ISender.hpp"

class Multicopter : public IExecutable
{
 public:

    static IExecutable* GetInstance();

    void Run() override;


 private:

    Multicopter();

    void Once();

 private:

    bool first_launched_;

    control::IController* flight_controller_;
    control::IController* calibration_controller_;
    display::IDisplayManager* display_manager_;
};

#endif
