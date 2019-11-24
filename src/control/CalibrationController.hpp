#ifndef CALIBRATION_CONTROLLER_HPP
#define CALIBRATION_CONTROLLER_HPP

#include <ArduinoSTL.h>

#include "IController.hpp"
#include "../ICalibratable.hpp"

#include "display/IDisplayManager.hpp"

namespace control
{

class CalibrationController : public IController
{
public:

    CalibrationController();
    CalibrationController(ICalibratable* device);
    CalibrationController(display::IDisplayManager* display_manager);
    CalibrationController(ICalibratable* device,
                          display::IDisplayManager* display_manager);

    void Control() override;

    void AddDevice(ICalibratable* device);


private:

    std::vector<ICalibratable*> devices_;

    display::IDisplayManager* display_manager_;

};

}

#endif
