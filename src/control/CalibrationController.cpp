#include "CalibrationController.hpp"

using namespace control;


void CalibrationController::Control()
{
    for (int i = 0; i < devices_.size(); ++i) {
        devices_[i]->Calibrate();
    }
}

CalibrationController::CalibrationController()
{

}

void CalibrationController::AddDevice(ICalibratable* device)
{
    devices_.push_back(device);
}
