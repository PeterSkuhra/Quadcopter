#include "ESCManager.hpp"
#include "DevicesWiring.hpp"


esc::ESCManager::ESCManager(const std::vector<uint8_t> &esc_pins,
                            const sensing::voltage::ISensor &voltage_sensor) :
    motor_count_(esc_pins.size())
{
    voltage_sensor_ = &voltage_sensor;

    motors_ = new esc::IESC*[motor_count_];

    for (uint8_t i = 0; i < motor_count_; ++i) {
        motors_[i] = new esc::ESC30A(esc_pins[i]);
    }
}

esc::ESCManager::~ESCManager()
{
    for (uint8_t i = 0; i < motor_count_; ++i) {
        delete [] motors_[i];
    }
    delete [] motors_;
}

void esc::ESCManager::SetSpeeds(std::vector<int16_t> &speeds)
{
    if (speeds.size() != motor_count_) {
        return;
    }

    for (uint8_t i = 0; i < motor_count_; ++i) {
        motors_[i]->SetSpeed(speeds[i]);
        // motors_[i]->SetSpeed(speeds[i], voltage_sensor_->GetAnalogValue());
    }
}

bool esc::ESCManager::Calibrate()
{
    is_calibrated_ = false;

    for (uint8_t i = 0; i < motor_count_; ++i) {
        motors_[i]->SetSpeed(MAX_PULSE);
    }
    delay(2200);

    for (uint8_t i = 0; i < motor_count_; ++i) {
        motors_[i]->SetSpeed(MIN_PULSE);
    }
    delay(300);

    is_calibrated_ = true;
    return is_calibrated_;
}

bool esc::ESCManager::IsCalibrated() const
{
    return is_calibrated_;
}
