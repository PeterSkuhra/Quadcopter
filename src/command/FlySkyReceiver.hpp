#ifndef FLYSKY_RECEIVER_HPP
#define FLYSKY_RECEIVER_HPP

#include <Arduino.h>
#include <ArduinoSTL.h>

#include "IReceiver.hpp"
#include "ICalibratable.hpp"
#include "PWMPinListener.hpp"


namespace command
{

#define CHANNEL_COUNT   10
#define MIN_VALUE       1000
#define MIDDLE_VALUE    1500
#define MAX_VALUE       2000

#define CALIBRATABLE_CHANNEL_COUNT  4


class FlySkyReceiver : public IReceiver
{
 public:
    FlySkyReceiver(const std::vector<uint8_t> &pins);

    ~FlySkyReceiver();

    uint16_t ReadChannel(uint8_t channel_number) const override;

    bool Calibrate() override;

    bool IsCalibrated() const override;


private:
    bool IsReadyToCalibrate();

 private:
    const uint8_t channel_count_;

    PWMPinListener** pwm_pin_listeners_;

    int16_t* channels_offsets_;

    bool is_calibrated_;
};

}

#endif
