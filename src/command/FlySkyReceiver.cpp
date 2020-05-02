#include "FlySkyReceiver.hpp"


command::FlySkyReceiver::FlySkyReceiver(const std::vector<uint8_t> &pins) :
    channel_count_(pins.size()),
    is_calibrated_(false)
{
    pwm_pin_listeners_ = new PWMPinListener*[channel_count_];

    for (uint8_t i = 0; i < channel_count_; ++i) {
        pwm_pin_listeners_[i] = new PWMPinListener(pins[i]);
    }

    channels_offsets_ = new int16_t[channel_count_];
    for (uint8_t i = 0; i < channel_count_; ++i) {
        channels_offsets_[i] = 0;
    }
}

command::FlySkyReceiver::~FlySkyReceiver()
{
    for (uint8_t i = 0; i < channel_count_; ++i) {
        delete [] pwm_pin_listeners_[i];
    }
    delete [] pwm_pin_listeners_;

    delete [] channels_offsets_;
}

uint16_t command::FlySkyReceiver::ReadChannel(uint8_t channel_number) const
{
    if ((channel_number <= channel_count_) && !(channel_number <= 0)) {
        uint16_t channel_value =
            pwm_pin_listeners_[channel_number - 1]->ReadChannel() +
                channels_offsets_[channel_number - 1];

        channel_value = constrain(channel_value, MIN_VALUE, MAX_VALUE);

        return channel_value;
    }

    return 0;
}

bool command::FlySkyReceiver::Calibrate()
{
    delay(1000);
    if (!IsReadyToCalibrate()) {
        return false;
    }
    delay(1000);

    const uint16_t kSamples = 1000;

    for (uint8_t i = 0; i < CALIBRATABLE_CHANNEL_COUNT; ++i) {
        int32_t sum = 0;

        for (uint16_t j = 0; j < kSamples; ++j) {
            sum += pwm_pin_listeners_[i]->ReadChannel();
        }
        if (i == 0) {   // Thrust
            channels_offsets_[i] = MIN_VALUE - (sum / kSamples);
        }
        else if ((i >= 1) && (i <= 3)) {    // Yaw, Pitch, Roll
            channels_offsets_[i] = MIDDLE_VALUE - (sum / kSamples);
        }
    }
    is_calibrated_ = true;

    return true;
}

bool command::FlySkyReceiver::IsCalibrated() const
{
    return is_calibrated_;
}

bool command::FlySkyReceiver::IsReadyToCalibrate()
{
    const uint8_t kTolerance = 50;
    const uint32_t kMaxTime = 10000;

    uint8_t all_done = 0b0000;

    uint32_t elapsed_time;
    uint32_t start_time = millis();

    while (all_done != 0b1111) {
        // Thrust
        if ((pwm_pin_listeners_[0]->ReadChannel() < (MIN_VALUE + kTolerance)) &&
            (pwm_pin_listeners_[0]->ReadChannel() > (MIN_VALUE - kTolerance))) {
            all_done |= 1;
        }
        else {
            all_done &= ~1;
        }

        // Yaw, Pitch, Roll
        for (uint8_t i = 1; i <= 3; ++i) {
            if ((pwm_pin_listeners_[i]->ReadChannel() < (MIDDLE_VALUE + kTolerance)) &&
                (pwm_pin_listeners_[i]->ReadChannel() > (MIDDLE_VALUE - kTolerance))) {
                all_done |= (1 << i);
            }
            else {
                all_done &= ~(1 << i);
            }
        }

        elapsed_time = millis() - start_time;
        if (elapsed_time >= kMaxTime) {
            return false;
        }
    }

    return true;
}
