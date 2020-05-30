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

/******************************************************************************
 *  This class represents FlySky receiver for remote control.
 *  Implements IReceiver interface.
 *
 *  You can calibrate four main channels to calculate offsets.
 *****************************************************************************/
class FlySkyReceiver : public IReceiver
{
 public:

     /**
      * Constructor.
      * It is important to keep the correct pins order.
      *
      * @param pins vector of connected pins to receiver
      */
     FlySkyReceiver(const std::vector<uint8_t> &pins);

     /**
      * Destructor.
      */
     ~FlySkyReceiver();

     /**
      * Returns the pulse length of the PWM signal for specific channel.
      *
      * @param channel_number   channel number to read
      *
      * @return the pulse length of the PWM signal for specific channel
      */
     uint16_t ReadChannel(uint8_t channel_number) const override;

     /**
      * Calibrates receiver and calculate offsets for four important channels.
      *
      * @return true if the calibration was successful
      */
     bool Calibrate() override;

     /**
      * Returns true if receiver is calibrated.
      *
      * @return true if receiver is calibrated
      */
     bool IsCalibrated() const override;


 private:

     /**
      * Checks the readiness of the receiver for calibration.
      *
      * @return true if receiver is ready to calibrate
      */
     bool IsReadyToCalibrate();

 private:

     /**
      * All channel count.
      */
     const uint8_t channel_count_;

     /**
      * Instances of PWMPinListener.
      */
     PWMPinListener** pwm_pin_listeners_;

     /**
      * Offsets for important channels.
      */
     int16_t* channels_offsets_;

     /**
      * Calibration success.
      */
     bool is_calibrated_;
};

}

#endif
