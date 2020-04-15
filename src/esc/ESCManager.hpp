#ifndef ESC_MANAGER_HPP
#define ESC_MANAGER_HPP

#include <Arduino.h>
#include <ArduinoSTL.h>

#include "ICalibratable.hpp"
#include "ESC30A.hpp"
#include "../sensing/voltage/ISensor.hpp"


namespace esc
{

/******************************************************************************
 *  The class represents the manager of several ESC's.
 *  The number of managing ESC's depends on number of elements in the vector.
 *
 *  
 *
 *  Class implements "ICalibratable" interface.
 *  So, you can calibrate all ESC's at the same time.
 *****************************************************************************/
class ESCManager : ICalibratable
{
public:

    /**
     *  Constructor.
     *
     *  @param esc_pins         vector of pins for esc
     *  @param voltage_sensor   reference of voltage sensor
     */
    ESCManager(const std::vector<uint8_t> &esc_pins,
               const sensing::voltage::ISensor &voltage_sensor);

    /**
     *  Destructor.
     */
    ~ESCManager();

    /**
     *  Applies to all motors.
     *  The number and order of elements in the speed vector MUST be the same
     *  as the number and order of elements in the pin vector.
     *
     *  @param speeds   vector od speeds for motors
     */
    void SetSpeeds(std::vector<int16_t> &speeds);

    /**
     *  Calibrates all ESC's.
     *
     *  @return true when all done
     */
    bool Calibrate() override;

    /**
     *  Returns true if all ESC's is calibrated.
     *
     *  @return true if all ESC's is calibrated
     */
    bool IsCalibrated() const override;


private:

    /**
     *  Calibration state.
     *  true = calibrated
     */
    bool is_calibrated_;

    /**
     *  Number of motors.
     */
    uint8_t motor_count_;

    /**
     *  Instances of motors.
     */
    esc::IESC** motors_;

    /**
     *  Pointer of voltage sensor.
     */
    sensing::voltage::ISensor* voltage_sensor_;
};

}

#endif
