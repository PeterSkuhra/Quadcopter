#ifndef ICALIBRATABLE_HPP
#define ICALIBRATABLE_HPP

/******************************************************************************
 *  Interface calibratable devices. Provides calibration methods
 *  and checks the success of the calibration.
 *****************************************************************************/
class ICalibratable
{
 public:

     /**
      * Default destructor.
      */
     virtual ~ICalibratable() = default;

     /**
      * Calibrates device and returns true if successful.
      *
      * @return true if successful
      */
     virtual bool Calibrate() = 0;

     /**
      * Checks the success of the calibration.
      *
      * @return true if device is calibrated
      */
     virtual bool IsCalibrated() const = 0;
};

#endif
