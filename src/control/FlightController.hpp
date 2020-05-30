#ifndef FLIGHT_CONTROLLER_HPP
#define FLIGHT_CONTROLLER_HPP

#include <ArduinoSTL.h>

#include "IController.hpp"

#include "command/IReceiver.hpp"
#include "sensing/imu/IIMU.hpp"
#include "sensing/voltage/ISensor.hpp"
#include "esc/ESCManager.hpp"

#include "visual/led/LED.hpp"
#include "visual/display/IDisplay.hpp"

#include "PID.hpp"
#include "ExponentialFilter.hpp"

namespace control
{

/******************************************************************************
 *  Class represents flight controller for quadcopter with X configuration.
 *  Implements IController interface.
 *
 *  Comunicates with IMU, RC, ESC modules.
 *  Filtering data with ExponentialFilter.
 *  Provides stabilization of quadcopter with cascade PID controller.
 *
 *  Show init informations on display and indicates states with LEDs.
 *****************************************************************************/
class FlightController : public IController
{
public:

    /**
     *  Constructor.
     */
    FlightController();

    /**
     *  Destructor.
     */
    ~FlightController();

    /**
     *  Initializes all important modules and performs calibrations.
     */
    void Init() override;

    /**
     *  Main control loop.
     */
    void Control() override;


private:

    /**
     *  Generic structure of flight motion data.
     */
    template <typename T>
    struct MotionData {
        T thrust;
        T yaw;
        T pitch;
        T roll;
    };

    /**
     *  Generic structure of Euler angles.
     */
    template <typename T>
    struct EulerAngles {
        T yaw;
        T pitch;
        T roll;
    };

    /**
     *  Generic structure of axes in 3D.
     */
    template <typename T>
    struct Axes {
        T x;
        T y;
        T z;
    };

    /**
     *  Generic structure of rotary motion.
     */
    template <typename T>
    struct RotaryMotion {
        EulerAngles<T> angle;
        Axes<T> angular_rate;
    };

    /**
     *  Initializes all PID controllers.
     */
    void InitPID();

    /**
     *  Initializes exponential filters.
     */
    void InitFilter();

    /**
     *  Initializes IMU.
     */
    void InitIMU();

    /**
     *  Initializes receiver.
     */
    void InitReceiver();

    /**
     *  Prints given text to USB and display.
     *
     *  @param text     given text
     *  @param clear    clear display before printing (optional, default is false)
     */
    void PrintInfo(const char* text, bool clear = false);

    /**
     *  Reads data from receiver.
     */
    void ReadReceiverData();

    /**
     *  Maps receiver data to defined ranges.
     */
    void MapReceiverData();

    /**
     *  Reads data from IMU.
     */
    void ReadIMUData();

    /**
     *  Calculations of cascade PID controllers.
     */
    void PIDCalculation();

    /**
     *  Calculations of motor speeds.
     */
    void CalculateMotorsSpeeds();

    /**
     *  Apply motors speeds.
     */
    void WriteMotorsSpeeds();


private:

    /**
     *  Initialized.
     */
    bool init_;

    /**
     *  Instance of IMU.
     */
    sensing::imu::IIMU* imu_;

    /**
     *  Instance of receiver.
     */
    command::IReceiver* receiver_;

    /**
     *  Instancec of ESCManager.
     */
    esc::ESCManager* esc_manager_;

    /**
     *  Voltage sensor for battery.
     */
    sensing::voltage::ISensor* voltage_sensor_;

    /**
     *  Vector of LEDs instances.
     */
    std::vector<visual::led::LED*> leds_;

    /**
     *  Instance of display.
     */
    visual::display::IDisplay* display_;

    /**
     *  Instances receiver filters.
     */
    MotionData<ExponentialFilter<float>* > receiver_filter_;

    /**
     *  Instances filters for angular rate data from IMU.
     */
    Axes<ExponentialFilter<float>* > angular_rate_filter_;

    /**
     *  Instance voltage sensor filter.
     */
    ExponentialFilter<float>* voltage_filter_;

    /**
     *  All gains for PID controllers.
     */
    RotaryMotion<PID::PIDData> pid_gains_;

    /**
     *  Instances of PID controllers.
     */
    RotaryMotion<PID*> pid_controller_;


    /**
     *  Structure for receiver data.
     */
    MotionData<int16_t> receiver_data_;

    /**
     *  Structure for IMU data.
     */
    RotaryMotion<float> imu_data_;

    /**
     *  Structure for outputs from PID controllers.
     */
    RotaryMotion<float> pid_data_;

    /**
     *  Vector of motors speeds.
     */
    std::vector<int16_t> motors_speeds_;

    /**
     *  Automatic stabilization and active cascade PID regulation.
     */
    bool automatic_stabilization_;

    /**
     *  Step of inner PID controllers.
     */
    uint8_t pid_out_step_;
};

}

#endif
