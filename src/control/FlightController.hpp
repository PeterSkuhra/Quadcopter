#ifndef FLIGHT_CONTROLLER_HPP
#define FLIGHT_CONTROLLER_HPP

#include <ArduinoSTL.h>

#include "IController.hpp"

#include "command/IReceiver.hpp"
#include "sensing/imu/IIMU.hpp"
#include "sensing/voltage/ISensor.hpp"
#include "esc/ESCManager.hpp"

#include "PID.hpp"
#include "ExponentialFilter.hpp"

namespace control
{

class FlightController : public IController
{
public:

    FlightController();

    ~FlightController();

    void Init() override;

    void Control() override;


private:

    template <typename T>
    struct MotionData {
        T thrust;
        T yaw;
        T pitch;
        T roll;
    };

    template <typename T>
    struct EulerAngles {
        T yaw;
        T pitch;
        T roll;
    };

    template <typename T>
    struct Axes {
        T x;
        T y;
        T z;
    };

    template <typename T>
    struct RotaryMotion {
        EulerAngles<T> angle;
        Axes<T> angular_rate;
    };



    void InitFilter();
    void InitPID();
    void ReadReceiverData();
    void MapReceiverData();
    void ReadIMUData();
    void PIDCalculation();
    void CalculateMotorsSpeeds();
    void WriteMotorsSpeeds();

private:

    sensing::voltage::ISensor* voltage_sensor_;

    sensing::imu::IIMU* imu_;
    command::IReceiver* receiver_;

    MotionData<ExponentialFilter<float>* > filter_;

    ExponentialFilter<float>* yaw_filter_;
    ExponentialFilter<float>* voltage_filter_;

    MotionData<int16_t> receiver_data_;
    //MotionData<float> imu_data_;
    // MotionData<float> pid_data_;
    // MotionData<PID*> pid_controller_;

    esc::ESCManager* esc_manager_;
    std::vector<int16_t> motors_speeds_;

    bool init_;


    // For Cascade PID
    RotaryMotion<float> imu_data_;
    RotaryMotion<float> pid_data_;
    RotaryMotion<PID*> pid_controller_;
    RotaryMotion<PID::PIDData> pid_gains_;

    Axes<ExponentialFilter<float>* > angular_rate_filter_;

    /// end for Cascade PID
};

}

#endif
