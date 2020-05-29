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

    void InitPID();
    void InitFilter();
    void InitIMU();
    void InitReceiver();

    void PrintInfo(const char* text, bool clear = false);

    void ReadReceiverData();
    void MapReceiverData();
    void ReadIMUData();
    void PIDCalculation();
    void CalculateMotorsSpeeds();
    void WriteMotorsSpeeds();

private:

    bool init_;

    sensing::imu::IIMU* imu_;
    command::IReceiver* receiver_;
    esc::ESCManager* esc_manager_;
    sensing::voltage::ISensor* voltage_sensor_;
    std::vector<visual::led::LED*> leds_;
    visual::display::IDisplay* display_;


    MotionData<ExponentialFilter<float>* > receiver_filter_;
    Axes<ExponentialFilter<float>* > angular_rate_filter_;
    ExponentialFilter<float>* voltage_filter_;


    RotaryMotion<PID::PIDData> pid_gains_;
    RotaryMotion<PID*> pid_controller_;


    MotionData<int16_t> receiver_data_;
    RotaryMotion<float> imu_data_;
    RotaryMotion<float> pid_data_;
    std::vector<int16_t> motors_speeds_;

    bool automatic_stabilization_;
    uint8_t pid_out_step_;

};

}

#endif
