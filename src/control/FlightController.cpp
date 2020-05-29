#include "FlightController.hpp"

#include "../DevicesWiring.hpp"

#include "command/FlySkyReceiver.hpp"
#include "sensing/imu/IMUReader.hpp"
#include "sensing/voltage/VoltageSensor.hpp"
#include "esc/ESC30A.hpp"
#include "visual/display/OLED_128x32.hpp"


using namespace wiring;

#define MIN_PULSE_US   1000
#define MAX_PULSE_US   1982

#define MAX_PITCH_ANGLE         25  //  degrees [°]
#define MAX_ROLL_ANGLE          25  //  degrees [°]
#define MAX_YAW_AGULAR_RATE     20  //  degrees per second [°/s]


control::FlightController::FlightController() :
    init_(false),
    automatic_stabilization_(true),
    pid_out_step_(0)
{
    receiver_ = new command::FlySkyReceiver(wiring::kReceiverPins);

    voltage_sensor_ =
        new sensing::voltage::VoltageSensor(BATTERY_VOLTAGE_SENSOR_PIN,
                                            BATTERY_VOLTAGE_SENSOR_RESOLUTION);

    imu_ = new sensing::imu::IMUReader(MPU6050_INT_PIN, false, false, true);

    esc_manager_ = new esc::ESCManager(wiring::kESCPins, *voltage_sensor_);

    leds_.reserve(kLEDPins.size());
    for (uint8_t i = 0; i < kLEDPins.size(); ++i) {
        leds_.push_back(new visual::led::LED(kLEDPins.at(i)));
    }

    display_ = new visual::display::OLED_128x32(kIntroText,
                                                OLED_128X32_I2C_ADDRESS);

    motors_speeds_.clear();
    motors_speeds_.resize(MOTOR_COUNT);

    this->InitFilter();
    this->InitPID();
}

control::FlightController::~FlightController()
{
    delete voltage_sensor_;
    delete imu_;
    delete receiver_;

    delete receiver_filter_.thrust;
    delete receiver_filter_.yaw;
    delete receiver_filter_.pitch;
    delete receiver_filter_.roll;

    delete angular_rate_filter_.x;
    delete angular_rate_filter_.y;
    delete angular_rate_filter_.z;

    delete pid_controller_.angular_rate.x;
    delete pid_controller_.angular_rate.y;
    delete pid_controller_.angular_rate.z;
    delete pid_controller_.angle.roll;
    delete pid_controller_.angle.pitch;
    delete pid_controller_.angle.yaw;

    delete esc_manager_;
}

void control::FlightController::Init()
{
    if (!init_) {

        leds_.at(ORANGE)->SetOn();

        display_->Begin();

        this->InitIMU();

        this->InitReceiver();

        init_ = true;

        this->PrintInfo("All is READY!!", true);
        this->PrintInfo("Happy FLY!!!");
    }
}

void control::FlightController::Control()
{
    this->ReadReceiverData();

    this->MapReceiverData();

    this->ReadIMUData();

    this->PIDCalculation();

    this->CalculateMotorsSpeeds();

    this->WriteMotorsSpeeds();

    float voltage = voltage_filter_->Filter(voltage_sensor_->GetAnalogValue());
}

void control::FlightController::InitPID()
{
    /************************Angular rate PID gains***************************/
    /*************************************************************************/
    // Angular PID gains for X
    pid_gains_.angular_rate.x.p = 2.0;     // 1.75
    pid_gains_.angular_rate.x.i = 0;
    pid_gains_.angular_rate.x.d = 23;

    // Angular PID gains for Y
    pid_gains_.angular_rate.y.p = 2.0;
    pid_gains_.angular_rate.y.i = 0;
    pid_gains_.angular_rate.y.d = 23;

    // Angular PID gains for Z
    pid_gains_.angular_rate.z.p = 3;
    pid_gains_.angular_rate.z.i = 0.0;
    pid_gains_.angular_rate.z.d = 0.0;

    /****************************Angle PID gains******************************/
    /*************************************************************************/
    // Angle PID gains for X (Roll)
    pid_gains_.angle.roll.p = 2.0;
    pid_gains_.angle.roll.i = 0.0;
    pid_gains_.angle.roll.d = 0.0;

    // Angle PID gains for Y (Pitch)
    pid_gains_.angle.pitch.p = 2.0;
    pid_gains_.angle.pitch.i = 0.0;
    pid_gains_.angle.pitch.d = 0.0;

    // Angle PID gains for Z (Yaw)
    pid_gains_.angle.yaw.p = 0;
    pid_gains_.angle.yaw.i = 0;
    pid_gains_.angle.yaw.d = 0;
    //=========================================================================

    pid_controller_.angular_rate.x = new PID(pid_gains_.angular_rate.x.p,
                                             pid_gains_.angular_rate.x.i,
                                             pid_gains_.angular_rate.x.d,
                                             300);

    pid_controller_.angular_rate.y = new PID(pid_gains_.angular_rate.y.p,
                                             pid_gains_.angular_rate.y.i,
                                             pid_gains_.angular_rate.y.d,
                                             300);

    pid_controller_.angular_rate.z = new PID(pid_gains_.angular_rate.z.p,
                                             pid_gains_.angular_rate.z.i,
                                             pid_gains_.angular_rate.z.d,
                                             300);

    pid_controller_.angle.roll = new PID(pid_gains_.angle.roll.p,
                                         pid_gains_.angle.roll.i,
                                         pid_gains_.angle.roll.d,
                                         300, true);

    pid_controller_.angle.pitch = new PID(pid_gains_.angle.pitch.p,
                                          pid_gains_.angle.pitch.i,
                                          pid_gains_.angle.pitch.d,
                                          300);

    pid_controller_.angle.yaw = new PID(pid_gains_.angle.yaw.p,
                                        pid_gains_.angle.yaw.i,
                                        pid_gains_.angle.yaw.d,
                                        0);

    this->PrintInfo("Init PID done!");
}

void control::FlightController::InitFilter()
{
    receiver_data_.thrust = receiver_->ReadChannel(1);
    receiver_data_.yaw    = receiver_->ReadChannel(2);
    receiver_data_.pitch  = receiver_->ReadChannel(3);
    receiver_data_.roll   = receiver_->ReadChannel(4);

    const uint8_t kFilterWeight = 10;

    receiver_filter_.thrust = new ExponentialFilter<float>(kFilterWeight, receiver_data_.thrust);
    receiver_filter_.yaw    = new ExponentialFilter<float>(kFilterWeight, receiver_data_.yaw);
    receiver_filter_.pitch  = new ExponentialFilter<float>(kFilterWeight, receiver_data_.pitch);
    receiver_filter_.roll   = new ExponentialFilter<float>(kFilterWeight, receiver_data_.roll);


    voltage_filter_ = new ExponentialFilter<float>(20, voltage_sensor_->GetAnalogValue());

    // for Cascade PID
    angular_rate_filter_.x = new ExponentialFilter<float>(20, 0);
    angular_rate_filter_.y = new ExponentialFilter<float>(20, 0);
    angular_rate_filter_.z = new ExponentialFilter<float>(20, 0);

    PrintInfo("Init filter done!");
}

void control::FlightController::InitIMU()
{
    imu_->Begin();

    this->PrintInfo("Calibrating IMU...", true);

    if (imu_->Calibrate()) {
        this->PrintInfo("Calib. IMU OK!");
    }
    else {
        this->PrintInfo("Calib. IMU NOK!", true);
        leds_.at(RED)->SetOn();
    }
    delay(100);
}

void control::FlightController::InitReceiver()
{
    repeat_calibration:
    PrintInfo("Calib. receiver...");

    if (receiver_->Calibrate()) {
        PrintInfo("Calib. receiver OK!");
    }
    else {
        PrintInfo("Calib. receiver NOK!", true);

        while(true) {
            leds_.at(RED)->SetOn();

            if (receiver_->ReadChannel(1) > 1900) {
                leds_.at(RED)->SetOff();
                goto repeat_calibration;
            }
        }
    }

    leds_.at(ORANGE)->SetOff();
    leds_.at(GREEN)->SetOn();
    delay(1000);


    leds_.at(ORANGE)->SetBlinkInterval(100, 100);
    leds_.at(ORANGE)->StartBlink();

    this->PrintInfo("Thrust stick UP!");

    while (receiver_->ReadChannel(1) < 1900) {
        leds_.at(ORANGE)->Update();
    }
    leds_.at(ORANGE)->SetOff();

    leds_.at(GREEN)->SetBlinkInterval(500, 300);
    leds_.at(GREEN)->StartBlink();

    this->PrintInfo("Thrust stick DOWN!");

    while (receiver_->ReadChannel(1) > 1050) {
        leds_.at(GREEN)->Update();
    }
    leds_.at(GREEN)->SetOn();
}

void control::FlightController::ReadReceiverData()
{
    receiver_data_.thrust = receiver_filter_.thrust->Filter(receiver_->ReadChannel(1));
    receiver_data_.yaw    = receiver_filter_.yaw->Filter(receiver_->ReadChannel(2));
    receiver_data_.pitch  = receiver_filter_.pitch->Filter(receiver_->ReadChannel(3));
    receiver_data_.roll   = receiver_filter_.roll->Filter(receiver_->ReadChannel(4));
}

void control::FlightController::MapReceiverData()
{
    receiver_data_.thrust = constrain(receiver_data_.thrust, 1000, 1800);

    receiver_data_.yaw = map(receiver_data_.yaw,
                             MIN_PULSE_US,
                             MAX_PULSE_US,
                             -MAX_YAW_AGULAR_RATE,
                             MAX_YAW_AGULAR_RATE);

    receiver_data_.pitch = map(receiver_data_.pitch,
                               MIN_PULSE_US,
                               MAX_PULSE_US,
                               -MAX_PITCH_ANGLE,
                               MAX_PITCH_ANGLE);

    receiver_data_.roll   = map(receiver_data_.roll,
                                MIN_PULSE_US,
                                MAX_PULSE_US,
                                -MAX_ROLL_ANGLE,
                                MAX_ROLL_ANGLE);
}

void control::FlightController::PrintInfo(const char* text, bool clear = false)
{
    if (clear) {
        display_->Clear();
    }

    Serial.println(text);
    display_->Print(text);
}

void control::FlightController::ReadIMUData()
{
    if (receiver_->ReadChannel(5) > 1800) {

        imu_->Update();

        imu_data_.angular_rate.x = imu_->GetXAngularRate();
        imu_data_.angular_rate.y = imu_->GetYAngularRate();
        imu_data_.angular_rate.z = imu_->GetZAngularRate();

        imu_data_.angle.yaw = imu_->GetYawAngle();  // netreba
        imu_data_.angle.pitch = imu_->GetPitchAngle();
        imu_data_.angle.roll = imu_->GetRollAngle();


        imu_data_.angular_rate.x = angular_rate_filter_.x->Filter(imu_data_.angular_rate.x);
        imu_data_.angular_rate.y = angular_rate_filter_.y->Filter(imu_data_.angular_rate.y);
        imu_data_.angular_rate.z = angular_rate_filter_.z->Filter(imu_data_.angular_rate.z);
    }
    else {
        delay(3);

        imu_data_.angular_rate.x = 0;
        imu_data_.angular_rate.y = 0;
        imu_data_.angular_rate.z = 0;

        imu_data_.angle.yaw = 0;
        imu_data_.angle.pitch = 0;
        imu_data_.angle.roll = 0;
    }
}

void control::FlightController::PIDCalculation()
{
    if (receiver_->ReadChannel(1) > 1050) {

        if (automatic_stabilization_) {

            if (pid_out_step_ >= 4) {
                pid_out_step_ = 0;

                // Outer PID loop
                pid_data_.angle.roll = pid_controller_.angle.roll->Update(
                    receiver_data_.roll,
                    imu_data_.angle.roll);

                pid_data_.angle.pitch = pid_controller_.angle.pitch->Update(
                    receiver_data_.pitch,
                    imu_data_.angle.pitch);
            }

            //Inner PID loop
            pid_data_.angular_rate.x = pid_controller_.angular_rate.x->Update(
                pid_data_.angle.roll,
                imu_data_.angular_rate.x);

            pid_data_.angular_rate.y = pid_controller_.angular_rate.y->Update(
                pid_data_.angle.pitch,
                imu_data_.angular_rate.y);

            pid_out_step_++;


            // Yaw only angular rate PID
            pid_data_.angular_rate.z = pid_controller_.angular_rate.z->Update(
                receiver_data_.yaw,
                imu_data_.angular_rate.z);
            // pid_data_.angular_rate.y = -pid_data_.angular_rate.y;
            pid_data_.angular_rate.z = -pid_data_.angular_rate.z;   // TODO in PID class!!!!!!!!!!!!!!!!!!!
        }
        else {
            pid_data_.angular_rate.x = pid_controller_.angular_rate.x->Update(
                receiver_data_.roll,            // setpoint
                imu_data_.angular_rate.x);      // process

            pid_data_.angular_rate.y = pid_controller_.angular_rate.y->Update(
                receiver_data_.pitch,
                imu_data_.angular_rate.y);

            pid_data_.angular_rate.z = pid_controller_.angular_rate.z->Update(
                receiver_data_.yaw,
                imu_data_.angular_rate.z);

            // pid_data_.angular_rate.y = -pid_data_.angular_rate.y;
            pid_data_.angular_rate.z = -pid_data_.angular_rate.z;   // TODO in PID class!!!!!!!!!!!!!!!!!!!
        }
    }
    else {
        pid_data_.angular_rate.x = 0;
        pid_data_.angular_rate.y = 0;
        pid_data_.angular_rate.z = 0;

        pid_data_.angle.roll = 0;
        pid_data_.angle.pitch = 0;
        pid_data_.angle.yaw = 0;
    }
}

void control::FlightController::CalculateMotorsSpeeds()
{
    // Front right
    motors_speeds_[FRONT_RIGHT] =
        receiver_data_.thrust +
        pid_data_.angular_rate.z +
        pid_data_.angular_rate.y +
        pid_data_.angular_rate.x;

    // Front left
    motors_speeds_[FRONT_LEFT] =
        receiver_data_.thrust -
        pid_data_.angular_rate.z +
        pid_data_.angular_rate.y -
        pid_data_.angular_rate.x;

    // Back right
    motors_speeds_[BACK_RIGHT] =
        receiver_data_.thrust -
        pid_data_.angular_rate.z -
        pid_data_.angular_rate.y +
        pid_data_.angular_rate.x;

    // Back left
    motors_speeds_[BACK_LEFT] =
        receiver_data_.thrust +
        pid_data_.angular_rate.z -
        pid_data_.angular_rate.y -
        pid_data_.angular_rate.x;
}

void control::FlightController::WriteMotorsSpeeds()
{
    esc_manager_->SetSpeeds(motors_speeds_);
}
