#include "FlightController.hpp"

#include "../DevicesWiring.hpp"

#include "command/FlySkyReceiver.hpp"
#include "sensing/imu/IMUReader.hpp"
#include "sensing/voltage/VoltageSensor.hpp"
#include "esc/ESC30A.hpp"


using namespace wiring;

#define MIN_PULSE_US   1000
#define MAX_PULSE_US   1982

#define MAX_PITCH_ANGLE         25  //  degrees [°]
#define MAX_ROLL_ANGLE          25  //  degrees [°]
#define MAX_YAW_AGULAR_RATE     20  //  degrees per second [°/s]


control::FlightController::FlightController() :
    init_(false)
{
    receiver_ = new command::FlySkyReceiver(wiring::kReceiverPins);

    voltage_sensor_ =
        new sensing::voltage::VoltageSensor(BATTERY_VOLTAGE_SENSOR_PIN,
                                            BATTERY_VOLTAGE_SENSOR_RESOLUTION);

    imu_ = new sensing::imu::IMUReader(MPU6050_INT_PIN, false, true, true);

    esc_manager_ = new esc::ESCManager(wiring::kESCPins, *voltage_sensor_);

    motors_speeds_.clear();
    motors_speeds_.resize(MOTOR_COUNT);

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

        pinMode(LED_ORANGE_PIN, OUTPUT);
        pinMode(LED_GREEN_PIN, OUTPUT);
        pinMode(LED_RED_PIN, OUTPUT);
        digitalWrite(LED_ORANGE_PIN, HIGH);

        // Ak je pripojeny ovladac a je SWA paka hore, tak cakaj na kalibraciu
        // while (receiver_->ReadChannel(5) > 1200) {

        // delay(1000);
        // Serial.println(F("\nWaiting..."));
        // // while (!(receiver_->ReadChannel(1) > 1800));
        //
        // Serial.println(F("\nCalibrating all ESC..."));
        // esc_manager_->Calibrate();
        // Serial.println(F("\nCalibration all ESC OK :)"));

        // Serial.println("CH5: " + String(receiver_->ReadChannel(5)));
        // Serial.println("CH6: " + String(receiver_->ReadChannel(6)));
        //
        //     // Ak bude SWB paka dole, tak kalibruj
        //     if ((receiver_->ReadChannel(6) < 1200) &&
        //         (receiver_->ReadChannel(5) > 1800)) {
        //         Serial.println(F("\nCalibrating all ESC..."));
        //
        //         if (esc_manager_->Calibrate()) {
        //             Serial.println(F("\nCalibration all ESC OK :)"));
        //             // break;
        //         }
        //         else {
        //             Serial.println(F("\nCalibration ESC NOK :("));
        //             while (true);
        //         }
        //     }
        // }

        // ESC's
        // delay(500);
        // if (receiver_->ReadChannel(10) < 1200 && receiver_->ReadChannel(10) > 900) {
        //     Serial.println(F("\nCalibrating all ESC..."));
        //
        //     if (esc_manager_->Calibrate()) {
        //         Serial.println(F("\nCalibration all ESC OK :)"));
        //     }
        //     else {
        //         Serial.println(F("\nCalibration ESC NOK :("));
        //     }
        // }

        // while (receiver_->ReadChannel(5) < 1200) {
        //     delay(10);
        //     Serial.println("CH5: " + String(receiver_->ReadChannel(5)));
        // }

        // IMU
        imu_->Begin();

        Serial.println(F("\nCalibrating imu..."));
        if (imu_->Calibrate()) {
            Serial.println(F("\nCalibration imu OK :)"));
        }
        else {
            Serial.println(F("\nCalibration imu NOK :("));
            digitalWrite(LED_RED_PIN, HIGH);
            //while(true);
        }
        delay(100);

        // Receiver
        repeat_calibration:
        Serial.println(F("\nCalibrating receiver..."));
        if (receiver_->Calibrate()) {
            Serial.println(F("\nCalibration rec OK :)"));
        }
        else {
            Serial.println(F("\nCalibration rec NOK :("));
            while(true) {
                digitalWrite(LED_RED_PIN, HIGH);

                if (receiver_->ReadChannel(1) > 1900) {
                    digitalWrite(LED_RED_PIN, LOW);
                    goto repeat_calibration;
                }
            }
        }


        this->InitFilter();

        digitalWrite(LED_ORANGE_PIN, LOW);
        digitalWrite(LED_GREEN_PIN, HIGH);

        init_ = true;

        while (receiver_->ReadChannel(1) < 1900) {
            digitalWrite(LED_ORANGE_PIN, !digitalRead(LED_ORANGE_PIN));
            delay(100);
        }
        digitalWrite(LED_ORANGE_PIN, LOW);

        while (receiver_->ReadChannel(1) > 1050) {
            digitalWrite(LED_GREEN_PIN, !digitalRead(LED_GREEN_PIN));
            delay(200);
        }
        digitalWrite(LED_GREEN_PIN, HIGH);
    }
}

void control::FlightController::InitPID()
{
    /************************Angular rate PID gains***************************/
    /*************************************************************************/
    // Angular PID gains for X
    pid_gains_.angular_rate.x.p = 0;
    pid_gains_.angular_rate.x.i = 0;
    pid_gains_.angular_rate.x.d = 5;

    // Angular PID gains for Y
    pid_gains_.angular_rate.y.p = 0;
    pid_gains_.angular_rate.y.i = 0;
    pid_gains_.angular_rate.y.d = 5;

    // Angular PID gains for Z
    pid_gains_.angular_rate.z.p = 3;
    pid_gains_.angular_rate.z.i = 0.02;
    pid_gains_.angular_rate.z.d = 0;

    /****************************Angle PID gains******************************/
    /*************************************************************************/
    // Angle PID gains for X (Roll)
    pid_gains_.angle.roll.p = 0;
    pid_gains_.angle.roll.i = 0;
    pid_gains_.angle.roll.d = 0;

    // Angle PID gains for Y (Pitch)
    pid_gains_.angle.pitch.p = 0;
    pid_gains_.angle.pitch.i = 0;
    pid_gains_.angle.pitch.d = 0;

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
                                         0);

    pid_controller_.angle.pitch = new PID(pid_gains_.angle.pitch.p,
                                          pid_gains_.angle.pitch.i,
                                          pid_gains_.angle.pitch.d,
                                          0);

    pid_controller_.angle.yaw = new PID(pid_gains_.angle.yaw.p,
                                        pid_gains_.angle.yaw.i,
                                        pid_gains_.angle.yaw.d,
                                        0);
}
static uint32_t previous = 0;
static uint32_t current = 0;
static uint32_t elapsed = 0;
static uint32_t loop_time = 0;
// static uint32_t step = 0;

void control::FlightController::Control()
{
    loop_time = micros();

    this->ReadReceiverData();

    this->MapReceiverData();

    this->ReadIMUData();

    this->PIDCalculation();

    this->CalculateMotorsSpeeds();

    this->WriteMotorsSpeeds();

    float voltage = voltage_filter_->Filter(voltage_sensor_->GetAnalogValue());
    // Serial.println("Volt: " + String(voltage));

    while ((micros() - loop_time) < 10000);
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
    angular_rate_filter_.x = new ExponentialFilter<float>(10, 0);
    angular_rate_filter_.y = new ExponentialFilter<float>(10, 0);
    angular_rate_filter_.z = new ExponentialFilter<float>(10, 0);
}

void control::FlightController::ReadReceiverData()
{
    receiver_data_.thrust = receiver_filter_.thrust->Filter(receiver_->ReadChannel(1));
    receiver_data_.yaw    = receiver_filter_.yaw->Filter(receiver_->ReadChannel(2));
    receiver_data_.pitch  = receiver_filter_.pitch->Filter(receiver_->ReadChannel(3));
    receiver_data_.roll   = receiver_filter_.roll->Filter(receiver_->ReadChannel(4));

    // Serial.print("Thr: ");
    // Serial.print(receiver_data_.thrust);
    // Serial.print("  Yaw: ");
    // Serial.print(receiver_data_.yaw);
    // Serial.print("  Pit: ");
    // Serial.print(receiver_data_.pitch);
    // Serial.print("  Rol: ");
    // Serial.println(receiver_data_.roll);

    // for (uint8_t i = 1; i <= 10; ++i) {
    //     Serial.print("CH" + String(i) + ": " + String(receiver_->ReadChannel(i)) + "  ");
    // }
    // Serial.println();
}

void control::FlightController::MapReceiverData()
{
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

    // Serial.println(String(receiver_data_.yaw) + " "
    //                            + String(receiver_data_.pitch) + " "
    //                            + String(receiver_data_.roll));
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

        // Serial.println(String(imu_data_.angular_rate.x) + " " +
        //                String(imu_data_.angular_rate.y) + " " +
        //                String(imu_data_.angular_rate.z));

        // Serial.println(String(imu_data_.angle.roll) + " " +
        //                String(imu_data_.angle.pitch) + " " +
        //                String(imu_data_.angle.yaw));

        // uint32_t current = micros();
        // uint32_t elapsed = current - previous;
        //
        // if (elapsed >= 5000) {
        //     previous = current;
        //     Serial.print(String(imu_data_.yaw) + ", ");
        //
        //     imu_data_.yaw = yaw_filter_->Filter(imu_data_.yaw);
        //     Serial.println(String(imu_data_.yaw) + ", " + String(step));
        //     step += 5;
        // }
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
        pid_data_.angular_rate.x = pid_controller_.angular_rate.x->Update(
            receiver_data_.roll,            // setpoint
            imu_data_.angular_rate.x);      // process

        pid_data_.angular_rate.y = pid_controller_.angular_rate.y->Update(
            receiver_data_.pitch,
            imu_data_.angular_rate.y);

        pid_data_.angular_rate.z = pid_controller_.angular_rate.z->Update(
            receiver_data_.yaw,
            imu_data_.angular_rate.z);

        pid_data_.angular_rate.z = -pid_data_.angular_rate.z;   // TODO in PID class!!!!!!!!!!!!!!!!!!!

        // Serial.println(
        //     String(pid_data_.angular_rate.x) + " " +
        //     String(pid_data_.angular_rate.y) + " " +
        //     String(pid_data_.angular_rate.z));

        Serial.println(String(receiver_data_.roll) + " " +
            String(pid_data_.angular_rate.x));
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


    // Serial.println("FR: " + String(motors_speeds_[FRONT_RIGHT]) +
    //                "  FL: " + String(motors_speeds_[FRONT_LEFT]) +
    //                "  BR: " + String(motors_speeds_[BACK_RIGHT]) +
    //                "  BL: " + String(motors_speeds_[BACK_LEFT]));
}

void control::FlightController::WriteMotorsSpeeds()
{
    esc_manager_->SetSpeeds(motors_speeds_);
}
