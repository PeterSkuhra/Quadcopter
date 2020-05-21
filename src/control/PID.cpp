#include "PID.hpp"


control::PID::PID(float p_gain,
                  float i_gain,
                  float d_gain,
                  float out_limit,
                  bool reverse_output) :
    output_limit_(out_limit),
    reverse_output_(reverse_output),
    prev_time_(0)
{
    gains_.p = p_gain;
    gains_.i = i_gain;
    gains_.d = d_gain;
}

float control::PID::Update(float setpoint, float process)
{
    float pid_output;
    const float kError = process - setpoint;
    const float kPrevError = prev_process_ - prev_setpoint_;

    uint32_t current_time = millis();
    uint32_t elapsed_time = current_time - prev_time_;

    outputs_.p = kError * gains_.p;
    outputs_.i = (outputs_.i + (kError * elapsed_time)) * gains_.i;
    // outputs_.i += (kError * gains_.i);           // POZOR, robi blbosti!!!!!!!!!!!!!!!!!!!!
    // outputs_.d = ((kError - kPrevError) / elapsed_time) * gains_.d;
    outputs_.d = (kError - kPrevError) * gains_.d;

    prev_setpoint_ = setpoint;
    prev_process_ = process;

    pid_output = outputs_.p + outputs_.i + outputs_.d;

    pid_output = constrain(pid_output, -output_limit_, output_limit_);

    if (reverse_output_) {
        pid_output = -pid_output;
    }

    prev_time_ = current_time;

    return pid_output;
}

void control::PID::SetPIDGains(float p_gain, float i_gain, float d_gain)
{
    gains_.p = p_gain;
    gains_.i = i_gain;
    gains_.d = d_gain;
}

void control::PID::SetOutputLimit(float limit)
{
    output_limit_ = limit;
}

float control::PID::GetOutputLimit() const
{
    return output_limit_;
}
