#ifndef PID_HPP
#define PID_HPP

#include <Arduino.h>

namespace control
{

class PID
{
public:
    /**
     *  Constructor.
     *
     *
     *  @param p_gain   P gain value
     *  @param i_gain   I gain value
     *  @param d_gain   D gain value
     */
    PID(float p_gain,
        float i_gain,
        float d_gain,
        float out_limit,
        bool reverse_output = false);

    /**
     *  Returns output of PID controller.
     *
     *  @param setpoint
     *  @param process
     *
     *  @return float   output of PID controller
     */
    float Update(float setpoint, float process);

    /**
     *  Sets PID gains.
     *
     *  @param p_gain   P gain value
     *  @param i_gain   I gain value
     *  @param d_gain   D gain value
     */
    void SetPIDGains(float p_gain, float i_gain, float d_gain);

    /**
     *  Sets output limit of PID controller.
     *  Result output limit value is absolute value of limit.
     *
     *  For example: SetOutputLimit(100);
     *  output_limit = [-100, 100]
     *
     *  @param limit    absolute value of output limit
     */
    void SetOutputLimit(float limit);

    /**
     *  Returns output limit od PID controller.
     *
     *  @return output limit of PID controller
     */
    float GetOutputLimit() const;

    /**
     *  Structure for PID gains.
     */
    struct PIDData
    {
        float p;
        float i;
        float d;
    };


private:

    PIDData gains_;
    PIDData outputs_;

    float prev_setpoint_;
    float prev_process_;
    uint32_t prev_time_;

    float output_limit_;
    bool reverse_output_;
};

}

#endif
