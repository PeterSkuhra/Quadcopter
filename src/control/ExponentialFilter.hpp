#ifndef EXPONENTIAL_FILTER_HPP
#define EXPONENTIAL_FILTER_HPP


/******************************************************************************
 *  Generic class which implements basic exponential filter.
 *  Its behaviour is like low pass filter.
 *  It attenuates signals with high frequencies.
 *****************************************************************************/
template<typename T> class ExponentialFilter
{
public:

    /**
     *  Constructor.
     *
     *  @param weight       Weight of filtering. In range [0, 100].
     *                      Lower value means better smoothing.
     *  @param init_value   Initialization value for start smoothing
     */
    ExponentialFilter(T weight, T init_value) :
        weight_(constrain(weight, 0.0, 100.0) / 100.0),
        smooth_value_(init_value)
    {
    }

    /**
     *  Filters raw value and returns smooth value.
     *
     *  @param raw_value    Raw value to filter.
     *
     *  @return     Smoothed value from current raw value
     *              and previous smoothed value.
     */
    T Filter(T raw_value)
    {
        smooth_value_ = (weight_ * raw_value) +
                        ((1.0 - weight_) * smooth_value_);

        return smooth_value_;
    }

    /**
     *  Resets filter.
     *  Smoothed value will be zero.
     */
    void Reset()
    {
        smooth_value_ = 0;
    }

    /**
     *  Resets filter to initialization value.
     *
     *  @param init_value   initialization value
     */
    void Reset(T init_value)
    {
        smooth_value_ = init_value;
    }

    /**
     *  Sets weight of filtering. In range [0, 100]
     *  Lower value of weight means more smoothed signal.
     *
     *  @param weight   weight of filtering
     */
    void SetWeight(T weight)
    {
        weight_ = constrain(weight, 0.0, 100.0) / 100.0;
    }

    /**
     *  Returns current weight of filtering in range [0, 100].
     *
     *  @return current weight of filtering
     */
    T GetWeight() const
    {
        return weight_ * 100.0;
    }


private:

    /**
     *  Weight of filtering. In range [0, 1]
     */
    T weight_;

    /**
     *  Filtered (smoothed) value.
     */
    T smooth_value_;
};

#endif
