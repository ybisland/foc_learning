#include "LowPassFilter.h"

#include <Arduino.h>

namespace
{
    constexpr float kTwoPi = 6.283185307f;
}

LowPassFilter::LowPassFilter()
    : cutoffHz_(0.0f),
      value_(0.0f),
      prevSampleUs_(0),
      hasValue_(false)
{
}

LowPassFilter::LowPassFilter(float cutoffHz)
    : cutoffHz_(cutoffHz),
      value_(0.0f),
      prevSampleUs_(0),
      hasValue_(false)
{
}

void LowPassFilter::configure(float cutoffHz)
{
    cutoffHz_ = cutoffHz;
}

float LowPassFilter::update(float sample)
{
    const uint32_t sampleTimeUs = micros();
    if (hasValue_)
    {
        const float elapsedSeconds = static_cast<float>(sampleTimeUs - prevSampleUs_) * 1.0e-6f;
        const float scaledInterval = kTwoPi * cutoffHz_ * elapsedSeconds;
        const float alpha = scaledInterval / (1.0f + scaledInterval);
        value_ += alpha * (sample - value_);
    }
    else
    {
        value_ = sample;
        hasValue_ = true;
    }

    prevSampleUs_ = sampleTimeUs;
    return value_;
}

void LowPassFilter::reset()
{
    hasValue_ = false;
}