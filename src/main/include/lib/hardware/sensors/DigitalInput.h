#pragma once

#include "lib/hardware/sensors/Sensor.h"

#include <frc/DigitalInput.h>


namespace hardware
{

namespace sensor
{

    class DigitalInput : public Sensor<bool>
    {
        public:

            DigitalInput(int CanId)
                : m_sensor{CanId}
            {}

            inline bool operator==(bool operand) override
            {
                return m_sensor.Get() == operand;
            }

            inline operator bool() override
            {
                return m_sensor.Get();
            }

            inline bool Get() override
            {
                return m_sensor.Get();
            }

    private:

        frc::DigitalInput m_sensor;

    };
}

}
