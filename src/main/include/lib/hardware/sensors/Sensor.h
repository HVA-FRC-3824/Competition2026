#pragma once

#include "lib/hardware/hardware.h"


namespace hardware
{

namespace sensor
{

    template <typename T>  // typename T is the return type of the sensor so it returns
    class Sensor : public Hardware
    {
        public:

            virtual bool operator==(T operand) = 0;

            virtual operator T() = 0;

            virtual T Get() = 0;
    };

}

}
