#pragma once

#include <memory>

#include <frc/geometry/Rotation3d.h>

#include "lib/hardware/hardware.h"

namespace hardware::gyro
{
    class GyroBase : public Hardware
    {
        public:

            virtual frc::Rotation3d GetRotation()                     = 0;
            virtual frc::Rotation3d GetOffset()                       = 0;
            virtual void            ResetYaw()                        = 0;
            virtual void            SetOffset(frc::Rotation3d offset) = 0;
    };
}
