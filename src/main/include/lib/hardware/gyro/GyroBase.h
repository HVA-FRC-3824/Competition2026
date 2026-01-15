#pragma once

#include <memory>

#include <frc/geometry/Rotation3d.h>
#include <units/angular_velocity.h>

#include "lib/hardware/hardware.h"

namespace hardware::gyro
{
    class GyroBase : public Hardware
    {
        public:

            virtual frc::Rotation3d GetRotation() { return frc::Rotation3d(); };
            virtual frc::Rotation3d GetOffset()   { return frc::Rotation3d(); };
            virtual void            ResetYaw() {};
            virtual void            SetOffset(frc::Rotation3d offset) {};
            virtual void            Update(units::radians_per_second_t omegaRadiansPerSec, units::second_t dt) {};
    };
}
