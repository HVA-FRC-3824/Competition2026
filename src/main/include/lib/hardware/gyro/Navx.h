#pragma once

#include "GyroBase.h"

#include <studica/AHRS.h>

#include <frc/RobotBase.h>
#include <frc/geometry/Rotation3d.h>

#include <units/angle.h>
#include <units/angular_velocity.h>

namespace hardware
{
    namespace gyro
    {
        // NavX class to support the NavX gyro
        class Navx : public GyroBase
        {
            public:

                Navx();

                frc::Rotation3d GetRotation() override;                         // Get the rotation of the gyro
                frc::Rotation3d GetOffset()   override;                         // Get the offset of the gyro
                void            ResetYaw()    override;                         // Reset the yaw of the gyro

                void            SetOffset(frc::Rotation3d offset) override;     // Set the offset of the gyro

                void            Update(units::radians_per_second_t omegaRadiansPerSec, units::second_t dt) override {}

            private:

                studica::AHRS               m_gyro{studica::AHRS::NavXComType::kMXP_SPI};  // NavX gyro

                frc::Rotation3d             m_offset{0_deg, 0_deg, 0_deg};      // The offset of the gyro
        };
    }
}
