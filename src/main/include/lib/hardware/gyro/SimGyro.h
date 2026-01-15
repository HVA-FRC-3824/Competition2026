#pragma once

#include <frc/geometry/Rotation3d.h>
#include <units/angle.h>
#include <units/angular_velocity.h>

#include "lib/hardware/gyro/GyroBase.h"


namespace hardware
{

namespace gyro
{

    class SimGyro : public GyroBase
    {
        public:
            SimGyro() {
            }

            void ResetYaw() override
            {
                m_yawRadians = 0_rad;
            }

            frc::Rotation3d GetRotation() override
            {
                return frc::Rotation3d{m_yawRadians};
            }

            frc::Rotation3d GetOffset() override
            {
                return frc::Rotation3d{0_rad, 0_rad, m_yawRadians};
            }

            void SetOffset(frc::Rotation3d offset) override
            {
                m_offset = offset.Angle();
            } 

            // Integrate chassis angular velocity (rad/s) over dt seconds
            void Update(units::radians_per_second_t omegaRadiansPerSec = 0_rad_per_s, units::second_t dt = 0_s) override
            {
                m_angularVelocity = omegaRadiansPerSec;
                m_yawRadians += omegaRadiansPerSec * dt;
            }
        
        private:
            units::radian_t             m_yawRadians     {0.0};
            units::radians_per_second_t m_angularVelocity{0.0};

            units::radian_t             m_offset{0.0};
    };

}

}