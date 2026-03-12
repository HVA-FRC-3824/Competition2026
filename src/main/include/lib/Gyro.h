#pragma once

#include "studica/AHRS.h"
#include <frc/geometry/Rotation2d.h>
#include <units/angular_velocity.h>
#include <units/angle.h>
#include <frc/DriverStation.h>

class Gyro
{
    public:

        explicit Gyro();

        frc::Rotation2d GetDriverHeading();
        frc::Rotation2d GetPoseHeading();
        void            Reset();
        void            SimPeriodic(units::degrees_per_second_t omega);

    private:

        frc::Rotation2d GetRawHeading();

        studica::AHRS   m_gyro   {studica::AHRS::NavXComType::kMXP_SPI};
        units::degree_t m_simGyro{0_deg};

        frc::Rotation2d m_driverOffset{0_deg};
        frc::Rotation2d m_poseOffset  {0_deg};
};
