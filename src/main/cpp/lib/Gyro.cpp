#include "lib/Gyro.h"

#include <frc/RobotBase.h>

#pragma region Gyro
/// @brief Constructor for the Gyro wrapper
Gyro::Gyro()
{
    // Optional alliance check for the 180 offset in pose estimation
    auto alliance = frc::DriverStation::GetAlliance();

    if (alliance && alliance.value() == frc::DriverStation::Alliance::kBlue)
    {
        m_poseOffset = frc::Rotation2d{180_deg};
    }   
}
#pragma endregion

#pragma region GetDriverHeading
/// @brief Gets the driver's current heading
/// @return The rotation for the driver (field relative)
frc::Rotation2d Gyro::GetDriverHeading()
{
    // Subtract the driver offset from the raw heading
    return GetRawHeading() - m_driverOffset;
}
#pragma endregion

#pragma region GetPoseHeading
/// @brief Gets the current heading for pose estimation
/// @return The rotation for pose estimation
frc::Rotation2d Gyro::GetPoseHeading()
{
    auto heading = GetRawHeading() - m_poseOffset;

    return heading;
}
#pragma endregion

#pragma region DriverReset
/// @brief Resets driver offsets to the current actual gyro heading
void Gyro::DriverReset()
{
    frc::Rotation2d currentHeading = GetRawHeading();
    m_driverOffset = currentHeading.Degrees() + 180_deg;
}
#pragma endregion

#pragma region PoseReset
/// @brief Resets driver offsets to the current actual gyro heading
void Gyro::PoseReset()
{
    m_gyro.Reset();
}
#pragma endregion

#pragma region SimPeriodic
/// @brief Advances the simulated gyro state
/// @param omega Current angular velocity
void Gyro::SimPeriodic(units::degrees_per_second_t omega)
{
    if (frc::RobotBase::IsSimulation())
    {
        // Advance by the cycle time (20 milliseconds)
        m_simGyro += omega * 0.02_s;
    }
}
#pragma endregion

#pragma region GetRawHeading
/// @brief Gets the raw navx rotation, or simulated rotation
/// @return The raw Rotation2d
frc::Rotation2d Gyro::GetRawHeading()
{
    if (frc::RobotBase::IsSimulation())
    {
        return frc::Rotation2d{m_simGyro};
    }

    // Return the gyro rotation (negated because NavX is mounted upside down)
    return -m_gyro.GetRotation2d();
}
#pragma endregion
