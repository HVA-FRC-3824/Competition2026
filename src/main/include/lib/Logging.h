#pragma once

#pragma region Includes
#include <networktables/NetworkTableInstance.h>
#include <networktables/StructArrayTopic.h>
#include <networktables/StructTopic.h>

// Types
#include <frc/kinematics/ChassisSpeeds.h>
#include <frc/kinematics/SwerveModuleState.h>
#include <frc/kinematics/SwerveModulePosition.h>
#pragma endregion

inline void Log(std::string_view name, bool value)
{
    nt::NetworkTableInstance::GetDefault().GetEntry(name).SetBoolean(value);
}

inline void Log(std::string_view name, int value)
{
    nt::NetworkTableInstance::GetDefault().GetEntry(name).SetInteger(value);
}

inline void Log(std::string_view name, double value)
{
    nt::NetworkTableInstance::GetDefault().GetEntry(name).SetDouble(value);
}

inline void Log(std::string_view name, std::string_view value)
{
    nt::NetworkTableInstance::GetDefault().GetEntry(name).SetString(value);
}

inline void Log(std::string_view name, wpi::array<frc::SwerveModuleState, 4> value)
{
    nt::NetworkTableInstance::GetDefault().GetStructArrayTopic<frc::SwerveModuleState>(name).Publish().Set(value);
}

inline void Log(std::string_view name, wpi::array<frc::SwerveModulePosition, 4> value)
{
    nt::NetworkTableInstance::GetDefault().GetStructArrayTopic<frc::SwerveModulePosition>(name).Publish().Set(value);
}

inline void Log(std::string_view name, frc::Pose2d value)
{
    nt::NetworkTableInstance::GetDefault().GetStructTopic<frc::Pose2d>(name).Publish().Set(value);
}

inline void Log(std::string_view name, frc::ChassisSpeeds value)
{
    nt::NetworkTableInstance::GetDefault().GetStructTopic<frc::ChassisSpeeds>(name).Publish().Set(value);
}