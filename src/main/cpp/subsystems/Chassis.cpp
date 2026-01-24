#include "subsystems/Chassis.h"

#pragma region Chassis
Chassis::Chassis()
{
    pathplanner::RobotConfig config = pathplanner::RobotConfig::fromGUISettings();

    // Configure the AutoBuilder
    pathplanner::AutoBuilder::configure(
        [this] () { return GetPose(); }, // Robot pose supplier
        [this] (frc::Pose2d pose) { m_poseEstimator.ResetPose(pose); }, // Method to reset odometry (will be called if your auto has a starting pose)
        [this] () { return m_desiredSpeeds; }, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
        [this] (auto speeds, auto feedforwards) { DriveRobotRelative(speeds); }, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also optionally outputs individual module feedforwards
        std::make_shared<pathplanner::PPHolonomicDriveController>( // PPHolonomicController is the built in path following controller for holonomic drive trains
            // TODO: magic numbers, test these
            pathplanner::PIDConstants(1.0, 0.0, 0.0), // Translation PID constants
            pathplanner::PIDConstants(1.0, 0.0, 0.0) // Rotation PID constants
        ),
        config, // The robot configuration
        []() {
            // Boolean supplier that controls when the path will be mirrored for the red alliance
            // This will flip the path being followed to the red side of the field.
            // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

            // THIS MEANS TO DESIGN ALL AUTOS AS BEING ON THE BLUE SIDE!!!!

            auto alliance = frc::DriverStation::GetAlliance();
            if (alliance) {
                return alliance.value() == frc::DriverStation::Alliance::kRed;
            }
            return false;
        },
        this // Reference to this subsystem to set requirements
    );
}
#pragma endregion

#pragma region Drive
/// @brief Method to drive the chassis with the specified speeds.
/// @param speeds The desired chassis speeds.
void Chassis::Drive(const frc::ChassisSpeeds& speeds)
{
    DriveRobotRelative(m_isFieldRelative ? frc::ChassisSpeeds::FromFieldRelativeSpeeds(speeds, GetHeading()) : speeds);
}
#pragma endregion

#pragma region Drive
/// @brief Method to drive the chassis with the specified speeds.
/// @param speeds The desired chassis speeds.
void Chassis::DriveRobotRelative(const frc::ChassisSpeeds& speeds)
{
    // If we're in x mode, we want to stay in x mode, 
    // unless we're not in x mode, then we don't want to be in x mode
    if (m_isXMode)
    {
        SetModuleStates(ChassisConstants::xStates);
        m_desiredStates = ChassisConstants::xStates;
        return;
    }

    // Save the desired speeds for logging later
    m_desiredSpeeds = speeds;

    // Save the desired states for use and logging later
    m_desiredStates = m_kinematics.ToSwerveModuleStates(speeds);

    // Set the desired state for each swerve module
    SetModuleStates(m_desiredStates);

    // // Simulate the gyro in simulation
    // if (frc::RobotBase::IsSimulation())
    //    m_gyro.Update(speeds.omega, 0.02_s);
    //    Log("Sim Angular Velocity ", speeds.omega.value());
}
#pragma endregion

#pragma region SetModuleStates
void Chassis::SetModuleStates(wpi::array<frc::SwerveModuleState, 4> states)
{
    m_swerveModules[0].SetDesiredState(states[0], "Front Left");
    m_swerveModules[1].SetDesiredState(states[1], "Front Right");
    m_swerveModules[2].SetDesiredState(states[2], "Rear Left");
    m_swerveModules[3].SetDesiredState(states[3], "Rear Right");
}
#pragma endregion

#pragma region ZeroHeading
/// @brief Method to zero the robot heading.
void Chassis::ZeroHeading()
{
    // Zero the gyro heading
    m_gyro.Reset();
}
#pragma endregion

#pragma region ResetWheelAnglesToZero 
/// @brief Method to reset the wheel angles to zero.
void Chassis::ResetWheelAnglesToZero()
{
    // Set the swerve wheel angles to zero
    m_swerveModules[0].SetWheelAngleToForward(ChassisConstants::FrontLeftForwardAngle);
    // m_swerveModules[1].SetWheelAngleToForward(ChassisConstants::FrontRightForwardAngle);
    // m_swerveModules[2].SetWheelAngleToForward(ChassisConstants::BackLeftForwardAngle);
    // m_swerveModules[3].SetWheelAngleToForward(ChassisConstants::BackRightForwardAngle);
}
#pragma endregion

#pragma region ResetDriveEncoders
/// @brief Method to reset the drive encoders.
void Chassis::ResetDriveEncoders()
{
    // Reset the swerve motor encoders
    for (auto& swerveModule : m_swerveModules)
    {
        swerveModule.ResetDriveEncoder();
    }
}
#pragma endregion

#pragma region GetModuleStates
/// @brief Method to get the current swerve module states.
/// @return The current swerve module states.
wpi::array<frc::SwerveModuleState, 4> Chassis::GetModuleStates()
{
    // Return the swerve module states
    return wpi::array<frc::SwerveModuleState, 4>
    {
        m_swerveModules[0].GetState(),
        m_swerveModules[1].GetState(),
        m_swerveModules[2].GetState(),
        m_swerveModules[3].GetState()
    };
}
#pragma endregion

#pragma region GetModulePositions
/// @brief Method to get the current swerve module positions.
/// @return The current swerve module positions.
wpi::array<frc::SwerveModulePosition, 4> Chassis::GetModulePositions()
{
    // Return the swerve module states
    return wpi::array<frc::SwerveModulePosition, 4>
    {
        m_swerveModules[0].GetPosition(),
        m_swerveModules[1].GetPosition(),
        m_swerveModules[2].GetPosition(),
        m_swerveModules[3].GetPosition()
    };
}
#pragma endregion

#pragma region FlipFieldCentric
/// @brief Method to flip the field centric mode.
void Chassis::FlipFieldCentric()
{
    // Toggle the field relative mode
    m_isFieldRelative = !m_isFieldRelative;
}
#pragma endregion

#pragma region GetXMode
bool Chassis::GetXMode()
{
    return m_isXMode;
}
#pragma endregion

#pragma region SetXMode
void Chassis::SetXMode(bool isXMode)
{
    m_isXMode = isXMode;
}
#pragma endregion

#pragma region GetHeading
/// @brief Method to get the robot heading.
/// @return The robot heading.
frc::Rotation2d Chassis::GetHeading()
{
    // Return the gyro rotation
    return m_gyro.GetRotation2d();
}
#pragma endregion

#pragma region GetPose
/// @brief Method to get the robot pose.
/// @return The robot pose.
frc::Pose2d Chassis::GetPose()
{
    // Return the estimated robot pose
    return m_poseEstimator.GetEstimatedPosition();
}
#pragma endregion

#pragma region GetChassisSpeeds
/// @brief Method to get the robot chassis speeds.
frc::ChassisSpeeds Chassis::GetChassisSpeeds()
{
    return m_desiredSpeeds;
}


#pragma region Periodic
/// @brief Method called once per scheduler run.
void Chassis::Periodic()
{
    // Update odometry
    // Update the pose estimator
    m_poseEstimator.Update(GetHeading(), GetModulePositions());

    // This also updates the pose estimator with vision as well as updating photonvisions internal estimators
    m_vision.Periodic();

    // Logging
    Log("Swerve Module States ",         GetModuleStates());
    Log("Desired Swerve Module States ", m_desiredStates);

    Log("Swerve Module Positions ", GetModulePositions());

    Log("Desired Chassis Speeds ", m_desiredSpeeds);
    Log("Actual Chassis Speeds ",  m_kinematics.ToChassisSpeeds(GetModuleStates()));

    Log("Robot Pose ", GetPose());
    Log("Gyro ", m_gyro.GetRotation2d().Degrees().value());

    Log("field relative ", m_isFieldRelative);
}
#pragma endregion