#include "subsystems/Chassis.h"

#pragma region Chassis
/// @brief Constructor for the Chassis subsystem
Chassis::Chassis()
{
    // TODO: When the robot CAD is done, update the pathplanner robot settings
    pathplanner::RobotConfig config = pathplanner::RobotConfig::fromGUISettings();

    // Configure the AutoBuilder
    pathplanner::AutoBuilder::configure(
        [this] { return GetPose(); },                                       // Robot pose supplier
        [this] (frc::Pose2d pose) { ResetPose(pose); },                     // Method to reset odometry (will be called if your auto has a starting pose)
        [this] { return m_kinematics.ToChassisSpeeds(GetModuleStates()); }, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
        [this] (auto speeds, auto feedforwards) { DriveRelative(speeds); }, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also optionally outputs individual module feedforwards
        std::make_shared<pathplanner::PPHolonomicDriveController>(          // PPHolonomicController is the built in path following controller for holonomic drive trains
            // TODO: magic numbers, test these
            pathplanner::PIDConstants(5.0, 0.0, 0.0),                       // Translation PID constants
            pathplanner::PIDConstants(5.0, 0.0, 0.0)                        // Rotation PID constants
        ),
        config,  // The robot configuration
        []() {
            // Boolean supplier that controls when the path will be mirrored for the red alliance
            // This will flip the path being followed to the red side of the field.
            // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

            // THIS MEANS TO DESIGN ALL AUTOS AS BEING ON THE BLUE SIDE!!!!

            auto alliance = frc::DriverStation::GetAlliance();
            if (alliance) 
                return alliance.value() == frc::DriverStation::Alliance::kRed;
            
            return false;
        },
        this // Reference to this subsystem to set requirements
    );

    // Load a path from the pathplanner path files to verify things are working
    auto path = pathplanner::PathPlannerPath::fromPathFile("Backup");

    // Create a path following command using AutoBuilder. This will also trigger event markers.
    auto command = pathplanner::AutoBuilder::followPath(path);
}
#pragma endregion

#pragma region Drive
/// @brief Method to drive the chassis with the specified speeds.
/// @param speeds The desired chassis speeds.
void Chassis::Drive(const frc::ChassisSpeeds& speeds)
{
    // Call the relative drive method with the correct frame of reference
    DriveRelative(m_isFieldRelative ? frc::ChassisSpeeds::FromFieldRelativeSpeeds(speeds, GetHeading()) : speeds);
}
#pragma endregion

#pragma region DriveRelative
/// @brief Method to drive the chassis with the specified speeds.
/// @param speeds The desired chassis speeds.
void Chassis::DriveRelative(const frc::ChassisSpeeds& speeds)
{
    // If the chassis is in x mode, than stay in x mode, ignoring the desired speeds
    if (m_isXMode)
    {
        // Set the module states to x mode
        SetModuleStates(ChassisConstants::xStates);

        // Save the desired speeds for logging later
        m_desiredStates = ChassisConstants::xStates;
        return;
    }

    // Save the desired speeds for logging later
    m_desiredSpeeds = speeds;

    // Save the desired states for use and logging later
    m_desiredStates = m_kinematics.ToSwerveModuleStates(speeds);

    // Set the desired state for each swerve module
    SetModuleStates(m_desiredStates);
}
#pragma endregion

#pragma region SetModuleStates
/// @brief Method to set the desired states for the swerve modules.
/// @param states The desired states for each swerve module.
void Chassis::SetModuleStates(wpi::array<frc::SwerveModuleState, 4> states)
{
    // Set the desired state for each swerve module
    m_swerveModules[0].SetDesiredState(states[0], "Front Left " );
    m_swerveModules[1].SetDesiredState(states[1], "Front Right ");
    m_swerveModules[2].SetDesiredState(states[2], "Rear Left "  );
    m_swerveModules[3].SetDesiredState(states[3], "Rear Right " );
}
#pragma endregion

#pragma region ResetGyroAngle
/// @brief Method to zero the robot heading.
void Chassis::ResetGyroAngle()
{
    // Do the sim representation
    if (frc::RobotBase::IsSimulation())
    {
        m_simGyro = 0_deg;
        return;
    }
        
    // Zero the gyro heading
    m_gyro.Reset();
}
#pragma endregion

#pragma region ResetWheelAnglesToZero 
/// @brief Method to reset the wheel angles to zero.
void Chassis::ResetWheelAnglesToZero()
{
    // We dont want to do this in simulation
    if (frc::RobotBase::IsSimulation())
        return;

    // Set the swerve wheel angles to zero
    m_swerveModules[0].SetWheelAngleToForward(ChassisConstants::FrontLeftForwardAngle);
    m_swerveModules[1].SetWheelAngleToForward(ChassisConstants::FrontRightForwardAngle);
    m_swerveModules[2].SetWheelAngleToForward(ChassisConstants::BackLeftForwardAngle);
    m_swerveModules[3].SetWheelAngleToForward(ChassisConstants::BackRightForwardAngle);
}
#pragma endregion

#pragma region ResetPose
/// @brief Resets Pose and odometry
void Chassis::ResetPose(frc::Pose2d pose)
{
    for (auto& swerveModule : m_swerveModules)
    {
        swerveModule.ResetEncoders();
    }

    m_poseEstimator.Update(pose.Rotation(), GetModulePositions());

    m_poseEstimator.ResetPose(pose);
    static int counter = 0;
    Log("Reset Pose ", pose.X().value());
    Log("reset counter ", counter++);

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

#pragma region ToggleFieldCentric
/// @brief Method to flip the field centric mode.
void Chassis::ToggleFieldCentric()
{
    // Toggle the field relative mode
    m_isFieldRelative = !m_isFieldRelative;
}
#pragma endregion

#pragma region ToggleXMode
/// @brief Method to flip X mode.
void Chassis::ToggleXMode()
{
    // Set whether the chassis in x mode
    m_isXMode = !m_isXMode;
}
#pragma endregion

#pragma region GetHeading
/// @brief Method to get the robot heading.
/// @return The robot heading.
frc::Rotation2d Chassis::GetHeading()
{
    // In sim, return the simulated angle
    if (frc::RobotBase::IsSimulation())
        return frc::Rotation2d{m_simGyro};

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

#pragma region GetSpeeds
/// @brief Method to get the robot chassis speeds.
/// @return The robot chassis speeds.
frc::ChassisSpeeds Chassis::GetSpeeds()
{
    // Return the desired chassis speeds
    return m_desiredSpeeds;
}
#pragma endregion

#pragma region Periodic
/// @brief Method called once per scheduler run.
void Chassis::Periodic()
{
    // Update gyro sim, advance by the cycle time (20 milliseconds)
    m_simGyro += m_desiredSpeeds.omega * 0.02_s;

    // Update the pose estimator
    m_poseEstimator.Update(GetHeading(), GetModulePositions());

    if (frc::RobotBase::IsSimulation())
    {
        for (auto& swerveModule : m_swerveModules)
            swerveModule.SimPeriodic();
    }
    else
    {
        // This also updates the pose estimator with vision as well as updating photonvisions internal estimators
        m_topVision.Periodic();
    }

    /// *** Logging *** ///
    Log("Actual Swerve Module States ",  GetModuleStates());
    Log("Actual Chassis Speeds ",        m_kinematics.ToChassisSpeeds(GetModuleStates()));

    Log("Desired Chassis Speeds ",       m_desiredSpeeds);
    Log("Desired Swerve Module States ", m_desiredStates);

    Log("Actual Robot Pose ", GetPose());
    Log("Field relative ",    m_isFieldRelative);
}
#pragma endregion
