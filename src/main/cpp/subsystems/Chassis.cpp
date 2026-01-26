#include "subsystems/Chassis.h"

#pragma region Chassis
/// @brief Constructor for the Chassis subsystem
Chassis::Chassis()
{
    pathplanner::RobotConfig config = pathplanner::RobotConfig::fromGUISettings();

    // Configure the AutoBuilder
    pathplanner::AutoBuilder::configure(
        [this] () { return GetPose(); },                                    // Robot pose supplier
        [this] (frc::Pose2d pose) { m_poseEstimator.ResetPose(pose); },     // Method to reset odometry (will be called if your auto has a starting pose)
        [this] () { return m_desiredSpeeds; },                              // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
        [this] (auto speeds, auto feedforwards) { DriveRelative(speeds); }, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also optionally outputs individual module feedforwards
        std::make_shared<pathplanner::PPHolonomicDriveController>(          // PPHolonomicController is the built in path following controller for holonomic drive trains
            // TODO: magic numbers, test these
            pathplanner::PIDConstants(1.0, 0.0, 0.0), // Translation PID constants
            pathplanner::PIDConstants(1.0, 0.0, 0.0)  // Rotation PID constants
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

    auto path = pathplanner::PathPlannerPath::fromPathFile("Example Path");

    // Create a path following command using AutoBuilder. This will also trigger event markers.
    auto command = pathplanner::AutoBuilder::followPath(path);

    m_timer.Reset();
    m_timer.Start();
    m_perodicCounter = 0;
}
#pragma endregion

#pragma region Drive
/// @brief Method to drive the chassis with the specified speeds.
/// @param speeds The desired chassis speeds.
void Chassis::Drive(const frc::ChassisSpeeds& speeds)
{
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

    // // Simulate the gyro in simulation
    // if (frc::RobotBase::IsSimulation())
    //    m_gyro.Update(speeds.omega, 0.02_s);
    //    Log("Sim Angular Velocity ", speeds.omega.value());
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
/// @brief Method to get whether the chassis is in X mode.
/// @return True if the chassis is in X mode, false otherwise.
bool Chassis::GetXMode()
{
    // Return whether the chassis is in X mode
    return m_isXMode;
}
#pragma endregion

#pragma region SetXMode
/// @brief Method to set whether the chassis is in X mode.
/// @param isXMode True to set X mode, false otherwise.
void Chassis::SetXMode(bool isXMode)
{
    // Set whether the chassis in x mode
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
    // Update the pose estimator
    m_poseEstimator.Update(GetHeading(), GetModulePositions());

    // This also updates the pose estimator with vision as well as updating photonvisions internal estimators
    m_vision.Periodic();

    // frc::SmartDashboard::PutNumber("Chassis Timer", m_timer.Get().value());

    // if (frc::DriverStation::IsEnabled())
    // {
    //     // Determine if 10 seconds have elapsed
    //     if (m_timer.HasElapsed(5_s))
    //     {
    //         // Reset the timer
    //         m_timer.Reset();
    
    //         m_perodicCounter++;
    
    //         frc::SmartDashboard::PutNumber("Chassis Counter", m_perodicCounter);
    
    //         frc::SwerveModuleState swerveModuleState;
    //         swerveModuleState.angle = frc::Rotation2d{360_deg * m_perodicCounter};
    //         swerveModuleState.speed = 0_mps;
    
    //         frc::SmartDashboard::PutNumber("Swerve Rotation", swerveModuleState.angle.Degrees().value());
    
    //         // The Swerve module angles in increments of 360 degrees
    //         m_swerveModules[0].SetDesiredState(swerveModuleState, "Front Left " );
    //         m_swerveModules[1].SetDesiredState(swerveModuleState, "Front Right ");
    //         m_swerveModules[2].SetDesiredState(swerveModuleState, "Rear Left "  );
    //         m_swerveModules[3].SetDesiredState(swerveModuleState, "Rear Right " );
    //     }
    // }
    // else 
    // {
    //     m_timer.Reset();
    // } 

    frc::SwerveModulePosition position =  m_swerveModules[0].GetPosition(); // Hack to get around compiler optimization removing unused code
    frc::SwerveModuleState    state     = m_swerveModules[0].GetState();    // Hack to get around compiler optimization removing unused code
    
    frc::SmartDashboard::PutNumber("Chassis FL Position", position.angle.Degrees().value());
    frc::SmartDashboard::PutNumber("Chassis FL Distance", position.distance.value());
    
    frc::SmartDashboard::PutNumber("Chassis FL State",    state.angle.Degrees().value());
    frc::SmartDashboard::PutNumber("Chassis FL Speed",    state.speed.value());

   /// *** Logging *** ///
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