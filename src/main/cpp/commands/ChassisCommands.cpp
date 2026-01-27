#include "commands/ChassisCommands.h"

#pragma region ChassisZeroHeading(Chassis *chassis)
/// @brief Creates a command to zero the heading of the gyro.
/// @param chassis A pointer to the chassis subsystem.
/// @return A CommandPtr that resets the gyro yaw to zero.
frc2::CommandPtr ChassisZeroHeading(Chassis *chassis)
{
    
    // Create and return a InstantCommand that resets the gyro yaw
    return frc2::InstantCommand{
        [=] () {chassis->ZeroHeading();},
        { chassis } // Requirements (subsystems required by this command)
    }.ToPtr();
}
#pragma endregion

#pragma region ChassisXMode
/// @brief A command that toggles in between XMode and driving mode
frc2::CommandPtr ChassisXMode(Chassis *chassis)
{
    return frc2::InstantCommand{
        [=] {
            if (!chassis->GetXMode())
            {
                chassis->SetXMode(true);
            } 
            else
            {
                chassis->SetXMode(false);
            }
        }, 
        {chassis}
    }.ToPtr();
}
#pragma endregion

#pragma region ChassisDrive(Chassis* chassis, std::function<frc::ChassisSpeeds()> chassisSpeedsSupplier)
 /// @brief Creates a command to drive the chassis using the provided speeds supplier.
///  @param chassis A pointer to the chassis subsystem.
///  @param chassisSpeedsSupplier A function that supplies the desired chassis speeds.
///  @return A CommandPtr that executes the chassis drive functionality.
frc2::CommandPtr ChassisDrive(Chassis *chassis, std::function<frc::ChassisSpeeds()> chassisSpeedsSupplier)
{
    // Create and return a repeating InstantCommand that drives the chassis
    return frc2::InstantCommand
    {
        [chassis, chassisSpeedsSupplier] () { chassis->Drive(chassisSpeedsSupplier()); },  // Execution function (runs repeatedly while the command is active)
        { chassis }                                                                        // Requirements (subsystems required by this command)
    }.ToPtr().Repeatedly();
    // because of how we implement it, I'm not sure if it needs to be .Repeatedly()'d but it won't hurt
}
#pragma endregion

#pragma region ChassisDrivePose(Chassis *chassis, frc::Pose2d targetPose)
/// @brief Creates a command to drive the chassis to a specified pose.
/// @param chassis A pointer to the chassis subsystem.
/// @param targetPose The target pose to drive to. End goal state relative to the origin, blue alliance side.
/// @return A CommandPtr that drives the chassis to the specified pose.
frc2::CommandPtr ChassisDrivePose(Chassis *chassis, frc::Pose2d targetPose)
{
    if (ChassisConstants::usingPathplanner)
    {
        return pathplanner::AutoBuilder::pathfindToPose(targetPose, ChassisConstants::constraints);
    }
    else
    {
        // // Set up config for trajectory
        // frc::TrajectoryConfig trajectoryConfig(m_speed, ChassisPoseConstants::MaxAcceleration);

        // // Add kinematics to ensure maximum speed is actually obeyed
        // trajectoryConfig.SetKinematics(m_drivetrain->m_kinematics);

        // // Ensure the new pose requires an X or Y move
        // // Note: GenerateTrajectory will throw an exception if the distance X and Y are zero
        // if (fabs(m_distanceX.value()) < 0.001 && fabs(m_distanceY.value()) < 0.001)
        //     m_distanceX = 0.01_in;

        // // Get the robot starting pose
        // auto startPose = m_drivetrain->GetPose();

        // // Create the trajectory to follow
        // frc::Pose2d endPose{startPose.X()                  + m_distanceX,
        //                     startPose.Y()                  + m_distanceY,
        //                     startPose.Rotation().Degrees() + m_angle};

        // // Create the trajectory to follow
        // auto trajectory = frc::TrajectoryGenerator::GenerateTrajectory(startPose, {}, endPose, trajectoryConfig);

        // // Create a profile PID controller
        // frc::ProfiledPIDController<units::radians> profiledPIDController{ChassisPoseConstants::PProfileController, 0, 0,
        //                                                                  ChassisPoseConstants::ThetaControllerConstraints};

        // // enable continuous input for the profile PID controller
        // profiledPIDController.EnableContinuousInput(units::radian_t{-std::numbers::pi}, units::radian_t{std::numbers::pi});

        // // Create the swerve controller command
        // m_swerveControllerCommand = new frc2::SwerveControllerCommand<4>(
        //     trajectory,
        //     [this]() { return m_drivetrain->GetPose(); },
        //     m_drivetrain->m_kinematics,
        //     frc::PIDController(ChassisPoseConstants::PXController, 0, 0),
        //     frc::PIDController(ChassisPoseConstants::PYController, 0, 0),
        //     profiledPIDController,
        //     [this](auto moduleStates) { m_drivetrain->SetModuleStates(moduleStates); },
        //     {m_drivetrain}
        // );
    }
}
#pragma endregion

#pragma region FlipFieldCentricity(Chassis *chassis)
/// @brief Creates a command to flip the field centricity of the chassis.
/// @param chassis A pointer to the chassis subsystem.
/// @return A CommandPtr that flips the field centricity.
frc2::CommandPtr FlipFieldCentricity(Chassis *chassis)
{
    // Create and return a InstantCommand that flips the field centricity
    return frc2::InstantCommand{
        [chassis] () { chassis->FlipFieldCentric(); }, // Execution function
        { chassis } // Requirements (subsystems required by this command)
    }.ToPtr();
}
#pragma endregion

// Effectively dead code, this is replaced by frc lib 
// #pragma region AlignToNearestTag(Chassis *chassis, frc::Transform2d targetOffset)
// // This command will align the robot to the nearest AprilTag
// // It will use the AprilTag's pose to determine the target position and rotation
// // The robot will drive towards the target position and rotate to face the target rotation
// frc2::CommandPtr AlignToNearestTag(Chassis *chassis, frc::Transform2d targetOffset)
// {
//         // Rotate offset and offset it relative to the target position's orientation
//         // This does actually work trust chat
//         frc::Pose2d targetWithOffset{
//             targetPosition.X() + targetOffset.Translation().X() * std::cos(targetPosition.Rotation().Radians().value())
//                                - targetOffset.Translation().Y() * std::sin(targetPosition.Rotation().Radians().value()),

//             targetPosition.Y() + targetOffset.Translation().X() * std::sin(targetPosition.Rotation().Radians().value())
//                                + targetOffset.Translation().Y() * std::cos(targetPosition.Rotation().Radians().value()),

//             targetPosition.Rotation().Degrees() + targetOffset.Rotation().Degrees()
//         };

//     return ChassisDrivePose(chassis, targetWithOffset);
// }
// #pragma endregion

