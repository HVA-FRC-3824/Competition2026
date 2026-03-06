#include "commands/ChassisCommands.h"

#pragma region ChassisZeroHeading
/// @brief Creates a command to zero the heading of the gyro.
/// @param chassis A pointer to the chassis subsystem.
/// @return A CommandPtr that resets the gyro yaw to zero.
frc2::CommandPtr ChassisZeroHeading(Chassis *chassis)
{
    // Create and return a InstantCommand that resets the gyro yaw
    return frc2::InstantCommand{
        [=] () {chassis->ResetGyroAngle();},
        { chassis }  // Requirements (subsystems required by this command)
    }.ToPtr();
}
#pragma endregion

#pragma region ChassisXMode
/// @brief A command that toggles in between XMode and driving mode
frc2::CommandPtr ChassisXMode(Chassis *chassis)
{
    // Create and return a InstantCommand that toggles XMode
    return frc2::InstantCommand{
        [=] { chassis->ToggleXMode(); }, 
        { chassis }
    }.ToPtr();
}
#pragma endregion

#pragma region ChassisDrive
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

#pragma region ChassisDrivePose
/// @brief Creates a command to drive the chassis to a specified pose.
/// @param chassis A pointer to the chassis subsystem.
/// @param targetPose The target pose to drive to. End goal state relative to the origin, blue alliance side.
/// @return A CommandPtr that drives the chassis to the specified pose.
frc2::CommandPtr ChassisDrivePose(Chassis *chassis, frc::Pose2d targetPose)
{
    // Use Pathplanner to generate the trajectory and command
    return pathplanner::AutoBuilder::pathfindToPose(targetPose, ChassisConstants::constraints);
}
#pragma endregion

#pragma region ToggleFieldCentricity
/// @brief Creates a command to flip the field centricity of the chassis.
/// @param chassis A pointer to the chassis subsystem.
/// @return A CommandPtr that flips the field centricity.
frc2::CommandPtr ToggleFieldCentricity(Chassis *chassis)
{
    // Create and return a InstantCommand that flips the field centricity
    return frc2::InstantCommand{
        [chassis] () { chassis->ToggleFieldCentric(); }, // Execution function
        { chassis } // Requirements (subsystems required by this command)
    }.ToPtr();
}
#pragma endregion
