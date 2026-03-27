package frc.robot.commands;

import java.util.function.Supplier;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Chassis;

public class ChassisCommands
{
    /// @brief Creates a command to zero the heading of the gyro.
    /// @param chassis A pointer to the chassis subsystem.
    /// @return A CommandPtr that resets the gyro yaw to zero.
    public static Command ChassisZeroHeading(Chassis chassis)
    {
        // Create and return a InstantCommand that resets the gyro yaw
        return new InstantCommand(
            () -> {chassis.ResetGyroAngle();},
            chassis  // Requirements (subsystems required by this command)
        );
    }

    /// @brief A command that toggles in between XMode and driving mode
    public static Command ChassisXMode(Chassis chassis)
    {
        // Create and return a InstantCommand that toggles XMode
        return new InstantCommand(
            () -> { chassis.ToggleXMode(); }, 
            chassis
        );
    }

    /// @brief Creates a command to drive the chassis using the provided speeds supplier.
    ///  @param chassis A pointer to the chassis subsystem.
    ///  @param chassisSpeedsSupplier A function that supplies the desired chassis speeds.
    ///  @return A CommandPtr that executes the chassis drive functionality.
    public static Command ChassisDrive(Chassis chassis, Supplier<ChassisSpeeds> chassisSpeedsSupplier)
    {
        // Create and return a repeating InstantCommand that drives the chassis
        return new InstantCommand(
            () -> { chassis.Drive(chassisSpeedsSupplier.get()); },
            chassis
        );
    }

    /// @brief Creates a command to drive the chassis using the provided speeds supplier.
    ///  @param chassis A pointer to the chassis subsystem.
    ///  @param chassisSpeedsSupplier The desired chassis speeds.
    ///  @return A CommandPtr that executes the chassis drive functionality.
    public static Command ChassisDrive(Chassis chassis, ChassisSpeeds chassisSpeeds)
    {
        // Create and return a repeating InstantCommand that drives the chassis
        return new InstantCommand(
            () -> { chassis.Drive(chassisSpeeds); },  // Execution function (runs repeatedly while the command is active)
            chassis                                                                        // Requirements (subsystems required by this command)
        );
        // because of how we implement it, I'm not sure if it needs to be .Repeatedly()'d but it won't hurt
    }

    /// @brief Creates a command to drive the chassis to a specified pose.
    /// @param chassis A pointer to the chassis subsystem.
    /// @param targetPose The target pose to drive to. End goal state relative to the origin, blue alliance side.
    /// @return A CommandPtr that drives the chassis to the specified pose.
    // Command ChassisDrivePose(Chassis chassis, Pose2d targetPose)
    // {
    //     // Use Pathplanner to generate the trajectory and command
    //     return AutoBuilder.pathfindToPose(targetPose, Chassis.constraints);
    // }

    /// @brief Creates a command to flip the field centricity of the chassis.
    /// @param chassis A pointer to the chassis subsystem.
    /// @return A CommandPtr that flips the field centricity.
    public static Command ToggleFieldCentricity(Chassis chassis)
    {
        // Create and return a InstantCommand that flips the field centricity
        return new InstantCommand(
            () -> { chassis.ToggleFieldCentric(); }, // Execution function
            chassis // Requirements (subsystems required by this command)
        );
    }
    /// @brief Creates a command to flip the slowness of the chassis.
    /// @param chassis A pointer to the chassis subsystem.
    /// @return A CommandPtr that flips the slowness.
    public static Command ToggleSlowMode(Chassis chassis)
    {
        // Create and return a InstantCommand that flips the field centricity
        return new InstantCommand(
            () -> { chassis.ToggleSlowMode(); }, // Execution function
            chassis // Requirements (subsystems required by this command)
        );
    }
}