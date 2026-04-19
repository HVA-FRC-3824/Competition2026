package frc.robot.lib.pathing;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;

import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;
import frc.robot.lib.Alliance;
import frc.robot.lib.BLine.FollowPath;
import frc.robot.lib.BLine.Path;
import frc.robot.subsystems.swerve.Swerve;

public class Pathing {

    FollowPath.Builder m_pathBuilder;

    public Pathing(
        Swerve drivetrain,
        PPHolonomicDriveController PPPid,
        PIDController BLineTransitionPid,
        PIDController BLineRotationPid,
        PIDController BLineCrossPid,
        LinearVelocity maxVelocity,
        LinearAcceleration maxAcceleration,
        AngularVelocity maxAngularVelocity,
        AngularAcceleration maxAngularAcceleration,
        Distance translationTolerance,
        Angle rotationTolerance,
        Distance intermediateHandOffDistance
    ) {
        Path.setDefaultGlobalConstraints(new Path.DefaultGlobalConstraints(
            maxVelocity.in(MetersPerSecond),
            maxAcceleration.in(MetersPerSecondPerSecond),
            maxAngularVelocity.in(DegreesPerSecond),
            maxAngularAcceleration.in(DegreesPerSecondPerSecond),
            translationTolerance.in(Meters),
            rotationTolerance.in(Degrees),
            intermediateHandOffDistance.in(Meters)
        ));
        
        m_pathBuilder = new FollowPath.Builder(
            drivetrain, 
            () -> drivetrain.getOutputs().m_pose, 
            () -> drivetrain.getOutputs().m_measuredSpeeds,
            speeds -> drivetrain.setInputs(new Swerve.Inputs(speeds, Swerve.State.RobotRelativeDriving)),
            BLineTransitionPid,
            BLineRotationPid,
            BLineCrossPid
        ).withDefaultShouldFlip()
        .withPoseReset(drivetrain::resetPose);

        RobotConfig config;
        try{
            config = RobotConfig.fromGUISettings();
        } catch (Exception e) {
            // Handle exception as needed
            e.printStackTrace();
            Logger.recordOutput("Pathing Initialization ERROR", "PP GUI error");
            return;
        }

        AutoBuilder.configure(
            () -> drivetrain.getOutputs().m_pose, // Robot pose supplier
            drivetrain::resetPose, // Method to reset odometry (will be called if your auto has a starting pose)
            () -> drivetrain.getOutputs().m_measuredSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
            (speeds, feedforwards) -> drivetrain.setInputs(new Swerve.Inputs(speeds, Swerve.State.RobotRelativeDriving)), // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also optionally outputs individual module feedforwards
            PPPid,
            config, // The robot configuration
            Alliance::isRed,
            drivetrain // Reference to this subsystem to set requirements
        );
    }
}