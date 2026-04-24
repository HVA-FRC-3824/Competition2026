package frc.robot.lib.pathing;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;

import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.lib.Alliance;
import frc.robot.lib.BLine.FollowPath;
import frc.robot.lib.BLine.Path;
import frc.robot.subsystems.swerve.Swerve;

public class Pathing {
  
  // private final SendableChooser<Command> m_autoChooser;

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
  }

  
  // public Command getAutonomousCommand() 
  // {
  //   return m_autoChooser.getSelected();
  // }
}