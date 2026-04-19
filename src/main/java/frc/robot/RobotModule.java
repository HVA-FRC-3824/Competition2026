package frc.robot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecondPerSecond;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.numbers.N0;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.lib.pathing.CustomPathing;
import frc.robot.lib.pathing.Pathing;
import frc.robot.lib.Logged;
import frc.robot.lib.Module;

public class RobotModule extends Module<N0, Logged> {

  private final RobotState   m_state   = new RobotState(); 
  private RobotState.Inputs  m_inputs  = new RobotState.Inputs();
  private RobotState.Outputs m_outputs = new RobotState.Outputs();

  private final SendableChooser<Command> m_autoChooser;
  private final CommandXboxController m_driver   = new CommandXboxController(0);
  // private final CommandXboxController m_operator = new CommandXboxController(1);
  
  private final Pathing pathBuilder = new Pathing(
    m_state.getSwerveRefForPathing(),
    new PPHolonomicDriveController(
      new PIDConstants(1.0, 0.0, 0.0),
      new PIDConstants(1.0, 0.0, 0.0)
    ),
    new PIDController(5.0, 0.0, 0.0), // BLineTransitionPid,
    new PIDController(3.0, 0.0, 0.0), // BLineRotationPid,
    new PIDController(2.0, 0.0, 0.0), // BLineCrossPid I dont know what this does
    Constants.Chassis.MaximumLinear,
    Constants.Chassis.MaximumLinearAcceleration,
    Constants.Chassis.MaximumAngularVelocity,
    Constants.Chassis.MaximumAngularAcceleration,
    Inches.of(4.0),
    Degrees.of(2.0),
    Meters.of(0.25) 
  );

  private final CustomPathing customPathBuilder = new CustomPathing(
    new ProfiledPIDController(3.0, 0, 0, 
      new Constraints(Constants.Chassis.MaximumLinear.in(MetersPerSecond), 
                      Constants.Chassis.MaximumLinearAcceleration.in(MetersPerSecondPerSecond))),
    new ProfiledPIDController(2.0, 0, 0,
      new Constraints(Constants.Chassis.MaximumAngularVelocity.in(RadiansPerSecond), 
                      Constants.Chassis.MaximumAngularAcceleration.in(RadiansPerSecondPerSecond))),
    Inches.of(1.0),
    Degrees.of(0.5),
    Meters.of(0.1),  // new interpolation point every 0.1 meter
    Meters.of(0.5)
  );

  private final CustomPathing.Controller controller = customPathBuilder.fromWaypoints(new Pose2d[] {new Pose2d(1.0, 1.0, new Rotation2d()), new Pose2d(1.0, 2.0, new Rotation2d())});

  public RobotModule() {

    m_autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", m_autoChooser);

  }

  @Override
  public void updateHardwareInputs() {
    m_outputs = m_state.getOutputs();

    if (DriverStation.isTeleopEnabled())
    {
      m_inputs.m_leftX  = -m_driver.getLeftX();
      m_inputs.m_leftY  = -m_driver.getLeftY();
      m_inputs.m_rightX = -m_driver.getRightX();
    } else if (DriverStation.isAutonomousEnabled()){
      // Pathplanning stuff here
      m_inputs.m_customPathingSpeeds = controller.getSpeeds(m_outputs.m_pose);
      m_inputs.m_state = RobotState.State.JacksonsCustomPathModeOfDoomAndDespair;
    }

    m_state.setInputs(m_inputs);

    m_state.modulePeriodic();
  }

  @Override 
  public void updateOutputs() {

  }

  public Command getAutonomousCommand() 
  {
    return m_autoChooser.getSelected();
  }
}
