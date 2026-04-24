package frc.robot;

import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.lib.Alliance;
import frc.robot.lib.Logged;
import frc.robot.lib.Module;
import frc.robot.lib.controls.Controller;
import frc.robot.lib.controls.EdgeController;
import frc.robot.lib.controls.EdgeController.Edge;
import frc.robot.lib.controls.AxisInput;

public class RobotContainer extends Module<RobotContainer.Inputs, RobotContainer.Outputs> {

	public static class Inputs {}

	public static class Outputs extends Logged { @Override public void log() {} }

	private final RobotModule   m_state   = new RobotModule(); 
	private RobotModule.Inputs  m_robotInputs  = new RobotModule.Inputs();
	private RobotModule.Outputs m_robotOutputs = new RobotModule.Outputs();

	private final EdgeController m_driver   = new EdgeController(0);
	private final Controller m_operator = new Controller(1);

	private final SendableChooser<Command> m_autoChooser;

	// Wrapper subsystem to run pathplanner commands
	static PpSubsystem cmdSub = new PpSubsystem();
	private static class PpSubsystem implements Subsystem {
		@Override public void periodic() {}
	}

	public RobotContainer() {
		m_inputs  = new Inputs();
		m_outputs = new Outputs();

		// pathBuilder = new Pathing(
		// 	m_state.getSwerveRefForPathing(),
		// 	new PPHolonomicDriveController(
		// 		new PIDConstants(2.0, 0.0, 0.0),
		// 		new PIDConstants(2.0, 0.0, 0.0)
		// 	),
		// 	new PIDController(5.0, 0.0, 0.0), // BLineTransitionPid,
		// 	new PIDController(3.0, 0.0, 0.0), // BLineRotationPid,
		// 	new PIDController(2.0, 0.0, 0.0), // BLineCrossPid I dont know what this does
		// 	Constants.Chassis.MaximumLinear,
		// 	Constants.Chassis.MaximumLinearAcceleration,
		// 	Constants.Chassis.MaximumAngularVelocity,
		// 	Constants.Chassis.MaximumAngularAcceleration,
		// 	Inches.of(4.0),
		// 	Degrees.of(2.0),
		// 	Meters.of(0.25) 
		// );
		
		// Pathplanner Commands
		NamedCommands.registerCommand("Intake",   cmdSub.runOnce(() -> m_robotInputs.m_isIntaking = true));
		NamedCommands.registerCommand("Intakent", cmdSub.runOnce(() -> m_robotInputs.m_isIntaking = false));
		NamedCommands.registerCommand("AS",     cmdSub.runOnce(() -> m_robotInputs.m_state = RobotModule.State.OffenceStance));
		NamedCommands.registerCommand("DS",     cmdSub.runOnce(() -> m_robotInputs.m_state = RobotModule.State.DefenceStance));
		NamedCommands.registerCommand("IS",     cmdSub.runOnce(() -> m_robotInputs.m_state = RobotModule.State.Idle));
		NamedCommands.registerCommand("JS",     cmdSub.runOnce(() -> m_robotInputs.m_state = RobotModule.State.JogUnstuck));
		NamedCommands.registerCommand("SS",     cmdSub.runOnce(() -> m_robotInputs.m_state = RobotModule.State.AimAndShootToTarget));
		NamedCommands.registerCommand("Jackson's Custom Path Mode Of Doom And Despair Stance", // DO NOT USE
					 cmdSub.runOnce(() -> m_robotInputs.m_state = RobotModule.State.JacksonsCustomPathModeOfDoomAndDespair));

		RobotConfig config;
    try {
      config = RobotConfig.fromGUISettings();
    } catch (Exception e) {
			// If you see this, diagnose the issue and redeploy
      e.printStackTrace();
      Logger.recordOutput("Pathing Initialization ERROR", e.toString());

      // This should never be the case
      config = new RobotConfig(30, 5, new ModuleConfig(Constants.Chassis.WheelCircumference / (2*Math.PI), 2, 1.0, DCMotor.getKrakenX60(1), 85, 0), 30);
    }

	// Access the drivetrain subsystem directly
	Swerve drivetrain = m_state.getSwerveRefForPathing();
    AutoBuilder.configure(
      () -> drivetrain.getOutputs().m_pose, 
      drivetrain::resetPose, 
      () -> drivetrain.getOutputs().m_measuredSpeeds, 
      (speeds, feedforwards) -> drivetrain.setInputs(new Swerve.Inputs(speeds, Swerve.State.RobotRelativeDriving)), // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also optionally outputs individual module feedforwards
      new PPHolonomicDriveController(
				new PIDConstants(2.0, 0.0, 0.0),
				new PIDConstants(2.0, 0.0, 0.0)
			),
      config,
      Alliance::isRed,
      drivetrain
    );
    
    m_autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", m_autoChooser);
	}

	@Override
	protected void updateHardwareInputs() {
		m_robotOutputs = m_state.getOutputs();
		
		if (DriverStation.isTeleopEnabled()) {

			AxisInput driveInputs = m_driver.getAxisInput(Constants.Chassis.TranslateExponentialPower, Constants.Chassis.AngularExponentialPower);
			m_robotInputs.m_leftX  = -driveInputs.leftX();
			m_robotInputs.m_leftY  = -driveInputs.leftY();
			m_robotInputs.m_rightX = -driveInputs.rightX();

			m_driver.a(Edge.Rising, () -> m_robotInputs.m_state = RobotModule.State.DefenceStance);
			m_driver.b(Edge.Rising, () -> m_robotInputs.m_state = RobotModule.State.OffenceStance);
			m_driver.x(Edge.Rising, () -> m_robotInputs.m_state = RobotModule.State.JogUnstuck);
			m_driver.y(Edge.Rising, () -> m_robotInputs.m_state = RobotModule.State.Idle);

			m_driver.leftBumper(Edge.Rising,  () -> m_robotInputs.m_isIntaking = true);
			m_driver.leftBumper(Edge.Falling, () -> m_robotInputs.m_isIntaking = false);
			
			m_driver.rightBumper(Edge.Rising,  () -> m_robotInputs.m_state = RobotModule.State.AimAndShootToTarget);
			m_driver.rightBumper(Edge.Falling, () -> m_robotInputs.m_state = RobotModule.State.OffenceStance);
		}

		m_state.setInputs(m_robotInputs);
		m_state.modulePeriodic();
	}

	@Override 
	protected void updateOutputs() {
	}

	public Command getAutoCommand() {
		return m_autoChooser.getSelected();
	}

	// Run this at the beginning of teleop
	public void resetState() {
		m_robotInputs.m_state = RobotModule.State.Idle;
	}
}
