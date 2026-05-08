package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.lib.Util;
import frc.robot.lib.motor.OrchestraOrchestrator;
import frc.robot.lib.motor.OrchestraOrchestrator.Song;
import frc.robot.subsystems.SubsystemLayer;
import frc.robot.subsystems.swerve.Swerve.AimTarget;

// This is analogus to RobotContainer
public class ControlLayer {

	private SubsystemLayer m_subsystems = new SubsystemLayer(null);

	private final CommandXboxController m_driver   = new CommandXboxController(0);
	private final CommandXboxController m_operator = new CommandXboxController(1);

	private final SendableChooser<Command> m_autoChooser;

	private final SendableChooser<SubsystemLayer.Robot> m_robotChooser = new SendableChooser<>();

	private final Command m_createNewRobot = m_subsystems.run(() -> m_subsystems = new SubsystemLayer(m_robotChooser.getSelected()))
		.withName("Create new robot");

	private final SendableChooser<OrchestraOrchestrator.Song> m_songChooser = new SendableChooser<>();

	private final Command m_playSong = m_subsystems.run(() -> OrchestraOrchestrator.playSong(m_songChooser.getSelected()))
		.withName("Play Song");

	public ControlLayer() {
		
		// Pathplanner Autonomouse Chooser

		m_autoChooser = AutoBuilder.buildAutoChooser();
		SmartDashboard.putData("Auto Chooser", m_autoChooser);

		// Dynamic Robot Chooser

		m_robotChooser.setDefaultOption("Default", null);
		m_robotChooser.addOption("Tricerabot", SubsystemLayer.Robot.Tricerabot);
		m_robotChooser.addOption("Dev", SubsystemLayer.Robot.DevBot);
		m_robotChooser.addOption("Sim", SubsystemLayer.Robot.Sim);

		SmartDashboard.putData(m_robotChooser);
		SmartDashboard.putData(m_createNewRobot);

		// Song Player & Chooser

		m_songChooser.setDefaultOption("Tetris", Song.Tetris);
		m_songChooser.addOption("Poofs", Song.Poofs);
		m_songChooser.addOption("Cynthia", Song.Cynthia);
		m_songChooser.addOption("GymLeader", Song.GymLeader);
		m_songChooser.addOption("Birthday", Song.Birthday);
		m_songChooser.addOption("Pirates", Song.Pirates);
		m_songChooser.addOption("Song2", Song.Song2);
		m_songChooser.addOption("UnderTheSea", Song.UnderTheSea);

		SmartDashboard.putData(m_songChooser);
		SmartDashboard.putData(m_playSong);

		// Pathplanner Commands

		NamedCommands.registerCommand("idle", m_subsystems.idle());
		NamedCommands.registerCommand("unstuck", m_subsystems.unstuck());
		NamedCommands.registerCommand("intake", m_subsystems.intake());
		NamedCommands.registerCommand("stopRoller", m_subsystems.stopRoller());
		NamedCommands.registerCommand("shootPassing", m_subsystems.aimAndShoot(ChassisSpeeds::new, AimTarget.Passing));
		NamedCommands.registerCommand("shootHub", m_subsystems.aimAndShoot(ChassisSpeeds::new, AimTarget.Score));

		// Default behaviors

		m_subsystems.m_swerve.setDefaultCommand(m_subsystems.m_swerve.fieldCentricDrive(this::getSpeeds, true));
		// m_subsystems.m_flywheel.setDefaultCommand(m_subsystems.m_flywheel.off());

		// Controller behaviors

		// m_driver.a().onTrue(m_subsystems.stop());
		// m_driver.b().onTrue(m_subsystems.retract());
		// m_driver.start().onTrue(m_subsystems.unstuck());

		// m_driver.leftTrigger().onTrue(m_subsystems.intake());
		// m_driver.leftTrigger().onFalse(m_subsystems.stopRoller());

		// m_driver.rightBumper().whileTrue(m_subsystems.aimAndShoot(this::getSpeeds, AimTarget.Passing));
		// m_driver.rightTrigger().whileTrue(m_subsystems.aimAndShoot(this::getSpeeds, AimTarget.Score));
		
		m_driver.rightBumper().whileTrue(m_subsystems.followTag());

		m_driver.a().onTrue(m_playSong);

		m_driver.povLeft() .whileTrue(m_subsystems.aimAndDrive(AimTarget.Left));
		m_driver.povRight().whileTrue(m_subsystems.aimAndDrive(AimTarget.Right));
		m_driver.povUp()   .whileTrue(m_subsystems.aimAndDrive(AimTarget.Up));
		m_driver.povDown() .whileTrue(m_subsystems.aimAndDrive(AimTarget.Down));
	}

	
  public ChassisSpeeds getSpeeds() {

    double leftY  = MathUtil.applyDeadband(-m_driver.getLeftY(), 0.09);
    double leftX  = MathUtil.applyDeadband(-m_driver.getLeftX(), 0.09);
    double rightX = MathUtil.applyDeadband(-m_driver.getRightX(),0.09);

	if (leftY == 0.0 && leftX == 0.0 && rightX == 0.0) return new ChassisSpeeds();

	double angle     = Math.atan2(leftY, leftX);
	double magnitude = Math.sqrt(leftY * leftY + leftX * leftX);
	
	magnitude = Util.applyExpo(magnitude, Constants.Chassis.TranslateExponentialPower);

	double strafe   = magnitude * Math.sin(angle);
	double forwards = magnitude * Math.cos(angle);
	double omega    = Util.applyExpo(rightX, Constants.Chassis.AngularExponentialPower);

    return new ChassisSpeeds(
      Constants.Chassis.MaximumLinear.times(strafe), 
      Constants.Chassis.MaximumLinear.times(forwards), 
      Constants.Chassis.MaximumAngularVelocity.times(omega));
  } 

	public Command getAutoCommand() {

		return m_autoChooser.getSelected();
	}

	public Command getSwerveZero() {

		return m_subsystems.resetSwerveModules();
	}
}
