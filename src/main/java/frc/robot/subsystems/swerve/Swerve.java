package frc.robot.subsystems.swerve;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Rotations;

import java.util.ArrayList;
import java.util.List;
import java.util.function.Supplier;

import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import frc.robot.Constants;
import frc.robot.lib.Alliance;
import frc.robot.lib.Module;
import frc.robot.lib.Module.Logged;
import frc.robot.lib.VisionData;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;

public class Swerve extends Module<Swerve.Outputs> {

	private SwerveIO m_io;

	private PIDController m_aimController = new PIDController(3.0, 0.0, 0.001);
	
	private SensorData m_data = new SensorData(Degrees.of(0.0), new ArrayList<>());

	private ChassisSpeeds m_desiredSpeeds = new ChassisSpeeds();

	public Swerve(SwerveIO io) {

		m_io = io;

		m_outputs = Outputs.zeroed();
		
		m_aimController.setTolerance(Units.degreesToRadians(2.0));
		m_aimController.enableContinuousInput(-Math.PI, Math.PI);

		RobotConfig config;
		try {
			config = RobotConfig.fromGUISettings();
		} catch (Exception e) {
			// If you see this, diagnose the issue and redeploy
			e.printStackTrace();
			Logger.recordOutput("Pathing Initialization ERROR", e.toString());

			// This should never be the case
			config = new RobotConfig(30, 5, new ModuleConfig(Constants.Chassis.WheelCircumference.in(Meters) / (2*Math.PI), 2, 1.0, DCMotor.getKrakenX60(1), 85, 1), 30);
		}

		// Access the drivetrain subsystem directly
		AutoBuilder.configure(
			() -> m_io.getPose(), 
			m_io::resetPose,
			m_io::getMeasuredSpeeds, 
			(speeds, ff) -> {
				SwerveModuleState[] states = Constants.Chassis.Kinematics.toSwerveModuleStates(speeds);
				SwerveDriveKinematics.desaturateWheelSpeeds(states, Constants.Chassis.MaximumLinear.in(MetersPerSecond));
				m_io.setModules(new ArrayList<SwerveModuleState>(List.of(states)));
			},
			new PPHolonomicDriveController(
						new PIDConstants(2.0, 0.0, 0.0),
						new PIDConstants(2.0, 0.0, 0.0)
					),
			config,
			Alliance::isRed,
			this
		);
	}

	public void updateSensorData(SensorData data) {

		m_data = data;
	}

	
	@Override
	public void updateOutputs() {

		m_io.update();
		
		m_io.updatePoseEstimator(new Rotation2d(m_data.heading()), m_io.getModulePositions());
		for (VisionData measure : m_data.visionMeasurement()) {	
			m_io.updateVisionInputs(measure);
		}

		// Probably not necessary, but I want to ensure that we aren't going through old measurements
		m_data.visionMeasurement().clear();

		m_aimController.calculate(m_data.heading.in(Radians));
		
		m_outputs = new Outputs(
			m_io.getPose(),
			m_desiredSpeeds,
			m_io.getMeasuredSpeeds(),
			m_aimController.atSetpoint(),
			Radians.of(m_aimController.getSetpoint()),
			Radians.of(m_aimController.getError())
		);
	}

	public Command resetSwerveModules() {
		return runOnce(() -> m_io.resetSwerveModules()).withName("resetSwerveModules");
	}

	public Command fieldCentricDrive(Supplier<ChassisSpeeds> speedsSupplier, boolean fieldCentric) {

		return run(() -> {
			drive(fieldCentric 
				? ChassisSpeeds.fromFieldRelativeSpeeds(
					speedsSupplier.get(), 
					new Rotation2d(m_data.heading()).plus(Alliance.isRed() ? Rotation2d.kPi : new Rotation2d(0)))
				: speedsSupplier.get());
		}).withName("Drive");
	}

	public Command xMode() {
		return runOnce(() -> m_io.setModules(Constants.Chassis.XishStates)).withName("XMode");
	}

	public Command aim(AimTarget target, Supplier<ChassisSpeeds> speedsSupplier) {

		return run(() -> {
			ChassisSpeeds speeds = speedsSupplier.get();

			// Get the angle between our position and the target via arctan (getAngle)
			Rotation2d angleToTarget = getTargetPos(target).getTranslation().getAngle();

			speeds.omegaRadiansPerSecond = m_aimController.calculate(
					m_data.heading().in(Radians),
					angleToTarget.getRadians()
			);

			drive(ChassisSpeeds.fromFieldRelativeSpeeds(
				speeds,
				new Rotation2d(m_data.heading()).plus(Alliance.isRed() ? Rotation2d.kPi : new Rotation2d(0))
			));
		}).withName("Aim at " + target.toString());
	}

	public Command aimPose(Supplier<Pose2d> target, Supplier<ChassisSpeeds> speedsSupplier, String targetName) {

		return run(() -> {
			ChassisSpeeds speeds = speedsSupplier.get();

			// Get the angle between our position and the target via arctan (getAngle)
			Rotation2d angleToTarget = target.get().getTranslation().getAngle();

			speeds.omegaRadiansPerSecond = m_aimController.calculate(
					m_data.heading().in(Radians),
					angleToTarget.getRadians()
			);

			drive(ChassisSpeeds.fromFieldRelativeSpeeds(
				speeds,
				new Rotation2d(m_data.heading()).plus(Alliance.isRed() ? Rotation2d.kPi : new Rotation2d(0))
			));
		}).withName("Aim at " + target.toString());
	}
	
	public Command aimPose(Supplier<Pose2d> target, Supplier<ChassisSpeeds> speedsSupplier) {
		
		return aimPose(target, speedsSupplier, target.get().toString());
	}

	public void drive(ChassisSpeeds speeds) {

		m_desiredSpeeds = speeds;
		
		SwerveModuleState[] states = Constants.Chassis.Kinematics.toSwerveModuleStates(speeds);
		// SwerveDriveKinematics.desaturateWheelSpeeds(states, Constants.Chassis.MaximumLinear.in(MetersPerSecond));
		m_io.setModules(new ArrayList<SwerveModuleState>(List.of(states)));
	}

	public Supplier<Rotation2d> getSimGyro() {
		return m_io.getSimGyro();
	}
	public SwerveDriveSimulation getSimSwerve() {
		return m_io.getSimSwerve();
	}

	public Pose2d getTargetPos(AimTarget target) {

		switch (target) {
			case Score:
				return (Alliance.isRed() ?
					Constants.Field.RedHub.toPose2d() :
					Constants.Field.BlueHub.toPose2d()
				);

			case Passing:
				return (Alliance.isRed() ?
					m_outputs.pose.nearest(List.of(Constants.Field.RedAllianceZoneClose, Constants.Field.RedAllianceZoneFar)) :
					m_outputs.pose.nearest(List.of(Constants.Field.BlueAllianceZoneClose, Constants.Field.BlueAllianceZoneFar))
				);

				
			case Down:
				return new Pose2d(Meters.of(0.0), Meters.of(2.0), new Rotation2d()).rotateBy(Rotation2d.kCCW_90deg);
			
			case Left:
				return new Pose2d(Meters.of(0.0), Meters.of(2.0), new Rotation2d()).rotateBy(Rotation2d.kZero);
			
			case Right:
				return new Pose2d(Meters.of(0.0), Meters.of(2.0), new Rotation2d()).rotateBy(Rotation2d.k180deg);
			
			case Up:
				return new Pose2d(Meters.of(0.0), Meters.of(2.0), new Rotation2d()).rotateBy(Rotation2d.kCW_90deg);

			default:
				return new Pose2d();
		}
	}

	public static enum AimTarget {
		Score,
		Passing,
		Up,
		Down,
		Left,
		Right 
	}

	public static record SensorData(
		Angle         			     heading,
		ArrayList<VisionData> visionMeasurement
	) {

	}

	public static record Outputs(
		Pose2d  pose,    
		ChassisSpeeds desiredSpeeds, 
		ChassisSpeeds measuredSpeeds,
		boolean isAimed, 
		Angle   aimSetPoint, 
		Angle   aimAngle
	) implements Logged {

		public static Outputs zeroed() {
			return new Outputs(
				new Pose2d(),
				new ChassisSpeeds(),
				new ChassisSpeeds(),
				false,
				Rotations.of(0.0),
				Rotations.of(0.0));
		}

		@Override
		public void log() {
			Logger.recordOutput("Swerve/Measured Pose", pose);
			Logger.recordOutput("Swerve/Desired Speeds", desiredSpeeds);
			Logger.recordOutput("Swerve/Measured Speeds", measuredSpeeds);
			Logger.recordOutput("Swerve/Aim Setpoint",  aimSetPoint);
			Logger.recordOutput("Swerve/Aim Error",  aimAngle);
		}
	}
}
