package frc.robot.subsystems.swerve;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Rotations;

import java.util.ArrayList;
import java.util.function.Supplier;

import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.littletonrobotics.junction.Logger;

import frc.robot.Constants;
import frc.robot.lib.Alliance;
import frc.robot.lib.Logged;
import frc.robot.lib.Module;
import frc.robot.lib.VisionMeasurement;

import frc.robot.subsystems.swerveModule.SwerveModule;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Subsystem;

public abstract class Swerve extends Module<Swerve.Inputs, Swerve.Outputs> implements Subsystem
{
	public static enum State
	{
		RobotRelativeDriving,
		FieldRelativeDriving,
		XMode,
		Aiming // By default field relative***
	}

	public static class Inputs
	{
		public ChassisSpeeds   m_speeds  = new ChassisSpeeds();
		public Angle       m_heading = Degrees.of(0);
		public State       m_state   = State.RobotRelativeDriving;
		public ArrayList<VisionMeasurement> m_visionMeasurement = new ArrayList<>();

		public Inputs(ChassisSpeeds speeds, State state, ArrayList<VisionMeasurement> visionMeasurement) {
			m_speeds = speeds;
			m_state  = state;
			m_visionMeasurement = visionMeasurement;
		}

		public Inputs(ChassisSpeeds speeds, State state) {
			m_speeds = speeds;
			m_state  = state;
		}

		public Inputs() {

		}
	}

	public static class Outputs extends Logged
	{
		public Pose2d    m_pose;
		public ChassisSpeeds m_desiredSpeeds;
		public ChassisSpeeds m_measuredSpeeds;
		public boolean     m_isAimed;
		public State     m_state;
		public Angle     m_aimSetPoint;
		public Angle     m_aimAngle;

		public Outputs()
		{
			m_pose       = new Pose2d();
			m_desiredSpeeds  = new ChassisSpeeds();
			m_measuredSpeeds = new ChassisSpeeds();
			m_isAimed    = false;
			m_state      = State.RobotRelativeDriving;
			m_aimSetPoint = Rotations.of(0.0);
			m_aimAngle  = Rotations.of(0.0);
		}

		public Outputs(Pose2d pose, ChassisSpeeds desiredSpeeds, ChassisSpeeds measuredSpeeds, boolean isAimed, State state, Angle aimSetPoint, Angle aimAngle)
		{
			m_pose = pose;
			m_desiredSpeeds = desiredSpeeds;
			m_measuredSpeeds = measuredSpeeds;
			m_isAimed = isAimed;
			m_state  = state;
			m_aimSetPoint = aimSetPoint;
			m_aimAngle  = aimAngle;
		}

		public void log() {
			Logger.recordOutput("Swerve/Measured Pose", m_pose);
			Logger.recordOutput("Swerve/Desired Speeds", m_desiredSpeeds);
			Logger.recordOutput("Swerve/Measured Speeds", m_measuredSpeeds);
			Logger.recordOutput("Swerve/State",  m_state);
			Logger.recordOutput("Swerve/Aim Setpoint",  m_aimSetPoint);
			Logger.recordOutput("Swerve/Aim Error",  m_aimAngle);
		}
	}

	PIDController m_aimController = new PIDController(5.0, 0.0, 0.0);

	// Handle logic between subsystems inputs and hardware inputs
	@Override
	protected void updateHardwareInputs()
	{
		updatePoseEstimator(new Rotation2d(m_inputs.m_heading), getModulePositions());
		for (VisionMeasurement measure : m_inputs.m_visionMeasurement)
			updateVisionInputs(measure);

		// Probably not necessary, but I want to ensure that we aren't going through old measurements
		m_inputs.m_visionMeasurement.clear();

		SwerveModuleState[] swerveModuleStates = null;
		switch (m_inputs.m_state) {
			case RobotRelativeDriving:
				swerveModuleStates = Constants.Chassis.kinematics.toSwerveModuleStates(m_inputs.m_speeds);
				break;

			case FieldRelativeDriving:
				swerveModuleStates = Constants.Chassis.kinematics.toSwerveModuleStates(ChassisSpeeds.fromFieldRelativeSpeeds(
						m_inputs.m_speeds,
						new Rotation2d(m_inputs.m_heading).plus(Alliance.isRed() ? Rotation2d.kPi : new Rotation2d(0))
					));
				break;

			case XMode:
				swerveModuleStates = Constants.Chassis.XishStates;
				break;

			case Aiming:
				// Get the angle between our position and the target via arctan (getAngle)
				Rotation2d angleToTarget = getTargetPose().getTranslation().minus(getPose().getTranslation()).getAngle();

				m_inputs.m_speeds.omegaRadiansPerSecond = m_aimController.calculate(
						m_inputs.m_heading.in(Radians), 
						angleToTarget.getRadians()
				);

				Logger.recordOutput("Swerve/reported target", angleToTarget.getDegrees());

				swerveModuleStates = Constants.Chassis.kinematics.toSwerveModuleStates(ChassisSpeeds.fromFieldRelativeSpeeds(
						m_inputs.m_speeds,
						new Rotation2d(m_inputs.m_heading).plus(Alliance.isRed() ? Rotation2d.kPi : new Rotation2d(0))
					));
				break;

			default:
				swerveModuleStates = new SwerveModuleState[4];
				break;
		}

		m_aimController.calculate(m_inputs.m_heading.in(Radians));

		setModules(new ArrayList<SwerveModule.Inputs>(java.util.Arrays.asList(
			new SwerveModule.Inputs(swerveModuleStates[0]),
			new SwerveModule.Inputs(swerveModuleStates[1]),
			new SwerveModule.Inputs(swerveModuleStates[2]),
			new SwerveModule.Inputs(swerveModuleStates[3])
		)));
	}

	// Update logging struct (m_outputs)
	@Override
	protected void updateOutputs()
	{
		m_outputs = new Outputs(
			getPose(),
			getDesiredSpeeds(),
			getMeasuredSpeeds(),
			isAimed(),
			m_inputs.m_state,
			Radians.of(m_aimController.getSetpoint()),
			Radians.of(m_aimController.getError())
		);
	}

	// For some inexplicable reason, maplesim does this internally
	protected void updatePoseEstimator(Rotation2d gyroHeading, SwerveModulePosition[] modulePositions) {}
	abstract void updateVisionInputs(VisionMeasurement measurement);

	abstract protected void setModules(ArrayList<SwerveModule.Inputs> inputs);

	protected ChassisSpeeds getDesiredSpeeds() { return m_inputs.m_speeds; }
	abstract protected ChassisSpeeds getMeasuredSpeeds();
	abstract public SwerveModulePosition[] getModulePositions();

	abstract public SwerveModuleState[] getModuleStates();

	abstract protected Pose2d getPose();
	abstract public void resetPose(Pose2d pose); // Needs to be public for PP and BLine

	protected boolean isAimed() {
		return m_aimController.atSetpoint();
	}

	public Supplier<Rotation2d>    getSimGyro() { return null; }
	public SwerveDriveSimulation getSimSwerve() { return null; }

	public Pose2d getTargetPose()
	{
		return Alliance.isRed() ?
			Constants.Field.RedHub.toPose2d() :
			Constants.Field.BlueHub.toPose2d();
	}
}
