package frc.robot.subsystems.swerve;

import java.util.ArrayList;
import java.util.function.Supplier;

import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.lib.VisionData;

public interface SwerveIO {
    	// For some inexplicable reason, maplesim does this internally
	public default void update() {}
	public default void updatePoseEstimator(Rotation2d gyroHeading, SwerveModulePosition[] modulePositions) {}
	public void updateVisionInputs(VisionData measurement);

	public void setModules(ArrayList<SwerveModuleState> inputs);

	public ChassisSpeeds getMeasuredSpeeds();

	SwerveModulePosition[] getModulePositions();
	public SwerveModuleState[] getModuleStates();

	public void resetSwerveModules();

	public Pose2d getPose();
	public void resetPose(Pose2d pose); // Needs to be public for PP and BLine

	public default Supplier<Rotation2d>    getSimGyro() { return null; }
	public default SwerveDriveSimulation getSimSwerve() { return null; }
}
