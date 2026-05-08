package frc.robot.subsystems.swerve;

import java.util.ArrayList;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants;
import frc.robot.lib.VisionData;
import frc.robot.subsystems.swerveModule.SwerveModule;
import frc.robot.subsystems.swerveModule.SwerveModuleIOTalonFx;

/// @brief Chassis subsystem for swerve drive control
///
///     Red            <----- Zero Angle             Blue
///              <--- 0 degrees  180 degrees ---->   X  <-----
///   ---  +-------------------------------------------------------------------+ (0, 0)
///  ^   |        7  6        |       17 28       29 |  
///  |   |                  |               30 |  |
///  |   |                  |                |  |
///  |   |                  |                |  V
///  |   |                  |                |
///  |   |        8  5        |       18 27        |  Y
/// 8.07 m | 16      9     4      |      19     26    31 |
///  |   | 15     10     3      |      20     25    32 |
///  |   |         11  2        |       21 24        |
///  |   |                  |                |
///  |   |                  |                |
///  |   | 14                 |                |
///  V   | 13      12  1        |       22 23        |
///   ---  +-------------------------------------------------------------------+
///    |<----------------------------- 16.56 m ----------------------------->|
///                     Top View
public class SwerveIOReal implements SwerveIO
{
  // Swerve module order for kinematics calculations
  //
  //     Front      Translation2d Coordinates
  //   FL +----------+ FR        ^ X
  //    | 0    1 |         |
  //    |      |      Y  |
  //    |      |      <------+-------
  //    | 2    3 |         |
  //   BL +----------+ BR        |
  

  private SwerveDrivePoseEstimator m_poseEstimator = new SwerveDrivePoseEstimator(
    Constants.Chassis.Kinematics,
    new Rotation2d(0),  // Initial gyro angle
    new SwerveModulePosition[]{new SwerveModulePosition(0, new Rotation2d(0)), new SwerveModulePosition(0, new Rotation2d(0)), new SwerveModulePosition(0, new Rotation2d(0)), new SwerveModulePosition(0, new Rotation2d(0))},   // Initial module positions
    new Pose2d()        // Initial pose
  );

  private SwerveModule m_frSwerveModules;
  private SwerveModule m_flSwerveModules;
  private SwerveModule m_brSwerveModules;
  private SwerveModule m_blSwerveModules;

  public SwerveIOReal() {

    m_flSwerveModules = new SwerveModule(new SwerveModuleIOTalonFx(
      0,
      Constants.CanIds.FrontLeftDriveId,  
      Constants.CanIds.FrontLeftTurnId,  
      Constants.CanIds.FrontLeftEncoderId,
      Constants.Chassis.FrontLeftForwardsAngle));

    m_frSwerveModules = new SwerveModule(new SwerveModuleIOTalonFx(
      1,
      Constants.CanIds.FrontRightDriveId, 
      Constants.CanIds.FrontRightTurnId, 
      Constants.CanIds.FrontRightEncoderId,
      Constants.Chassis.FrontRightForwardsAngle));

    m_blSwerveModules = new SwerveModule(new SwerveModuleIOTalonFx(
      2,
      Constants.CanIds.BackLeftDriveId,   
      Constants.CanIds.BackLeftTurnId,   
      Constants.CanIds.BackLeftEncoderId,
      Constants.Chassis.BackLeftForwardsAngle));

    m_brSwerveModules = new SwerveModule(new SwerveModuleIOTalonFx(
      3,
      Constants.CanIds.BackRightDriveId,  
      Constants.CanIds.BackRightTurnId,  
      Constants.CanIds.BackRightEncoderId,
      Constants.Chassis.BackRightForwardsAngle));
  }
  
  @Override
  public void update() {
    m_flSwerveModules.updateOutputs();
    m_frSwerveModules.updateOutputs();
    m_blSwerveModules.updateOutputs();
    m_brSwerveModules.updateOutputs();
  }

	public void resetSwerveModules() {
    m_flSwerveModules.resetSwerveModules();
    m_frSwerveModules.resetSwerveModules();
    m_blSwerveModules.resetSwerveModules();
    m_brSwerveModules.resetSwerveModules();
  }

  @Override
  public ChassisSpeeds getMeasuredSpeeds() {
    return Constants.Chassis.Kinematics.toChassisSpeeds(getModuleStates());
  }

  @Override
  public void updatePoseEstimator(Rotation2d gyroHeading, SwerveModulePosition[] modulePositions) {
    m_poseEstimator.update(gyroHeading, modulePositions);
  }

  @Override
  public void updateVisionInputs(VisionData measurement) {
    m_poseEstimator.addVisionMeasurement(
      measurement.visionMeasurement(), 
      measurement.timestampSeconds(), 
      measurement.stdDevs()
    );
  }

  @Override
  public void setModules(ArrayList<SwerveModuleState> inputs)
  {
    // Set the desired state for each swerve module
    m_flSwerveModules.drive(inputs.get(0));
    m_frSwerveModules.drive(inputs.get(1));
    m_blSwerveModules.drive(inputs.get(2));
    m_brSwerveModules.drive(inputs.get(3));
  }

  @Override
  public void resetPose(Pose2d pose)
  {
    m_poseEstimator.resetPose(pose);
  }

  @Override
  public SwerveModuleState[] getModuleStates()
  {
    SwerveModuleState[] states = {
      m_flSwerveModules.getOutputs().measuredSwerveModuleState(),
      m_frSwerveModules.getOutputs().measuredSwerveModuleState(),
      m_blSwerveModules.getOutputs().measuredSwerveModuleState(),
      m_brSwerveModules.getOutputs().measuredSwerveModuleState()
    };

    return states;
  }

  @Override
  public SwerveModulePosition[] getModulePositions()
  {
    SwerveModulePosition[] positions = {
      m_flSwerveModules.getOutputs().swerveModulePosition(),
      m_frSwerveModules.getOutputs().swerveModulePosition(),
      m_blSwerveModules.getOutputs().swerveModulePosition(),
      m_brSwerveModules.getOutputs().swerveModulePosition()
    };

    return positions;
  }

  @Override
  public Pose2d getPose()
  {
    return m_poseEstimator.getEstimatedPosition();
  }
};