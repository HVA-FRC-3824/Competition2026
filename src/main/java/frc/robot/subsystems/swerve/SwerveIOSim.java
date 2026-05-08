package frc.robot.subsystems.swerve;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Pounds;

import java.util.ArrayList;
import java.util.function.Supplier;

import org.ironmaple.simulation.drivesims.COTS;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import frc.robot.Constants;
import frc.robot.lib.VisionData;
import frc.robot.subsystems.swerveModule.SwerveModule;
import frc.robot.subsystems.swerveModule.SwerveModuleIOSim;

public class SwerveIOSim implements SwerveIO {

  private final SwerveModule m_frSwerveModules;
  private final SwerveModule m_flSwerveModules;
  private final SwerveModule m_brSwerveModules;
  private final SwerveModule m_blSwerveModules;

  private final DriveTrainSimulationConfig driveTrainConfig = DriveTrainSimulationConfig.Default()
    .withCustomModuleTranslations(Constants.Chassis.ModulePositions)
    .withRobotMass(Pounds.of(115.0))
    .withGyro(COTS.ofPigeon2())
    .withSwerveModule(COTS.ofMark4i(
        DCMotor.getKrakenX60(1),
        DCMotor.getKrakenX44(1),
        COTS.WHEELS.COLSONS.cof,
        2))
    .withTrackLengthTrackWidth(Inches.of(24), Inches.of(30))
    .withBumperSize(Inches.of(32), Inches.of(38));

  private final SwerveDriveSimulation m_swerveDrive = new SwerveDriveSimulation(driveTrainConfig, new Pose2d(1.0, 1.0, new Rotation2d()));

  public SwerveIOSim() {
    
    m_frSwerveModules = new SwerveModule(new SwerveModuleIOSim(
      0, m_swerveDrive.getModules()[0],
      Constants.CanIds.FrontRightDriveId, 
      Constants.CanIds.FrontRightTurnId, 
      Constants.CanIds.FrontRightEncoderId));

    m_flSwerveModules = new SwerveModule(new SwerveModuleIOSim(
      1, m_swerveDrive.getModules()[1],
      Constants.CanIds.FrontLeftDriveId,  
      Constants.CanIds.FrontLeftTurnId,  
      Constants.CanIds.FrontLeftEncoderId));

    m_brSwerveModules = new SwerveModule(new SwerveModuleIOSim(
      2, m_swerveDrive.getModules()[2],
      Constants.CanIds.BackRightDriveId,  
      Constants.CanIds.BackRightTurnId,  
      Constants.CanIds.BackRightEncoderId));

    m_blSwerveModules = new SwerveModule(new SwerveModuleIOSim(
      3, m_swerveDrive.getModules()[3],
      Constants.CanIds.BackLeftDriveId,   
      Constants.CanIds.BackLeftTurnId,   
      Constants.CanIds.BackLeftEncoderId));
  }
  
	public void resetSwerveModules() {
    
  }

  @Override
  public Supplier<Rotation2d> getSimGyro() {
    return () -> m_swerveDrive.getGyroSimulation().getGyroReading();
  }

  @Override
  public SwerveDriveSimulation getSimSwerve() {
    return m_swerveDrive;
  }

  @Override
  public void updateVisionInputs(VisionData measurement) {
    
  } // TODO: do

  @Override
  public void setModules(ArrayList<SwerveModuleState> inputs) {

    m_frSwerveModules.drive(inputs.get(0));
    m_flSwerveModules.drive(inputs.get(1));
    m_brSwerveModules.drive(inputs.get(2));
    m_blSwerveModules.drive(inputs.get(3));
  }

  @Override
  public ChassisSpeeds getMeasuredSpeeds() {
    return m_swerveDrive.getDriveTrainSimulatedChassisSpeedsRobotRelative();
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
  public Pose2d getPose() {
    return m_swerveDrive.getSimulatedDriveTrainPose();
  }

  @Override
  public void resetPose(Pose2d pose) {
    m_swerveDrive.setSimulationWorldPose(pose);
  }
  
}
