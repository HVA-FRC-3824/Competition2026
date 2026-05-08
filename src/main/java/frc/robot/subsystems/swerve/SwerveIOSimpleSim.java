package frc.robot.subsystems.swerve;

import static edu.wpi.first.units.Units.Inches;

import java.util.ArrayList;
import java.util.function.Supplier;

import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.COTS;
import org.ironmaple.simulation.drivesims.SelfControlledSwerveDriveSimulation;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.system.plant.DCMotor;
import frc.robot.Constants;
import frc.robot.lib.VisionData;

public class SwerveIOSimpleSim implements SwerveIO {
  
  private final SelfControlledSwerveDriveSimulation m_simulatedDrive;

  public SwerveIOSimpleSim() {

    final DriveTrainSimulationConfig driveTrainConfig = DriveTrainSimulationConfig.Default()
        .withGyro(COTS.ofPigeon2())
        .withSwerveModule(COTS.ofMark4i(
            DCMotor.getKrakenX60(1),
            DCMotor.getKrakenX44(1),
            COTS.WHEELS.COLSONS.cof,
            2))
        .withTrackLengthTrackWidth(Inches.of(24), Inches.of(30))
        .withBumperSize(Inches.of(32), Inches.of(38));

    // Creating the SelfControlledSwerveDriveSimulation instance
    m_simulatedDrive = new SelfControlledSwerveDriveSimulation(
        new SwerveDriveSimulation(driveTrainConfig, new Pose2d(1.0, 1.0, new Rotation2d())));

    // Register the drivetrain simulation to the simulation world
    SimulatedArena.getInstance().addDriveTrainSimulation(m_simulatedDrive.getDriveTrainSimulation());
  }

  public void resetSwerveModules() {
    
  }

  public void addVisionMeasurement(Pose2d visionMeasurement, double timestampSeconds, Matrix<N3, N1> stdDevs) {
    // Logging here is anti-idomatic but whatever, you're a nerd
    Logger.recordOutput("Measured/Vision Measurement", visionMeasurement);
    m_simulatedDrive.addVisionEstimation(visionMeasurement, timestampSeconds, stdDevs);
  }

  @Override
  public SwerveDriveSimulation getSimSwerve() {
    return m_simulatedDrive.getDriveTrainSimulation();
  }

  @Override
  public Supplier<Rotation2d> getSimGyro() {
    return () -> m_simulatedDrive.getActualPoseInSimulationWorld().getRotation();
  }

  @Override
  public void setModules(ArrayList<SwerveModuleState> inputs) {
    // send simulation data to dashboard for testing
    Logger.recordOutput("Simulation/Actual Pose", m_simulatedDrive.getActualPoseInSimulationWorld());
    Logger.recordOutput("Simulation/Measured Pose", getPose());

    m_simulatedDrive.runChassisSpeeds(
      Constants.Chassis.Kinematics.toChassisSpeeds(new SwerveModuleState[] {
        inputs.get(0),
        inputs.get(1),
        inputs.get(2),
        inputs.get(3)
      }),
      new Translation2d(),
      false,
      true);
  }

  @Override
  public ChassisSpeeds getMeasuredSpeeds() {
    return m_simulatedDrive.getActualSpeedsRobotRelative();
  }

  @Override
  public void updateVisionInputs(VisionData measurement) {
    m_simulatedDrive.addVisionEstimation(measurement.visionMeasurement(), measurement.timestampSeconds(), measurement.stdDevs());
  }

  @Override
  public SwerveModuleState[] getModuleStates()
  {
    return m_simulatedDrive.getMeasuredStates();
  }

  @Override
  public SwerveModulePosition[] getModulePositions()
  {
    return m_simulatedDrive.getLatestModulePositions();
  }

  @Override
  public Pose2d getPose() {

    return m_simulatedDrive.getOdometryEstimatedPose();
  }

  @Override
  public void resetPose(Pose2d pose) {
    m_simulatedDrive.resetOdometry(pose);
    m_simulatedDrive.setSimulationWorldPose(pose);
  }
}