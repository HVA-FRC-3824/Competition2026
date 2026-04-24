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
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;
import frc.robot.lib.VisionMeasurement;
import frc.robot.subsystems.swerveModule.SwerveModule;

public class SwerveSimpleSim extends Swerve {
  
  private final SelfControlledSwerveDriveSimulation m_simulatedDrive;

  public SwerveSimpleSim() 
  {
    m_inputs = new Inputs();
    m_outputs = new Outputs();
    
    m_aimController.setTolerance(Units.degreesToRadians(5.0));
    m_aimController.enableContinuousInput(-Math.PI, Math.PI);

    // For your own code, please configure your drivetrain properly according to the documentation
    final DriveTrainSimulationConfig driveTrainConfig = DriveTrainSimulationConfig.Default()
        // Specify gyro type (for realistic gyro drifting and error simulation)
        .withGyro(COTS.ofPigeon2())
        // Specify swerve module (for realistic swerve dynamics)
        .withSwerveModule(COTS.ofMark4i(
            DCMotor.getKrakenX60(1), // Drive motor is a Kraken X60
            DCMotor.getKrakenX44(1), // Steer motor is a Falcon 500
            COTS.WHEELS.COLSONS.cof, // Use the COF for Colson Wheels
            2)) // L3 Gear ratio
        // Configures the track length and track width (spacing between swerve modules)
        .withTrackLengthTrackWidth(Inches.of(24), Inches.of(30))
        // Configures the bumper size (dimensions of the robot bumper)
        .withBumperSize(Inches.of(32), Inches.of(38));

    // Creating the SelfControlledSwerveDriveSimulation instance
    m_simulatedDrive = new SelfControlledSwerveDriveSimulation(
        new SwerveDriveSimulation(driveTrainConfig, new Pose2d(1.0, 1.0, new Rotation2d())));

    // Register the drivetrain simulation to the simulation world
    SimulatedArena.getInstance().addDriveTrainSimulation(m_simulatedDrive.getDriveTrainSimulation());
  }

  public void addVisionMeasurement(Pose2d visionMeasurement, double timestampSeconds, Matrix<N3, N1> stdDevs)
  {
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
  protected void setModules(ArrayList<SwerveModule.Inputs> inputs) {
    // update the odometry of the SimplifedSwerveSimulation instance
    m_simulatedDrive.periodic();

    // send simulation data to dashboard for testing
    Logger.recordOutput("Simulation/Actual Pose", m_simulatedDrive.getActualPoseInSimulationWorld());
    Logger.recordOutput("Simulation/Measured Pose", getPose());

    m_simulatedDrive.runChassisSpeeds(
      Constants.Chassis.kinematics.toChassisSpeeds(new SwerveModuleState[] {
        inputs.get(0).m_moduleState,
        inputs.get(1).m_moduleState,
        inputs.get(2).m_moduleState,
        inputs.get(3).m_moduleState
      }),
      new Translation2d(),
      false,
      true);
  }

  @Override
  protected ChassisSpeeds getMeasuredSpeeds() {
    return m_simulatedDrive.getActualSpeedsRobotRelative();
  }

  @Override
  protected void updateVisionInputs(VisionMeasurement measurement) {
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
  protected Pose2d getPose() {
    return m_simulatedDrive.getOdometryEstimatedPose();
  }

  @Override
  public void resetPose(Pose2d pose) {
    m_simulatedDrive.resetOdometry(pose);
    m_simulatedDrive.setSimulationWorldPose(pose);
  }
}