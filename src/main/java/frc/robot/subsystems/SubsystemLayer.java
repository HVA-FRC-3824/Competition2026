package frc.robot.subsystems;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import java.util.ArrayList;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.lib.Module;
import frc.robot.lib.Module.Logged;
import frc.robot.lib.Util;
import frc.robot.lib.VisionData;
import frc.robot.subsystems.flywheel.Flywheel;
import frc.robot.subsystems.flywheel.FlywheelIOSim;
import frc.robot.subsystems.flywheel.FlywheelIOTalonFX;
import frc.robot.subsystems.flywheel.Flywheel.Setpoints;
import frc.robot.subsystems.flywheel.FlywheelIONothing;
import frc.robot.subsystems.gyro.Gyro;
import frc.robot.subsystems.gyro.GyroPigeon;
import frc.robot.subsystems.gyro.GyroSim;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.indexer.IndexerIONothing;
import frc.robot.subsystems.indexer.IndexerSim;
import frc.robot.subsystems.indexer.IndexerTalonFX;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeIONothing;
import frc.robot.subsystems.intake.IntakeSim;
import frc.robot.subsystems.intake.IntakeTalonFX;
import frc.robot.subsystems.roller.Roller;
import frc.robot.subsystems.roller.RollerIONothing;
import frc.robot.subsystems.roller.RollerSim;
import frc.robot.subsystems.roller.RollerTalonFX;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.subsystems.swerve.Swerve.AimTarget;
import frc.robot.subsystems.swerve.SwerveIOReal;
import frc.robot.subsystems.swerve.SwerveIOSimpleSim;
import frc.robot.subsystems.swerve.SwerveIOSummer;
import frc.robot.subsystems.vision.Vision;

// This is similar to RobotState or SubsystemManager
public class SubsystemLayer extends Module<SubsystemLayer.Outputs> {

  // SUBSYSTEM INSTANTATIONS

  public final Swerve m_swerve;
  public final Gyro m_gyro;
  public final Roller m_roller;
  public final Indexer m_indexer;
  public final Intake m_intake;
  public final Flywheel m_flywheel;
  public final Vision m_vision;

  // SUBSYSTEM OUTPUTS

  private Gyro.Outputs     m_gyroOutputs = Gyro.Outputs.zeroed();
  private Swerve.Outputs   m_swerveOutputs = Swerve.Outputs.zeroed();
  private Roller.Outputs   m_rollerOutputs = Roller.Outputs.zeroed();
  private Indexer.Outputs  m_indexerOutputs = Indexer.Outputs.zeroed();
  private Intake.Outputs   m_intakeOutputs = Intake.Outputs.zeroed();
  private Flywheel.Outputs m_flywheelOutputs = Flywheel.Outputs.zeroed();
  public  Vision.Outputs   m_visionOutputs   = Vision.Outputs.zeroed();
  
  public enum Robot {
    Tricerabot,
    DevBot,
    Sim
  }

  public SubsystemLayer(Robot type) {

    m_outputs = new Outputs(new Pose2d());

    if (type == null) type = (RobotBase.isSimulation()) ? Robot.Sim : Robot.DevBot;

    switch (type) {
      case Tricerabot:
        m_swerve   = new Swerve(new SwerveIOReal());
        m_gyro     = new Gyro(new GyroPigeon());
        m_roller   = new Roller(new RollerTalonFX());
        m_indexer  = new Indexer(new IndexerTalonFX());
        m_intake   = new Intake(new IntakeTalonFX());
        m_flywheel = new Flywheel(new FlywheelIOTalonFX());
        m_vision   = new Vision(m_swerve.getSimSwerve());
        break;
    
      case DevBot:
        m_swerve   = new Swerve(new SwerveIOSummer());
        m_gyro     = new Gyro(new GyroPigeon());
        m_roller   = new Roller(new RollerIONothing());
        m_indexer  = new Indexer(new IndexerIONothing());
        m_intake   = new Intake(new IntakeIONothing());
        m_flywheel = new Flywheel(new FlywheelIONothing());
        m_vision   = new Vision(m_swerve.getSimSwerve());
        break;
    
      case Sim:
      default:
        m_swerve   = new Swerve(new SwerveIOSimpleSim());
        m_gyro     = new Gyro(new GyroSim(m_swerve.getSimGyro()));
        m_roller   = new Roller(new RollerSim(m_swerve.getSimSwerve()));
        m_indexer  = new Indexer(new IndexerSim());
        m_intake   = new Intake(new IntakeSim());
        m_flywheel = new Flywheel(new FlywheelIOSim(m_swerve.getSimSwerve(), m_intake.getSimIntake()));
        m_vision   = new Vision(m_swerve.getSimSwerve());
        break;
    }
  }

  @Override
  public void updateOutputs() {

    m_outputs = new Outputs(m_swerveOutputs.pose());

    m_gyroOutputs     = m_gyro.getOutputs();
    m_swerveOutputs   = m_swerve.getOutputs();
    m_rollerOutputs   = m_roller.getOutputs();
    m_indexerOutputs  = m_indexer.getOutputs();
    m_intakeOutputs   = m_intake.getOutputs();
    m_flywheelOutputs = m_flywheel.getOutputs();
    m_visionOutputs   = m_vision.getOutputs();

    // Inter-Subsystem Communication
    m_swerve.updateSensorData(new Swerve.SensorData(m_gyroOutputs.heading(), (ArrayList<VisionData>) m_visionOutputs.measurements().clone())); // May need to clone here
    m_vision.clearData();

    m_flywheel.updateSensorData(new Flywheel.SensorData(
      Util.dist(
        m_swerveOutputs.pose(), 
        m_swerve.getTargetPos(m_flywheel.getTarget())
      )
    ));
  }

  public Command stop() {

    return new ParallelCommandGroup(
      m_roller.off(), 
      m_flywheel.off(), 
      m_indexer.off()
    ).withName("stop");
  }

  public Command unstuck() {

    return new ParallelCommandGroup(
      m_indexer.backwards(), 
      m_roller.backwards(), 
      m_flywheel.set(Setpoints.Backwards)
    ).withName("unstuck");
  }

  public Command intake() {

    return new ParallelCommandGroup(m_roller.on(), m_intake.deployed()).withName("intake");
  }

  public Command retract() {

    return new ParallelCommandGroup(m_roller.off(), m_intake.stowed()).withName("retract");
  }

  public Command stopRoller() {

    return m_roller.off().withName("stop roller");
  }

  public Command followTag() {
    
    return m_swerve.aimPose(() -> m_visionOutputs.lastSeenTag(), ChassisSpeeds::new, "AprilTag");
  }

  public Command aimAndDrive(AimTarget target) {

    return m_swerve.aim(target, 
      () -> {
        Pose2d traj = new Pose2d(1, 0, new Rotation2d()).rotateBy(m_swerve.getTargetPos(target).getTranslation().getAngle());
        return new ChassisSpeeds(
          MetersPerSecond.of(traj.getX()), 
          MetersPerSecond.of(traj.getY()), 
          RadiansPerSecond.of(0.0)
        );
      }
    ).withName("Aim Drive " + target.toString());
  }

  public Command aimAndShoot(Supplier<ChassisSpeeds> speedsSupplier, AimTarget target) {

    return m_swerve.aim(target, speedsSupplier)
      .until(this::GetIsReady)
      .andThen(
        m_swerve.xMode(), 
        m_flywheel.auto(target).alongWith(m_intake.alligator(), m_roller.on()))
      .finallyDo(m_roller.off()::execute).withName("Aim Shoot " + target.toString());
  }

  public Command resetSwerveModules() {

    return m_swerve.resetSwerveModules();
  }

  public boolean GetIsReady() {

    Logger.recordOutput("IsReady/Is SpunUp", m_flywheelOutputs.isSpunUp());
    Logger.recordOutput("IsReady/Is Aimed", m_swerveOutputs.isAimed());

    // If the flywheel is spun up and we're aiming at the target, shoot
    return m_flywheelOutputs.isSpunUp() && 
           m_swerveOutputs.isAimed();
  }

  public static record Outputs(
    Pose2d pose
  ) implements Logged {

    public static Outputs zeroed() {
      return new Outputs(new Pose2d());
    }

    @Override
    public void log() {
      Logger.recordOutput("Pose",  pose);
    }
  }
}
