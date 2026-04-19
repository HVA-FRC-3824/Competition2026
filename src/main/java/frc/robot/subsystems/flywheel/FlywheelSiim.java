package frc.robot.subsystems.flywheel;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.FeetPerSecond;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import org.ironmaple.simulation.IntakeSimulation;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.seasonspecific.rebuilt2026.RebuiltFuelOnFly;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.robot.Constants;
import frc.robot.lib.motor.talonFX.SimpleTalon;

// It shares a name with the wpilibj name FlywheelSim... we fix this
public class FlywheelSiim extends Flywheel
{
    public AngularVelocity m_lastinput = RotationsPerSecond.of(0.0);

    public Angle m_simPosTurns = Rotations.of(0.0);

    final public SimpleTalon m_motor;
    final public FlywheelSim m_motorModel;

    public SwerveDriveSimulation m_simDrive = null;
    public IntakeSimulation      m_intakeSimulation = null;

    public FlywheelSiim(SwerveDriveSimulation simDrive, IntakeSimulation intakeSimulation)
    {
        m_inputs = new Inputs();
        m_outputs = new Outputs();

        m_motor = new SimpleTalon(Constants.CanIds.FlywheelMotorId, Constants.Flywheel.Config, true);

        m_motorModel = new FlywheelSim(
            LinearSystemId.createFlywheelSystem(
                DCMotor.getKrakenX60(2), 
                0.021, 
                1.0),
            DCMotor.getKrakenX60(2),
            0.2);

        m_simDrive         = simDrive;
        m_intakeSimulation = intakeSimulation;
    }

    protected void setFlywheel(AngularVelocity velocity)
    {
        m_motor.setVelocity(velocity);

        m_motorModel.setInputVoltage(m_motor.getAppliedVoltage().in(Volts));
        m_motorModel.update(0.02);

        m_motor.simPeriodic(m_motorModel.getAngularVelocity());

        for (int i = 0; i < m_inputs.m_simIntakeSpeed / 5; i++) {
            if (m_intakeSimulation.obtainGamePieceFromIntake()) { // "isFuel" check
                Distance offset;
                double time = Timer.getTimestamp();
                time = (time - (double) ((int) time));
                if (time >= 0.75) {
                    offset = Inches.of((0.5 / 2) + 0.5);
                } else if (time >= 0.5) {
                    offset = Inches.of((0.5 / 2));
                } else if (time >= 0.25) {
                    offset = Inches.of((-0.5 / 2));
                } else {
                    offset = Inches.of((-0.5 / 2) - 0.5);
                }

                SimulatedArena.getInstance()
                    .addGamePieceProjectile(
                        new RebuiltFuelOnFly(
                            (Translation2d)  m_simDrive.getSimulatedDriveTrainPose().getTranslation(),
                            (Translation2d)  new Translation2d(offset.in(Meters), Units.inchesToMeters(-20)), // shooter offet from center
                            (ChassisSpeeds)  m_simDrive.getDriveTrainSimulatedChassisSpeedsFieldRelative(),
                            (Rotation2d)     m_simDrive.getSimulatedDriveTrainPose().getRotation(),
                            (Distance)       Inches.of(15), // initial height of the ball, in meters
                            (LinearVelocity) FeetPerSecond.of(0.908497 * velocity.in(RotationsPerSecond) - 4.02533), // initial velocity, in m/s
                            (Angle)          Degrees.of(90+24)) // shooter angle
                            .withProjectileTrajectoryDisplayCallBack(
                                (poses) -> Logger.recordOutput("successfulShotsTrajectory", poses.toArray(Pose3d[]::new)),
                                (poses) -> Logger.recordOutput("missedShotsTrajectory", poses.toArray(Pose3d[]::new)))
                    );
            }
        }
    }

    protected void stopFlywheel()
    {
        m_motor.brake();

        m_motor.setPosition(m_simPosTurns);

        m_motorModel.setInputVoltage(m_motor.getAppliedVoltage().in(Volts));
        m_motorModel.update(0.02);

        m_motor.simPeriodic(RadiansPerSecond.of(m_motorModel.getAngularVelocityRadPerSec()));
    }

    protected AngularVelocity getReference()
    {
        return m_lastinput;
    }

    protected AngularVelocity getMeasured()
    {
        return m_motor.getVelocity();
    }
}
