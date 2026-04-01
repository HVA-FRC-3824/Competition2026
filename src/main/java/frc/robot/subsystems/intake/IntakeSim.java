package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.FeetPerSecond;
import static edu.wpi.first.units.Units.Inches;
import org.ironmaple.simulation.IntakeSimulation;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.AbstractDriveTrainSimulation;
import org.ironmaple.simulation.seasonspecific.rebuilt2026.RebuiltFuelOnFly;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

public class IntakeSim implements IntakeIO
{
    private boolean m_canIntake = false;

    private final IntakeSimulation m_intakeSimulation;

    private final AbstractDriveTrainSimulation m_driveSimulation;

    public IntakeSim(AbstractDriveTrainSimulation driveTrain)
    {
        m_driveSimulation = driveTrain;

        m_intakeSimulation = IntakeSimulation.OverTheBumperIntake(
            // Specify the type of game pieces that the intake can collect
            "Fuel",
            // Specify the drivetrain to which this intake is attached
            driveTrain,
            Inches.of(27.0),    // Width of the intake
            Inches.of(12.0),    // How far the intake extends beyond the bumper
            IntakeSimulation.IntakeSide.FRONT, // Which side of the chassis the intake is on
            30                  // Capacity: max game pieces the intake can hold
        );
    }

    @Override
    public void setPos(double pos)
    {
        // Simulate intake position here
        if (pos > Constants.Intake.IntakeStowedTurns) 
            {
            m_canIntake = true;
        }
        else
        {
            m_canIntake = false;
        }
    }

    @Override
    public void setRollers(double speed)
    {
        // Simulate intake rollers here
        if (speed >= Constants.Intake.IntakeDriveTurnsPerSec && m_canIntake) 
        {
            m_intakeSimulation.startIntake();
            SmartDashboard.putString("INTAKEEEE", "Intaking");
        }
        else
        {
            m_intakeSimulation.stopIntake();
            SmartDashboard.putString("INTAKEEEE", "Not Intaking");
        }
    }

    public boolean isFuelInsideIntake() {
        return m_intakeSimulation.getGamePiecesAmount() != 0; // True if there is a game piece in the intake
    }

    @Override // Defined by IntakeIO
    public void launchFuel(double shooterSpeed) {
        // We run this twice to emulate the 2-3 balls that we will be shooting at a time
        
        // Dont shoot balls every loop cycle, to prevent launching too many balls at once
        if (((int) Timer.getTimestamp()) % 3 != 0)
            return;

        if (m_intakeSimulation.obtainGamePieceFromIntake())
        {
            SimulatedArena.getInstance()
                .addGamePieceProjectile(
                    new RebuiltFuelOnFly(
                        (Translation2d)  m_driveSimulation.getSimulatedDriveTrainPose().getTranslation(),
                        (Translation2d)  new Translation2d(Units.inchesToMeters(-3), Units.inchesToMeters(-20)), // shooter offet from center
                        (ChassisSpeeds)  m_driveSimulation.getDriveTrainSimulatedChassisSpeedsFieldRelative(),
                        (Rotation2d)     m_driveSimulation.getSimulatedDriveTrainPose().getRotation(),
                        (Distance)       Inches.of(15), // initial height of the ball, in meters
                        (LinearVelocity) FeetPerSecond.of(shooterSpeed), // initial velocity, in m/s
                        (Angle)          Degrees.of(90+24)) // shooter angle
                        .withProjectileTrajectoryDisplayCallBack(
                            (poses) -> Logger.recordOutput("successfulShotsTrajectory", poses.toArray(Pose3d[]::new)),
                            (poses) -> Logger.recordOutput("missedShotsTrajectory", poses.toArray(Pose3d[]::new)))
                );
        }

        if (m_intakeSimulation.obtainGamePieceFromIntake())
        {
            SimulatedArena.getInstance()
                .addGamePieceProjectile(
                    new RebuiltFuelOnFly(
                        (Translation2d)  m_driveSimulation.getSimulatedDriveTrainPose().getTranslation(),
                        (Translation2d)  new Translation2d(Units.inchesToMeters(3), Units.inchesToMeters(-20)), // shooter offet from center
                        (ChassisSpeeds)  m_driveSimulation.getDriveTrainSimulatedChassisSpeedsFieldRelative(),
                        (Rotation2d)     m_driveSimulation.getSimulatedDriveTrainPose().getRotation(),
                        (Distance)       Inches.of(15), // initial height of the ball, in meters
                        (LinearVelocity) FeetPerSecond.of(shooterSpeed), // initial velocity, in m/s
                        (Angle)          Degrees.of(90+24)) // shooter angle
                        .withProjectileTrajectoryDisplayCallBack(
                            (poses) -> Logger.recordOutput("successfulShotsTrajectory", poses.toArray(Pose3d[]::new)),
                            (poses) -> Logger.recordOutput("missedShotsTrajectory", poses.toArray(Pose3d[]::new)))
                );
        }
    }    
}
