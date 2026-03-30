package frc.robot.subsystems.chassis;

import static edu.wpi.first.units.Units.Inches;

import java.util.Optional;

import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.COTS;
import org.ironmaple.simulation.drivesims.SelfControlledSwerveDriveSimulation;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig;
import org.littletonrobotics.junction.Logger;
import org.photonvision.EstimatedRobotPose;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.Vision;
import us.hebi.quickbuf.Descriptors.Descriptor;

public class SimChassis implements ChassisIO 
{
    private final SelfControlledSwerveDriveSimulation simulatedDrive;
    private final Field2d field2d;

    private boolean m_xMode = false;

    public SimChassis() 
    {
        // For your own code, please configure your drivetrain properly according to the documentation
        final DriveTrainSimulationConfig driveTrainConfig = DriveTrainSimulationConfig.Default()
                // Specify gyro type (for realistic gyro drifting and error simulation)
                .withGyro(COTS.ofPigeon2())
                // Specify swerve module (for realistic swerve dynamics)
                .withSwerveModule(COTS.ofMark4i(
                        DCMotor.getKrakenX60(1), // Drive motor is a Kraken X60
                        DCMotor.getKrakenX44(1), // Steer motor is a Falcon 500
                        COTS.WHEELS.COLSONS.cof, // Use the COF for Colson Wheels
                        3)) // L3 Gear ratio
                // Configures the track length and track width (spacing between swerve modules)
                .withTrackLengthTrackWidth(Inches.of(24), Inches.of(30))
                // Configures the bumper size (dimensions of the robot bumper)
                .withBumperSize(Inches.of(32), Inches.of(38));

        // Creating the SelfControlledSwerveDriveSimulation instance
        simulatedDrive = new SelfControlledSwerveDriveSimulation(
                new SwerveDriveSimulation(driveTrainConfig, new Pose2d(1.0, 1.0, new Rotation2d())));

        // Register the drivetrain simulation to the simulation world
        SimulatedArena.getInstance().addDriveTrainSimulation(simulatedDrive.getDriveTrainSimulation());

        // A field2d widget for debugging
        field2d = new Field2d();

        RobotConfig config;
        try 
        {
            config = RobotConfig.fromGUISettings();
        } 
        catch (Exception e) 
        {
            e.printStackTrace();
            return;
        }

        // Configure the AutoBuilder
        AutoBuilder.configure(
            this::getPose,
            this::resetPose,
            this::getMeasuredSpeeds,
            this::driveRobotRelative,
            new PPHolonomicDriveController(
                new PIDConstants(5.0, 0.0, 0.0),
                new PIDConstants(5.0, 0.0, 0.0)
            ),
            config,
            // Path Flipping: Determines if the path should be flipped based on the robot's alliance color
            () -> DriverStation.getAlliance().orElse(Alliance.Blue).equals(Alliance.Red),
            this);
    }

    @Override
    public void driveRobotRelative(ChassisSpeeds chassisSpeeds) {
        Logger.recordOutput("Simulation/Desired Speeds", chassisSpeeds);
        if (m_xMode) {
            setModuleStates(new SwerveModuleState[] {
                new SwerveModuleState(0, Rotation2d.fromDegrees(45)),
                new SwerveModuleState(0, Rotation2d.fromDegrees(-45)),
                new SwerveModuleState(0, Rotation2d.fromDegrees(-45)),
                new SwerveModuleState(0, Rotation2d.fromDegrees(45))
            });
            return;
        }
        this.simulatedDrive.runChassisSpeeds(
                chassisSpeeds,
                new Translation2d(),
                false,
                true);
    }

    @Override
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        simulatedDrive.runSwerveStates(desiredStates);
    }

    @Override
    public SwerveModulePosition[] getModulePositions() {
        return simulatedDrive.getLatestModulePositions();
    }

    @Override
    public SwerveModuleState[] getModuleStates() {
        return simulatedDrive.getMeasuredStates();
    }

    @Override
    public Pose2d getPose() {
        return simulatedDrive.getOdometryEstimatedPose();
    }

    @Override
    public void resetPose(Pose2d pose) {
        simulatedDrive.setSimulationWorldPose(pose);
        simulatedDrive.resetOdometry(pose);
    }

    @Override
    public void toggleXMode() {
        m_xMode = !m_xMode;
    }

    @Override
    public boolean getIsXMode() {
        return m_xMode;
    }

    @Override
    public void periodic() {
        // update the odometry of the SimplifedSwerveSimulation instance
        simulatedDrive.periodic();

        if (Vision.getResult1() != null)
        {
            Optional<EstimatedRobotPose> visionBotPose1 = Vision.getEstimatedGlobalPoseCam1(Vision.getResult1(), getPose());
            if (visionBotPose1.isPresent()){
                simulatedDrive.addVisionEstimation(visionBotPose1.get().estimatedPose.toPose2d(), Timer.getMatchTime(), Vision.updateEstimationStdDevs(visionBotPose1, visionBotPose1.get().targetsUsed));
            }
        }
        if (Vision.getResult2() != null)
        {
            Optional<EstimatedRobotPose> visionBotPose2 = Vision.getEstimatedGlobalPoseCam2(getPose(), Vision.getResult2());
            if (visionBotPose2.isPresent())
            {
                simulatedDrive.addVisionEstimation(visionBotPose2.get().estimatedPose.toPose2d(), Timer.getTimestamp(), Vision.updateEstimationStdDevs(visionBotPose2, visionBotPose2.get().targetsUsed));
            }
        } 
        

        // send simulation data to dashboard for testing
        field2d.setRobotPose(simulatedDrive.getActualPoseInSimulationWorld());
        field2d.getObject("odometry").setPose(getPose());
        Logger.recordOutput("Simulation/Actual Pose", simulatedDrive.getActualPoseInSimulationWorld());
        Logger.recordOutput("Simulation/Measured Pose", getPose());
    }
}