package frc.robot.subsystems.chassis;

import static edu.wpi.first.units.Units.Inches;

import java.util.List;
import java.util.Optional;

import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.COTS;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig;
import org.photonvision.EstimatedRobotPose;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Vision;
import frc.robot.Constants;
import frc.robot.subsystems.gyro.Gyro;
import frc.robot.subsystems.gyro.GyroIO;
import frc.robot.subsystems.gyro.GyroIOSim;

/// @brief Chassis subsystem for swerve drive control
///
///       Red                      <----- Zero Angle                       Blue
///                            <--- 0 degrees    180 degrees ---->     X  <-----
///   ---  +-------------------------------------------------------------------+ (0, 0)
///    ^   |                7  6              |             17 28           29 |  
///    |   |                                  |                             30 |  |
///    |   |                                  |                                |  |
///    |   |                                  |                                |  V
///    |   |                                  |                                |
///    |   |                8  5              |             18 27              |  Y
/// 8.07 m | 16          9       4            |          19       26        31 |
///    |   | 15         10       3            |          20       25        32 |
///    |   |               11  2              |             21 24              |
///    |   |                                  |                                |
///    |   |                                  |                                |
///    |   | 14                               |                                |
///    V   | 13            12  1              |             22 23              |
///   ---  +-------------------------------------------------------------------+
///        |<----------------------------- 16.56 m --------------------------->|
///                                       Top View
public class Chassis extends SubsystemBase
{
    // Swerve module order for kinematics calculations
    //
    //         Front          Translation2d Coordinates
    //   FL +----------+ FR              ^ X
    //      | 0      1 |                 |
    //      |          |            Y    |
    //      |          |          <------+-------
    //      | 2      3 |                 |
    //   RL +----------+ RR              |
    
    private SwerveModuleIO[] m_swerveModules;
    
    private SwerveDrivePoseEstimator m_poseEstimator = new SwerveDrivePoseEstimator(
        Constants.Chassis.kinematics,         // Kinematics object
        new Rotation2d(0),  // Initial gyro angle
        new SwerveModulePosition[]{new SwerveModulePosition(0, new Rotation2d(0)), new SwerveModulePosition(0, new Rotation2d(0)), new SwerveModulePosition(0, new Rotation2d(0)), new SwerveModulePosition(0, new Rotation2d(0))},     // Initial module positions
        new Pose2d()              // Initial pose, will be overriden by vision
    );

    // Create and configure a drivetrain simulation configuration
    DriveTrainSimulationConfig m_driveTrainSimulationConfig;
    SwerveDriveSimulation m_swerveDriveSimulation;

    ChassisSpeeds m_desiredSpeeds = new ChassisSpeeds(0,0,0);

    boolean       m_isFieldRelative = false;

    boolean       m_isXMode = false;

    GyroIO        m_gyro = new Gyro(Constants.CanIds.PigeonGyroId);

    Vision        m_vision;

    public Chassis()
    {
        m_swerveModules = List.of(
            new SwerveModule(Constants.CanIds.FrontLeftDriveId,  Constants.CanIds.FrontLeftTurnId,  Constants.CanIds.FrontLeftEncoderId),
            new SwerveModule(Constants.CanIds.FrontRightDriveId, Constants.CanIds.FrontRightTurnId, Constants.CanIds.FrontRightEncoderId),
            new SwerveModule(Constants.CanIds.BackRightDriveId,  Constants.CanIds.BackRightTurnId,  Constants.CanIds.BackRightEncoderId),
            new SwerveModule(Constants.CanIds.BackLeftDriveId,   Constants.CanIds.BackLeftTurnId,   Constants.CanIds.BackLeftEncoderId)
        ).toArray(new SwerveModuleIO[0]);

        m_gyro = new Gyro(Constants.CanIds.PigeonGyroId);
        
        RobotConfig config;
        try {
            config = RobotConfig.fromGUISettings();
        } catch (Exception e) {
            e.printStackTrace();
            return;
        }

        // Configure the AutoBuilder
        AutoBuilder.configure(
            this::getPose,                                       // Robot pose supplier
            this::resetPose,                     // Method to reset odometry (will be called if your auto has a starting pose)
            this::getSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
            (speeds, feedforwards) -> { driveRelative(speeds); }, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also optionally outputs individual module feedforwards
            new PPHolonomicDriveController(          // PPHolonomicController is the built in path following controller for holonomic drive trains
                // TODO: magic numbers, test these
                new PIDConstants(1.0, 0.0, 0.0),                       // Translation PID constants
                new PIDConstants(1.0, 0.0, 0.0)                        // Rotation PID constants
            ),
            config, // The robot configuration
                // Path Flipping: Determines if the path should be flipped based on the robot's alliance color
                () -> DriverStation.getAlliance().orElse(Alliance.Blue).equals(Alliance.Red),
            this // Reference to this subsystem to set requirements
        );
        
        resetGyroAngle();
        resetWheelAnglesToZero();
    }

    public void  drive(ChassisSpeeds speeds)
    {
        driveRelative(DriverStation.isTeleop() ? 
            ChassisSpeeds.fromFieldRelativeSpeeds(speeds, getHeading().plus((DriverStation.getAlliance().orElse(Alliance.Blue).equals(Alliance.Red)) ? Rotation2d.kPi : new Rotation2d(0))) :
            speeds);
    }

    public void  driveRelative(ChassisSpeeds speeds)
    {
        // If the chassis is in x mode, than stay in x mode, ignoring the desired speeds
        if (m_isXMode)
        {
            // Set the module states to x mode
            setModuleStates((SwerveModuleState[]) Constants.Chassis.xStates);

            // Save the desired speeds for logging later
            return;
        }
        
        // Save the desired states for use and logging later
        SwerveModuleState[] desiredStates = Constants.Chassis.kinematics.toSwerveModuleStates(speeds);

        // Set the desired state for each swerve module
        setModuleStates(desiredStates);
    }

    public void setModuleStates(SwerveModuleState[] states)
    {
        // Set the desired state for each swerve module
        m_swerveModules[0].setDesiredState(states[0], "Front Left " );
        m_swerveModules[1].setDesiredState(states[1], "Front Right ");
        m_swerveModules[2].setDesiredState(states[2], "Rear Right "  );
        m_swerveModules[3].setDesiredState(states[3], "Rear Left " );
    }

    public void resetGyroAngle()
    {
        m_gyro.resetGyroAngle();
    }

    public void resetWheelAnglesToZero()
    {
            // Set the swerve wheel angles to zero
            m_swerveModules[0].setWheelAngleToForward(Constants.Chassis.FrontLeftForwardDegrees);
            m_swerveModules[1].setWheelAngleToForward(Constants.Chassis.FrontRightForwardDegrees);
            m_swerveModules[2].setWheelAngleToForward(Constants.Chassis.BackRightForwardDegrees);
            m_swerveModules[3].setWheelAngleToForward(Constants.Chassis.BackLeftForwardDegrees);
    }

    public void resetPose(Pose2d pose)
    {
        m_poseEstimator.resetPose(pose);
    }

    public SwerveModuleState[] getModuleStates()
    {
        SwerveModuleState[] states = {
            m_swerveModules[0].getState(),
            m_swerveModules[1].getState(),
            m_swerveModules[2].getState(),
            m_swerveModules[3].getState()
        };

        return states;
    }

    public SwerveModulePosition[] getModulePositions()
    {
        SwerveModulePosition[] positions = {
            m_swerveModules[0].getPosition(),
            m_swerveModules[1].getPosition(),
            m_swerveModules[2].getPosition(),
            m_swerveModules[3].getPosition()
        };

        return positions;
    }

    public void toggleFieldCentric()
    {
        m_isFieldRelative = !m_isFieldRelative;
    }

    public void toggleXMode()
    {
        m_isXMode = !m_isXMode;
    }
    
    public Rotation2d getHeading()
    {
        return m_gyro.getGyroRotation();
    }

    public Pose2d getPose()
    {
        return m_poseEstimator.getEstimatedPosition();
    }

    public ChassisSpeeds getSpeeds()
    {
        return Constants.Chassis.kinematics.toChassisSpeeds(getModuleStates());
    }

    @Override
    public void periodic()
    {
        m_poseEstimator.update(getHeading(), getModulePositions());

        if (Vision.getResult1() != null)
        {
            Optional<EstimatedRobotPose> visionBotPose1 = Vision.getEstimatedGlobalPoseCam1(Vision.getResult1(), getPose());
            if (visionBotPose1.isPresent()){
                m_poseEstimator.addVisionMeasurement(visionBotPose1.get().estimatedPose.toPose2d(), Timer.getMatchTime(), Vision.updateEstimationStdDevs(visionBotPose1, visionBotPose1.get().targetsUsed));
            }
        }
        if (Vision.getResult2() != null)
        {
            Optional<EstimatedRobotPose> visionBotPose2 = Vision.getEstimatedGlobalPoseCam2(getPose(), Vision.getResult2());
            if (visionBotPose2.isPresent())
            {
                m_poseEstimator.addVisionMeasurement(visionBotPose2.get().estimatedPose.toPose2d(), Timer.getTimestamp(), Vision.updateEstimationStdDevs(visionBotPose2, visionBotPose2.get().targetsUsed));
            }
        } 
    }

    public boolean getIsXMode()         { return m_isXMode; }
    public boolean getIsFieldRelative() { return m_isFieldRelative; }
};