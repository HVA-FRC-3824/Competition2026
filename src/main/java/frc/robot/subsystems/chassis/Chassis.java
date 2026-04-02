package frc.robot.subsystems.chassis;

import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Vision;
import frc.robot.Constants;
import frc.robot.subsystems.gyro.Gyro;
import frc.robot.subsystems.gyro.GyroIO;

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
public class Chassis extends SubsystemBase implements ChassisIO
{
    // Swerve module order for kinematics calculations
    //
    //         Front          Translation2d Coordinates
    //   FL +----------+ FR              ^ X
    //      | 0      1 |                 |
    //      |          |            Y    |
    //      |          |          <------+-------
    //      | 2      3 |                 |
    //   BL +----------+ BR              |
    
    private SwerveModule m_frSwerveModules = new SwerveModule(Constants.CanIds.FrontRightDriveId, Constants.CanIds.FrontRightTurnId, Constants.CanIds.FrontRightEncoderId);
    private SwerveModule m_flSwerveModules = new SwerveModule(Constants.CanIds.FrontLeftDriveId,  Constants.CanIds.FrontLeftTurnId,  Constants.CanIds.FrontLeftEncoderId);
    private SwerveModule m_brSwerveModules = new SwerveModule(Constants.CanIds.BackRightDriveId,  Constants.CanIds.BackRightTurnId,  Constants.CanIds.BackRightEncoderId);
    private SwerveModule m_blSwerveModules = new SwerveModule(Constants.CanIds.BackLeftDriveId,   Constants.CanIds.BackLeftTurnId,   Constants.CanIds.BackLeftEncoderId);

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

    boolean       m_isXMode = false;

    GyroIO        m_gyro = new Gyro(Constants.CanIds.PigeonGyroId);

    Vision         m_vision = new Vision(this::addVisionMeasurement);

    public Chassis()
    {
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
            this::getMeasuredSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
            (speeds, feedforwards) -> { driveRobotRelative(speeds); }, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also optionally outputs individual module feedforwards
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

    @Override
    public void  driveRobotRelative(ChassisSpeeds speeds)
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

    @Override
    public void setModuleStates(SwerveModuleState[] states)
    {
        SwerveDriveKinematics.desaturateWheelSpeeds(states, Constants.Chassis.MaximumSpeedMetersPerSec);

        states[0].optimize(m_flSwerveModules.getState().angle);
        states[1].optimize(m_frSwerveModules.getState().angle);
        states[2].optimize(m_blSwerveModules.getState().angle);
        states[3].optimize(m_brSwerveModules.getState().angle);

        // Set the desired state for each swerve module
        m_flSwerveModules.setDesiredState(states[0], "Front Left " );
        m_frSwerveModules.setDesiredState(states[1], "Front Right ");
        m_blSwerveModules.setDesiredState(states[2], "Rear Right "  );
        m_brSwerveModules.setDesiredState(states[3], "Rear Left " );
    }

    @Override
    public void resetGyroAngle()
    {
        m_gyro.resetGyroAngle();
    }

    public void resetWheelAnglesToZero()
    {
        // Set the swerve wheel angles to zero
        m_flSwerveModules.setWheelAngleToForward(Constants.Chassis.FrontLeftForwardDegrees);
        m_frSwerveModules.setWheelAngleToForward(Constants.Chassis.FrontRightForwardDegrees);
        m_blSwerveModules.setWheelAngleToForward(Constants.Chassis.BackLeftForwardDegrees);
        m_brSwerveModules.setWheelAngleToForward(Constants.Chassis.BackRightForwardDegrees);
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
            m_flSwerveModules.getState(),
            m_frSwerveModules.getState(),
            m_blSwerveModules.getState(),
            m_brSwerveModules.getState()
        };

        return states;
    }

    @Override
    public SwerveModulePosition[] getModulePositions()
    {
        SwerveModulePosition[] positions = {
            m_flSwerveModules.getPosition(),
            m_frSwerveModules.getPosition(),
            m_blSwerveModules.getPosition(),
            m_brSwerveModules.getPosition()
        };

        return positions;
    }

    @Override
    public void toggleXMode()
    {
        m_isXMode = !m_isXMode;
    }
    
    @Override
    public Rotation2d getHeading()
    {
        return m_gyro.getGyroRotation();
    }

    @Override
    public Pose2d getPose()
    {
        return m_poseEstimator.getEstimatedPosition();
    }

    @Override
    public void periodic()
    {
        m_poseEstimator.update(getHeading(), getModulePositions());

        m_vision.periodic();
    }

    public void addVisionMeasurement(Pose2d visionMeasurement, double timestampSeconds, Matrix<N3, N1> stdDevs) 
    {
        m_poseEstimator.addVisionMeasurement(visionMeasurement, timestampSeconds, stdDevs);
    }

    @Override
    public boolean getIsXMode() { return m_isXMode; }
};