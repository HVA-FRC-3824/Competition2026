package frc3824.subsystems;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;

import com.ctre.phoenix6.hardware.Pigeon2;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc3824.Constants.ChassisConstants;
import frc3824.subsystems.Vision;
import frc3824.Constants;
import frc3824.lib.SwerveModule;

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
        public Chassis()
        {
            // TODO: When the robot CAD is done, update the pathplanner robot settings
            RobotConfig config;
            try {
                config = RobotConfig.fromGUISettings();
            } catch (Exception e) {
                // TODO make real
                e.printStackTrace();
                return;
            }

            // Configure the AutoBuilder
            AutoBuilder.configure(
                this::GetPose,                                       // Robot pose supplier
                this::ResetPose,                     // Method to reset odometry (will be called if your auto has a starting pose)
                this::GetSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
                (speeds, feedforwards) -> { DriveRelative(speeds); }, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also optionally outputs individual module feedforwards
                new PPHolonomicDriveController(          // PPHolonomicController is the built in path following controller for holonomic drive trains
                    // TODO: magic numbers, test these
                    new PIDConstants(1.0, 0.0, 0.0),                       // Translation PID constants
                    new PIDConstants(1.0, 0.0, 0.0)                        // Rotation PID constants
                ),
                config,  // The robot configuration
                () -> {
                    // Boolean supplier that controls when the path will be mirrored for the red alliance
                    // This will flip the path being followed to the red side of the field.
                    // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

                    // THIS MEANS TO DESIGN ALL AUTOS AS BEING ON THE BLUE SIDE!!!!

                    Alliance alliance = DriverStation.getAlliance().orElse(Alliance.Red);
                    return alliance == Alliance.Red;
                },
                this // Reference to this subsystem to set requirements
            );
        }

        public void  Drive(ChassisSpeeds speeds)
        {
            DriveRelative(DriverStation.isTeleop() ? ChassisSpeeds.fromFieldRelativeSpeeds(speeds, GetHeading()) : speeds);
        }

        public void  DriveRelative(ChassisSpeeds speeds)
        {
            // If the chassis is in x mode, than stay in x mode, ignoring the desired speeds
            if (m_isXMode)
            {
                // Set the module states to x mode
                SetModuleStates(Constants.ChassisConstants.xStates);

                // Save the desired speeds for logging later
                m_desiredStates = Constants.ChassisConstants.xStates;
                return;
            }
            
            m_desiredSpeeds = speeds;

            // Save the desired states for use and logging later
            m_desiredStates = Constants.ChassisConstants.kinematics.toSwerveModuleStates(speeds);

            // Set the desired state for each swerve module
            SetModuleStates(m_desiredStates);
        }

        public void SetModuleStates(SwerveModuleState[] states)
        {
            // Set the desired state for each swerve module
            m_swerveModules[0].setDesiredState(states[0], "Front Left " );
            m_swerveModules[1].setDesiredState(states[1], "Front Right ");
            m_swerveModules[2].setDesiredState(states[2], "Rear Left "  );
            m_swerveModules[3].setDesiredState(states[3], "Rear Right " );
        }

        public void  ResetGyroAngle()
        {
            m_gyro.reset();
        }

        public void  ResetPoseGyroAngle() { m_gyro.reset(); }

        public void  ResetWheelAnglesToZero()
        {
                // Set the swerve wheel angles to zero
                m_swerveModules[0].setWheelAngleToForward(ChassisConstants.FrontRightForwardDegrees);
                m_swerveModules[1].setWheelAngleToForward(ChassisConstants.FrontRightForwardDegrees);
                m_swerveModules[2].setWheelAngleToForward(ChassisConstants.FrontRightForwardDegrees);
                m_swerveModules[3].setWheelAngleToForward(ChassisConstants.FrontRightForwardDegrees);
        }

        public void ResetPose(Pose2d pose)
        {
            m_poseEstimator.resetPose(pose);
        }

        SwerveModuleState[] GetModuleStates()
        {
            SwerveModuleState[] states = {
                m_swerveModules[0].getState(),
                m_swerveModules[1].getState(),
                m_swerveModules[2].getState(),
                m_swerveModules[3].getState()
            };

            return states;
        }

        SwerveModulePosition[] GetModulePositions()
        {
            SwerveModulePosition[] positions = {
                m_swerveModules[0].getPosition(),
                m_swerveModules[1].getPosition(),
                m_swerveModules[2].getPosition(),
                m_swerveModules[3].getPosition()
            };

            return positions;
        }
    
        public void ToggleFieldCentric()
        {
            m_isFieldRelative = !m_isFieldRelative;
        }

        public void ToggleXMode()
        {
            m_isXMode = !m_isXMode;
        }

        public void ToggleSlowMode()
        {
            m_isSlowMode = !m_isSlowMode;
        }
        
        public Rotation2d GetHeading()
        {
            return m_gyro.getRotation2d();
        }

        public Pose2d GetPose()
        {
            return m_poseEstimator.getEstimatedPosition();
        }

        public ChassisSpeeds GetSpeeds()
        {
            return ChassisConstants.kinematics.toChassisSpeeds(GetModuleStates());
        }

        @Override
        public void periodic()
        {
            m_poseEstimator.update(GetHeading(), GetModulePositions());

            if (Vision.getm_result1() != null)
            {
                Optional<EstimatedRobotPose> visionBotPose1 = Vision.getEstimatedGlobalPoseCam1(Vision.getm_result1(), GetPose());
                if (visionBotPose1.isPresent()){
                    m_poseEstimator.addVisionMeasurement(visionBotPose1.get().estimatedPose.toPose2d(), Timer.getMatchTime(), Vision.updateEstimationStdDevs(visionBotPose1, visionBotPose1.get().targetsUsed));
                }
            }
            if (Vision.getm_result2() != null)
            {
                Optional<EstimatedRobotPose> visionBotPose2 = Vision.getEstimatedGlobalPoseCam2(GetPose(), Vision.getm_result2());
                if (visionBotPose2.isPresent())
                {
                    m_poseEstimator.addVisionMeasurement(visionBotPose2.get().estimatedPose.toPose2d(), Timer.getTimestamp(), Vision.updateEstimationStdDevs(visionBotPose2, visionBotPose2.get().targetsUsed));
                }
            } 
        }

        public boolean GetIsSlowMode() { return m_isSlowMode; }
    
        // Swerve module order for kinematics calculations
        //
        //         Front          Translation2d Coordinates
        //   FL +----------+ FR              ^ X
        //      | 0      1 |                 |
        //      |          |            Y    |
        //      |          |          <------+-------
        //      | 2      3 |                 |
        //   RL +----------+ RR              |
        
        SwerveModule[] m_swerveModules = {
            new SwerveModule(Constants.ConstantsCanIds.FrontLeftDriveId,  Constants.ConstantsCanIds.FrontLeftTurnId,  Constants.ConstantsCanIds.FrontLeftEncoderId),
            new SwerveModule(Constants.ConstantsCanIds.FrontRightDriveId, Constants.ConstantsCanIds.FrontRightTurnId, Constants.ConstantsCanIds.FrontRightEncoderId),
            new SwerveModule(Constants.ConstantsCanIds.BackLeftDriveId,   Constants.ConstantsCanIds.BackLeftTurnId,   Constants.ConstantsCanIds.BackLeftEncoderId),
            new SwerveModule(Constants.ConstantsCanIds.BackRightDriveId,  Constants.ConstantsCanIds.BackRightTurnId,  Constants.ConstantsCanIds.BackRightEncoderId) 
        };
        
        private SwerveDrivePoseEstimator m_poseEstimator = new SwerveDrivePoseEstimator(
            Constants.ChassisConstants.kinematics,         // Kinematics object
            new Rotation2d(0),  // Initial gyro angle
            GetModulePositions(),     // Initial module positions
            new Pose2d()              // Initial pose, will be overriden by vision
        );

        SwerveModuleState[] m_desiredStates = {
            new SwerveModuleState(0, new Rotation2d()), 
            new SwerveModuleState(0, new Rotation2d()), 
            new SwerveModuleState(0, new Rotation2d()), 
            new SwerveModuleState(0, new Rotation2d())
        };

        ChassisSpeeds m_desiredSpeeds = new ChassisSpeeds(0,0,0);

        boolean       m_isFieldRelative = false;

        boolean       m_isXMode = false;

        boolean       m_isSlowMode = false;
    
        Pigeon2       m_gyro = new Pigeon2(1);

        Vision        m_vision;
};