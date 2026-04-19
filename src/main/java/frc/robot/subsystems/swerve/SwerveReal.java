package frc.robot.subsystems.swerve;

import java.util.ArrayList;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants;
import frc.robot.lib.VisionMeasurement;
import frc.robot.subsystems.swerveModule.SwerveModule;
import frc.robot.subsystems.swerveModule.SwerveModuleTalonFx;

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
public class SwerveReal extends Swerve
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
    

    private SwerveDrivePoseEstimator m_poseEstimator = new SwerveDrivePoseEstimator(
        Constants.Chassis.kinematics,
        new Rotation2d(0),  // Initial gyro angle
        new SwerveModulePosition[]{new SwerveModulePosition(0, new Rotation2d(0)), new SwerveModulePosition(0, new Rotation2d(0)), new SwerveModulePosition(0, new Rotation2d(0)), new SwerveModulePosition(0, new Rotation2d(0))},     // Initial module positions
        new Pose2d()              // Initial pose
    );

    private SwerveModule m_frSwerveModules;
    private SwerveModule m_flSwerveModules;
    private SwerveModule m_brSwerveModules;
    private SwerveModule m_blSwerveModules;

    public SwerveReal() {
        
        m_inputs = new Inputs();
        m_outputs = new Outputs();

        m_frSwerveModules = new SwerveModuleTalonFx(
            0,
            Constants.CanIds.FrontRightDriveId, 
            Constants.CanIds.FrontRightTurnId, 
            Constants.CanIds.FrontRightEncoderId);

        m_flSwerveModules = new SwerveModuleTalonFx(
            1,
            Constants.CanIds.FrontLeftDriveId,  
            Constants.CanIds.FrontLeftTurnId,  
            Constants.CanIds.FrontLeftEncoderId);

        m_brSwerveModules = new SwerveModuleTalonFx(
            2,
            Constants.CanIds.BackRightDriveId,  
            Constants.CanIds.BackRightTurnId,  
            Constants.CanIds.BackRightEncoderId);

        m_blSwerveModules = new SwerveModuleTalonFx(
            3,
            Constants.CanIds.BackLeftDriveId,   
            Constants.CanIds.BackLeftTurnId,   
            Constants.CanIds.BackLeftEncoderId);
        
        resetWheelAnglesToZero();
    }

    @Override
    protected ChassisSpeeds getMeasuredSpeeds()
    {
        return Constants.Chassis.kinematics.toChassisSpeeds(getModuleStates());
    }

    @Override
    protected void updatePoseEstimator(Rotation2d gyroHeading, SwerveModulePosition[] modulePositions) {
        m_poseEstimator.update(gyroHeading, modulePositions);
    }

    @Override
    protected void updateVisionInputs(VisionMeasurement measurement) {
        m_poseEstimator.addVisionMeasurement(
            measurement.visionMeasurement(), 
            measurement.timestampSeconds(), 
            measurement.stdDevs()
        );
    }

    @Override
    protected void setModules(ArrayList<SwerveModule.Inputs> inputs)
    {
        // Set the desired state for each swerve module
        m_flSwerveModules.setInputs(inputs.get(0));
        m_frSwerveModules.setInputs(inputs.get(1));
        m_blSwerveModules.setInputs(inputs.get(2));
        m_brSwerveModules.setInputs(inputs.get(3));
    }


    public void resetWheelAnglesToZero()
    {
        // Set the swerve wheel angles to zero
        m_flSwerveModules.setWheelAngleToForward(Constants.Chassis.FrontLeftForwardsAngle);
        m_frSwerveModules.setWheelAngleToForward(Constants.Chassis.FrontRightForwardsAngle);
        m_blSwerveModules.setWheelAngleToForward(Constants.Chassis.BackLeftForwardsAngle);
        m_brSwerveModules.setWheelAngleToForward(Constants.Chassis.BackRightForwardsAngle);
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
            m_flSwerveModules.getOutputs().m_measuredSwerveModuleState,
            m_frSwerveModules.getOutputs().m_measuredSwerveModuleState,
            m_blSwerveModules.getOutputs().m_measuredSwerveModuleState,
            m_brSwerveModules.getOutputs().m_measuredSwerveModuleState
        };

        return states;
    }

    @Override
    public SwerveModulePosition[] getModulePositions()
    {
        SwerveModulePosition[] positions = {
            m_flSwerveModules.getOutputs().m_swerveModulePosition,
            m_frSwerveModules.getOutputs().m_swerveModulePosition,
            m_blSwerveModules.getOutputs().m_swerveModulePosition,
            m_brSwerveModules.getOutputs().m_swerveModulePosition
        };

        return positions;
    }

    @Override
    public Pose2d getPose()
    {
        return m_poseEstimator.getEstimatedPosition();
    }
};