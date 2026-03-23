#pragma once

#pragma region Includes
#include <wpi/array.h>

#include "lib/Gyro.h"

#include <frc/DriverStation.h>
#include <frc/Timer.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc2/command/SubsystemBase.h>

#include <pathplanner/lib/auto/AutoBuilder.h>
#include <pathplanner/lib/config/RobotConfig.h>
#include <pathplanner/lib/controllers/PPHolonomicDriveController.h>
#include <pathplanner/lib/path/PathPlannerPath.h>

#include <frc/geometry/Pose2d.h>

#include <frc/geometry/Rotation2d.h>
#include <frc/geometry/Translation2d.h>

#include <frc/kinematics/ChassisSpeeds.h>
#include <frc/kinematics/SwerveModuleState.h>
#include <frc/kinematics/SwerveModulePosition.h>
#include <frc/kinematics/SwerveDriveKinematics.h>

#include <frc/estimator/SwerveDrivePoseEstimator.h>

#include "lib/Logging.h"
#include "lib/VisionPose.h"
#include "lib/SwerveModule.h"

#include "Constants.h"
#include "ConstantsRoboRio.h"
#pragma endregion

#pragma region ChassisConstants
namespace ChassisConstants
{
    // NOTE: The absolute encoder range is 0.5 to -0.5
    // These are the absolute encoder values that correspond to the wheels facing "forward"
    constexpr units::degree_t FrontLeftForwardAngle { -0.023438 * 360.0 };
    constexpr units::degree_t FrontRightForwardAngle{ -0.362793 * 360.0 };
    constexpr units::degree_t BackLeftForwardAngle  { -0.361328 * 360.0 };
    constexpr units::degree_t BackRightForwardAngle { -0.276855 * 360.0 };

    constexpr auto MaximumSpeed = 10_fps;
    constexpr auto MaximumAngularVelocity = 1_rad_per_s * (std::numbers::pi);

    constexpr units::inch_t WheelBase {30.0};
    constexpr units::inch_t TrackWidth{24.0};

    constexpr wpi::array<frc::SwerveModuleState, 4> xStates
    {
        frc::SwerveModuleState{0_mps, 315_deg},  // FL
        frc::SwerveModuleState{0_mps,  45_deg},  // FR
        frc::SwerveModuleState{0_mps,  45_deg},  // BL
        frc::SwerveModuleState{0_mps, 315_deg}   // BR
    };

    constexpr pathplanner::PathConstraints constraints{MaximumSpeed, MaximumSpeed / 1_s, MaximumAngularVelocity, MaximumAngularVelocity / 1_s};
    
    constexpr std::string_view             BackCameraName{"CameraBack"};

    // 13 in backwards 17.125 in up 180 roll rotation
    constexpr frc::Transform3d             BackRobotToCamera{frc::Translation3d{-13_in, 0_in, 17.0_in}, frc::Rotation3d{180_deg, 0_deg, 180_deg}};

    constexpr std::string_view             TopCameraName{"CameraTop"};

    // 9.55 in right 20 in up 3 in back
    constexpr frc::Transform3d             TopRobotToCamera{frc::Translation3d{-3_in, -9.5_in, 20_in}, frc::Rotation3d{0_deg, 0_deg, 0_deg}};


    const     frc::AprilTagFieldLayout     TagLayout = frc::AprilTagFieldLayout::LoadField(frc::AprilTagField::k2026RebuiltWelded);

    const     Eigen::Matrix<double, 3, 1>  SingleTagStdDevs{2.0, 2.0, 1.0}; // Reduced by 1/4
    const     Eigen::Matrix<double, 3, 1>  MultiTagStdDevs {0.5, 0.5, 0.5}; // Reduced by 1/2
}
#pragma endregion

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
class Chassis : public frc2::SubsystemBase
{
    public:

        explicit                                 Chassis();

        void                                     Drive(const frc::ChassisSpeeds &speeds);
        void                                     DriveRelative(const frc::ChassisSpeeds &speeds);

        void                                     SetModuleStates(wpi::array<frc::SwerveModuleState, 4> states);

        void                                     ResetGyroAngle();
        void                                     ResetPoseGyroAngle() { m_gyro.PoseReset(); }
        void                                     ResetWheelAnglesToZero();
        void                                     ResetPose(frc::Pose2d pose);

        wpi::array<frc::SwerveModuleState, 4>    GetModuleStates();
        wpi::array<frc::SwerveModulePosition, 4> GetModulePositions();
    
        void                                     ToggleFieldCentric();
        void                                     ToggleXMode();
        void                                     ToggleSlowMode();

        frc::Rotation2d                          GetDriverHeading();
        frc::Rotation2d                          GetPoseHeading();
        frc::Pose2d                              GetPose();
        frc::ChassisSpeeds                       GetSpeeds();

        void                                     Periodic() override;

        bool                                     GetIsSlowMode() { return m_isSlowMode; }
    
    private:
        
        // Swerve module order for kinematics calculations
        //
        //         Front          Translation2d Coordinates
        //   FL +----------+ FR              ^ X
        //      | 0      1 |                 |
        //      |          |            Y    |
        //      |          |          <------+-------
        //      | 2      3 |                 |
        //   RL +----------+ RR              |
        
        std::array<SwerveModule, 4> m_swerveModules
        {
            SwerveModule{ConstantsCanIds::FrontLeftDriveId,  ConstantsCanIds::FrontLeftTurnId,  ConstantsCanIds::FrontLeftEncoderId},
            SwerveModule{ConstantsCanIds::FrontRightDriveId, ConstantsCanIds::FrontRightTurnId, ConstantsCanIds::FrontRightEncoderId},
            SwerveModule{ConstantsCanIds::BackLeftDriveId,   ConstantsCanIds::BackLeftTurnId,   ConstantsCanIds::BackLeftEncoderId},
            SwerveModule{ConstantsCanIds::BackRightDriveId,  ConstantsCanIds::BackRightTurnId,  ConstantsCanIds::BackRightEncoderId} 
        };
        
        frc::SwerveDriveKinematics<4> m_kinematics
        {
            frc::Translation2d{ ChassisConstants::WheelBase / 2,  ChassisConstants::TrackWidth / 2}, // Front Left
            frc::Translation2d{ ChassisConstants::WheelBase / 2, -ChassisConstants::TrackWidth / 2}, // Front Right
            frc::Translation2d{-ChassisConstants::WheelBase / 2,  ChassisConstants::TrackWidth / 2}, // Back Left
            frc::Translation2d{-ChassisConstants::WheelBase / 2, -ChassisConstants::TrackWidth / 2}  // Back Right
        };

        frc::SwerveDrivePoseEstimator<4> m_poseEstimator
        {
            m_kinematics,         // Kinematics object
            frc::Rotation2d{0_deg},  // Initial gyro angle
            GetModulePositions(),    // Initial module positions
            frc::Pose2d()            // Initial pose, will be overriden by vision
        };

        wpi::array<frc::SwerveModuleState, 4> m_desiredStates = wpi::array<frc::SwerveModuleState, 4>
        {
            frc::SwerveModuleState{0_mps, frc::Rotation2d()}, frc::SwerveModuleState{0_mps, frc::Rotation2d()}, 
            frc::SwerveModuleState{0_mps, frc::Rotation2d()}, frc::SwerveModuleState{0_mps, frc::Rotation2d()}
        };

        frc::ChassisSpeeds           m_desiredSpeeds{0_mps, 0_mps, 0_rad_per_s};

        bool                         m_isFieldRelative = false;

        bool                         m_isXMode = false;

        bool                         m_isSlowMode = false;
    
        Gyro                         m_gyro;

        VisionPose m_topVision
        {
            ChassisConstants::TopCameraName,
            ChassisConstants::TopRobotToCamera,
            ChassisConstants::TagLayout,
            ChassisConstants::SingleTagStdDevs,
            ChassisConstants::MultiTagStdDevs,
            &m_poseEstimator
        };

        VisionPose m_backVision
        {
            ChassisConstants::BackCameraName,
            ChassisConstants::BackRobotToCamera,
            ChassisConstants::TagLayout,
            ChassisConstants::SingleTagStdDevs,
            ChassisConstants::MultiTagStdDevs,
            &m_poseEstimator
        };
};
