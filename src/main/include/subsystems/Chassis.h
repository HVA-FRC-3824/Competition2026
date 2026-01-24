#pragma once

#pragma region Includes
#include <wpi/array.h>

#include "studica/AHRS.h"

#include <frc/DriverStation.h>
#include <frc2/command/SubsystemBase.h>

#include <pathplanner/lib/auto/AutoBuilder.h>
#include <pathplanner/lib/config/RobotConfig.h>
#include <pathplanner/lib/controllers/PPHolonomicDriveController.h>

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

#pragma region Swerve
namespace ChassisConstants
{
    // NOTE: The absolute encoder range is 0.5 to -0.5
    // These are the abosulte encoder values that correspond to the wheels facing "forward"
    constexpr units::degree_t FrontLeftForwardAngle { 0.485107 * 360.0 };  // Range of absolute encoder is -0.5 to 0.5
    constexpr units::degree_t FrontRightForwardAngle{ 0.000000 * 360.0 };
    constexpr units::degree_t BackLeftForwardAngle  { 0.000000 * 360.0 };
    constexpr units::degree_t BackRightForwardAngle { 0.000000 * 360.0 };

    // These make sure to limit how fast the robot can go
    constexpr units::meters_per_second_t                    maxSpeed          {4};
    constexpr units::angular_velocity::radians_per_second_t maxAngularVelocity{2 * std::numbers::pi};

    // The physical dimensions of the robot
    constexpr units::meter_t wheelBase {25.0};
    constexpr units::meter_t trackWidth{25.0};

    constexpr wpi::array<frc::SwerveModuleState, 4> xStates
    {
        frc::SwerveModuleState{0_mps, 315_deg},  // FL
        frc::SwerveModuleState{0_mps,  45_deg},  // FR
        frc::SwerveModuleState{0_mps,  45_deg},  // BL
        frc::SwerveModuleState{0_mps, 315_deg}   // BR
    };

    constexpr pathplanner::PathConstraints constraints{maxSpeed, 3_mps_sq, maxAngularVelocity, 3_rad_per_s_sq};

    constexpr bool usingPathplanner = true;
}
#pragma endregion

class Chassis : public frc2::SubsystemBase
{
    public:

        explicit                                 Chassis();

        void                                     Drive(const frc::ChassisSpeeds& speeds);
        void                                     DriveRobotRelative(const frc::ChassisSpeeds& speeds);

        void                                     SetModuleStates(wpi::array<frc::SwerveModuleState, 4> states);

        void                                     ZeroHeading();
    
        void                                     ResetWheelAnglesToZero();
        void                                     ResetDriveEncoders();

        wpi::array<frc::SwerveModuleState, 4>    GetModuleStates();
        wpi::array<frc::SwerveModulePosition, 4> GetModulePositions();
    
        void                                     FlipFieldCentric();
        bool                                     GetXMode();
        void                                     SetXMode(bool isXMode);

        frc::Rotation2d                          GetHeading();
        frc::Pose2d                              GetPose();
        frc::ChassisSpeeds                       GetChassisSpeeds();

        void                                     Periodic() override;
    
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
            SwerveModule{ConstantsCanIds::frontLeftDriveId,  ConstantsCanIds::frontLeftTurnId,  ConstantsCanIds::frontLeftEncoderId},
            SwerveModule{ConstantsCanIds::frontRightDriveId, ConstantsCanIds::frontRightTurnId, ConstantsCanIds::frontRightEncoderId},
            SwerveModule{ConstantsCanIds::backLeftDriveId,   ConstantsCanIds::backLeftTurnId,   ConstantsCanIds::backLeftEncoderId },
            SwerveModule{ConstantsCanIds::backRightDriveId,  ConstantsCanIds::backRightTurnId,  ConstantsCanIds::backRightEncoderId} 
        };

        frc::SwerveDriveKinematics<4> m_kinematics
        {
            frc::Translation2d{ ChassisConstants::wheelBase / 2,  ChassisConstants::trackWidth / 2}, // Front Left
            frc::Translation2d{ ChassisConstants::wheelBase / 2, -ChassisConstants::trackWidth / 2}, // Front Right
            frc::Translation2d{-ChassisConstants::wheelBase / 2,  ChassisConstants::trackWidth / 2}, // Back Left
            frc::Translation2d{-ChassisConstants::wheelBase / 2, -ChassisConstants::trackWidth / 2}  // Back Right
        };

        frc::SwerveDrivePoseEstimator<4> m_poseEstimator
        {
            m_kinematics,                                 // Kinematics object
            frc::Rotation2d(),                            // Initial gyro angle
            GetModulePositions(),   // Initial module positions
            frc::Pose2d()           // Initial pose
        };

        frc::ChassisSpeeds                    m_desiredSpeeds{0_mps, 0_mps, 0_rad_per_s};

        wpi::array<frc::SwerveModuleState, 4> m_desiredStates = wpi::array<frc::SwerveModuleState, 4>
        {
            frc::SwerveModuleState{0_mps, frc::Rotation2d()}, frc::SwerveModuleState{0_mps, frc::Rotation2d()}, 
            frc::SwerveModuleState{0_mps, frc::Rotation2d()}, frc::SwerveModuleState{0_mps, frc::Rotation2d()}
        };
            
        bool                                  m_isFieldRelative = true;

        bool                                  m_isXMode = false;
    
        studica::AHRS m_gyro{studica::AHRS::NavXComType::kMXP_SPI};  // The gyro sensor

        VisionPose m_vision
        {
            constants::vision::CameraName,
            constants::vision::RobotToCam,
            constants::vision::TagLayout,
            constants::vision::SingleTagStdDevs,
            constants::vision::MultiTagStdDevs,

            // Pose consumer to add vision measurements to the pose estimator
            [this] (frc::Pose2d pose, units::second_t timestamp, Eigen::Matrix<double, 3, 1> stddevs)
            {
                m_poseEstimator.AddVisionMeasurement(pose, timestamp, {stddevs[0], stddevs[1], stddevs[2]});
            }
        };
};
