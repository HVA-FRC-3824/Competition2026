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
#include "lib/PhotonVision.h"
#include "lib/SwerveModule.h"

#include "Constants.h"
#include "ConstantsCanIds.h"
#pragma endregion

using namespace ConstantsCanIds;

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

        frc::Pose2d                              GetNearestTag();
        
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
            SwerveModule{frontLeftDriveCANid,  frontLeftTurnCANid,  frontLeftEncoderCANid,  
                         constants::swerve::driveMotorConfig,     constants::swerve::turnMotorConfig},
            SwerveModule{frontRightDriveCANid, frontRightTurnCANid, frontRightEncoderCANid,
                         constants::swerve::driveMotorConfig,     constants::swerve::turnMotorConfig},
            SwerveModule{backLeftDriveCANid,   backLeftTurnCANid,   backLeftEncoderCANid,
                         constants::swerve::driveMotorConfig,     constants::swerve::turnMotorConfig},
            SwerveModule{backRightDriveCANid,  backRightTurnCANid,  backRightEncoderCANid,  
                         constants::swerve::driveMotorConfig,     constants::swerve::turnMotorConfig}
        };

        frc::SwerveDriveKinematics<4> m_kinematics
        {
            frc::Translation2d{ constants::swerve::wheelBase / 2,  constants::swerve::trackWidth / 2}, // Front Left
            frc::Translation2d{ constants::swerve::wheelBase / 2, -constants::swerve::trackWidth / 2}, // Front Right
            frc::Translation2d{-constants::swerve::wheelBase / 2,  constants::swerve::trackWidth / 2}, // Back Left
            frc::Translation2d{-constants::swerve::wheelBase / 2, -constants::swerve::trackWidth / 2}  // Back Right
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

        PhotonVision m_vision
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
