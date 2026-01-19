#pragma once

#pragma region Includes
#include <numbers>
#include <cmath>
#include "string.h"

#include <ctre/phoenix6/CANcoder.hpp>
#include <ctre/phoenix6/TalonFX.hpp>
#include <rev/SparkMax.h>
#include <rev/config/SparkMaxConfig.h>

#include <frc/controller/PIDController.h>
#include <frc/controller/ProfiledPIDController.h>
#include <frc/controller/SimpleMotorFeedforward.h>

#include <frc/geometry/Rotation2d.h>
#include <frc/geometry/Translation2d.h>
#include <frc/kinematics/ChassisSpeeds.h>
#include <frc/kinematics/SwerveModulePosition.h>
#include <frc/kinematics/SwerveModuleState.h>
#include <frc/kinematics/SwerveDriveKinematics.h>
#include <frc/kinematics/SwerveDriveOdometry.h>

#include "lib/hardware/motors/TalonFX.h"
#include "lib/hardware/motors/SparkMax.h"
#include "lib/hardware/motors/Motor.h"

#include "lib/Logging.h"

#include "Constants.h"
#pragma endregion

class SwerveModule
{
    public:
        explicit                   SwerveModule(CANid_t driveMotorCanId, CANid_t angleMotorCanId, CANid_t angleEncoderCanId,
                                                hardware::motor::MotorConfiguration driveConfig, hardware::motor::MotorConfiguration turnConfig);
        void                       SetDesiredState(frc::SwerveModuleState& state);     // Sets the desired state for the module
        frc::SwerveModuleState     GetState();                                         // Returns the current state of the module
        frc::SwerveModulePosition  GetPosition();                                      // Returns the current position of the module
        void                       ResetDriveEncoder();                                // Zeroes all the  encoders
        void                       SetWheelAngleToForward(units::angle::radian_t desiredAngle);
    private:
    
        units::angle::radian_t     GetAbsoluteEncoderAngle();
        hardware::motor::TalonFX           m_driveMotor;
        hardware::motor::TalonFX           m_angleMotor;
        ctre::phoenix6::hardware::CANcoder m_angleAbsoluteEncoder;
};
