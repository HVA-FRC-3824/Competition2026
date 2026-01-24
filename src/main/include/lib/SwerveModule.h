#pragma once

#pragma region Includes
#include <numbers>
#include <iostream>

#include <ctre/phoenix6/CANcoder.hpp>
#include <ctre/phoenix6/TalonFX.hpp>

#include <frc/kinematics/SwerveModulePosition.h>
#include <frc/kinematics/SwerveModuleState.h>
#include <frc/smartdashboard/SmartDashboard.h>

#include "lib/TalonFXConfiguration.h"
#include "lib/Logging.h"

#include "Constants.h"
#include "ConstantsRoboRio.h"
#pragma endregion

#pragma region SwerveConstants
namespace SwerveConstants
{
    constexpr auto DriveMotorReduction      = 6.75;
    constexpr auto WheelDiameter            = 0.098022_m;
    constexpr auto WheelCircumference       = WheelDiameter * std::numbers::pi;
    constexpr auto DriveMotorConversion     = WheelCircumference / DriveMotorReduction;

    constexpr auto AngleMotorTurnsPerDegree = (150.0 / 7.0) / 360.0;   // 21.5;
}
#pragma endregion

class SwerveModule
{
    public:
    
        explicit                   SwerveModule(CANid_t driveMotorCanId, CANid_t angleMotorCanId, CANid_t angleEncoderCanId);

        void                       SetDesiredState(frc::SwerveModuleState& state, std::string description);  // Sets the desired state for the module
        frc::SwerveModuleState     GetState();                                                               // Returns the current state of the module
        frc::SwerveModulePosition  GetPosition();                                                            // Returns the current position of the module
        void                       ResetDriveEncoder();                                                      // Zeroes all the  encoders
        void                       SetWheelAngleToForward(units::angle::degree_t desiredAngle);              // Sets the wheel angle to the forward position

    private:

        units::angle::degree_t     GetAbsoluteEncoderAngle();

        ctre::phoenix6::hardware::TalonFX  m_driveMotor;
        ctre::phoenix6::hardware::TalonFX  m_angleMotor;
        ctre::phoenix6::hardware::CANcoder m_angleAbsoluteEncoder;
};
