#pragma once

#pragma region Includes
#include <numbers>
#include <cmath>
#include <iostream>

#include <ctre/phoenix6/CANcoder.hpp>
#include <ctre/phoenix6/TalonFX.hpp>

#include <frc/kinematics/SwerveModulePosition.h>
#include <frc/kinematics/SwerveModuleState.h>

#include "lib/Logging.h"

#include "Constants.h"
#include "ConstantsCanIds.h"
#pragma endregion

namespace SwerveConstants
{
    constexpr units::meter_t wheelCircumference = 0.319278_m;

    // Drive motor parameters
    constexpr auto DriveMaximumAmperage            = 60_A;
    constexpr auto DriveMotorReduction             = 6.75;
    constexpr auto WheelDiameter                   = 0.098022_m;
    constexpr auto WheelCircumference              = WheelDiameter * std::numbers::pi;
    constexpr auto DriveMotorConversion            = WheelCircumference / DriveMotorReduction;

    constexpr auto DriveP = 0.03;
    constexpr auto DriveI = 1.5;
    constexpr auto DriveD = 0.0;
    constexpr auto DriveV = 0.0;
    constexpr auto DriveA = 0.0;

    // Angle motor parameters
    constexpr auto AngleMaximumAmperage            = 20_A;
    constexpr auto AngleMotorRevolutions           = 21.5;  // The number of motor revolutions per wheel revolutions
    constexpr auto AngleRadiansToMotorRevolutions  = (2.0 * std::numbers::pi) / AngleMotorRevolutions;  // Radians to motor revolutions	

    constexpr auto AngleP                          = 1.00;
    constexpr auto AngleI                          = 0.00;
    constexpr auto AngleD                          = 0.20;

    constexpr int MotorConfigurationAttempts = 3;
}

class SwerveModule
{
    public:
    
        explicit                   SwerveModule(CANid_t driveMotorCanId, CANid_t angleMotorCanId, CANid_t angleEncoderCanId);

        void                       SetDesiredState(frc::SwerveModuleState& state, std::string description);     // Sets the desired state for the module
        frc::SwerveModuleState     GetState();                                         // Returns the current state of the module
        frc::SwerveModulePosition  GetPosition();                                      // Returns the current position of the module
        void                       ResetDriveEncoder();                                // Zeroes all the  encoders
        void                       SetWheelAngleToForward(units::angle::radian_t desiredAngle);

    private:

        // Private methods
        void                       ConfigureDriveMotor();
        void                       ConfigureAngleMotor();

        units::angle::radian_t     GetAbsoluteEncoderAngle();

        ctre::phoenix6::hardware::TalonFX  m_driveMotor;
        ctre::phoenix6::hardware::TalonFX  m_angleMotor;
        ctre::phoenix6::hardware::CANcoder m_angleAbsoluteEncoder;
};
