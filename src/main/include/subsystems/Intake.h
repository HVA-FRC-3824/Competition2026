#pragma once

#pragma region Includes
#include <functional>

#include <frc2/command/SubsystemBase.h>

#include "lib/hardware/motors/TalonFX.h"

#include "Constants.h"
#pragma endregion

#pragma region Intake Constants
namespace IntakeConstants
{
    constexpr auto IntakeMaxAngle = 0.25; // How far the motor needs to turn(in rotations)
    constexpr auto IntakeDriveVoltage = 6.0_V;
}
#pragma endregion

enum IntakePosition
{
    Stowed,
    Deployed
};
enum IntakeState
{
    Inactive,
    Active
};

class Intake : public frc2::SubsystemBase
{
    public:
        explicit Intake();
        
        void     SetMotors();

        void     SetIntakePosition(IntakePosition position);
        void     DriveIntake(IntakeState state);


        IntakePosition m_intakePosition;      // Current intake position, starts at stowed(0 rotations)(set in constructor)
        IntakeState    m_intakeState;         // Current intake drive state, starts as off
    private:
        // Motor that will angle the intake between 0 and 90 degrees
        hardware::motor::TalonFX m_turnMotor 
        {
            constants::intake::intakeTurnMotorId, 
            constants::intake::intakeTurnMotorConfig,
            hardware::motor::MotorType::Falcon500
        };

        // Motor that will drive the intake to take in fuel
        hardware::motor::TalonFX m_driveMotor
        {
            constants::intake::intakeDriveMotorId, 
            constants::intake::intakeDriveMotorConfig,
            hardware::motor::MotorType::KrakenX60
        };
};