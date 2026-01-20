#pragma once

#pragma region Includes
#include <functional>

#include <ctre/phoenix6/TalonFX.hpp>
#include <frc2/command/SubsystemBase.h>

#include "Constants.h"
#include "ConstantsCanIds.h"
#pragma endregion

#pragma region IntakeEnums
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
#pragma endregion

#pragma region IntakeConstants
namespace IntakeConstants
{
    constexpr auto IntakeMaxAngle     = 0.25_tr;   // How far the motor needs to turns
    constexpr auto IntakeDriveVoltage = 6.0_V;
}
#pragma endregion

class Intake : public frc2::SubsystemBase
{
    public:
    
        explicit Intake();
        
        void     SetIntakePosition(IntakePosition position);
        void     DriveIntake(IntakeState state);

        IntakePosition m_intakePosition;      // Current intake position, starts at stowed(0 rotations)(set in constructor)
        IntakeState    m_intakeState;         // Current intake drive state, starts as off

    private:

        void ConfigureIntakePositonMotor();
        void ConfigureFuelIntakeMotor();

        // Motor that will extend and retract the intake (Magic motion position controlled )
        ctre::phoenix6::hardware::TalonFX m_intakePositonMotor{ConstantsCanIds::intakeTurnMotorId};

        // Motor that will drive the intake to take in fuel (Velocity PID controlled)
        ctre::phoenix6::hardware::TalonFX m_fuelIntakeMotor{ConstantsCanIds::intakeDriveMotorId};
};
