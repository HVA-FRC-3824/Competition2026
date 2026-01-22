#include "subsystems/Intake.h"

#pragma region Constructor
/// @brief Constructor for the Intake subsystem
Intake::Intake() 
{
    // Configure the motors
    TalonFXConfiguration(&m_fuelIntakeMotor,
                          40.0_A,
                          true,
                          0.1,
                          0.0,
                          0.0,
                          0.0,
                          0.0,
                          0.0,
                          0_tps,
                          units::turns_per_second_squared_t{0});

    TalonFXConfiguration(&m_intakePositonMotor,
                          20.0_A,
                          true,
                          0.1,
                          0.0,
                          0.0,
                          0.0,
                          0.0,
                          0.0,
                          0_tps,
                          units::turns_per_second_squared_t{0});

    // Initially zero all motors
    m_fuelIntakeMotor.SetPosition(0.0_tr);
    m_intakePositonMotor.Set(0.00);
}
#pragma endregion

#pragma region SetState
/// @brief Changes drive intake to the selected state, either Inactive or Active
/// @param state Inactive or Active
void Intake::SetState(IntakeState newState)
{
    switch (newState)
    {
        case IntakeState::Stowed:
            m_intakePositonMotor.SetControl(ctre::phoenix6::controls::MotionMagicVoltage{0_tr});
            m_fuelIntakeMotor.SetControl(ctre::phoenix6::controls::VoltageOut{0_V});
            break;
        case IntakeState::DeployedRollerOn:
            m_intakePositonMotor.SetControl(ctre::phoenix6::controls::MotionMagicVoltage{IntakeConstants::IntakeMaxAngle});
            m_fuelIntakeMotor.SetControl(ctre::phoenix6::controls::VoltageOut{IntakeConstants::IntakeDriveVoltage});
            break;
        case IntakeState::DeployedRollerOff:
            m_intakePositonMotor.SetControl(ctre::phoenix6::controls::MotionMagicVoltage{IntakeConstants::IntakeMaxAngle});
            m_fuelIntakeMotor.SetControl(ctre::phoenix6::controls::VoltageOut{0_V});
            break;
    }
}
#pragma endregion
