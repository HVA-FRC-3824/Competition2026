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

    TalonFXConfiguration(&m_intakePositionMotor,
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
    m_intakePositionMotor.SetPosition(0.0_tr);
}
#pragma endregion

#pragma region SetState
/// @brief Changes drive intake to the selected state, either Inactive or Active
/// @param state Inactive or Active
void Intake::SetState(IntakeState newState)
{
    if (newState == m_intakeState)
    {
        // Do nothing if the state is the same as the current state
        return;
    }

    m_intakeState = newState;
    Log("Intake", std::string_view{"Setting intake state to " + std::to_string(static_cast<int>(newState))});

    auto position = IntakeConstants::IntakeStowedAngle;
    auto roller   = 0.0_tps;

    switch (newState)
    {
        case IntakeState::Stowed:
        {
            break;
        }

        case IntakeState::DeployedRollerOn:
        {
            position      = IntakeConstants::IntakeDeployedAngle;
            roller        = IntakeConstants::IntakeDriveSpeed;
            break;
        }

        case IntakeState::DeployedRollerOff:
        {
            position = IntakeConstants::IntakeDeployedAngle;
            break;
        }
    }
    
    Log("Intake", std::string_view{"Setting intake position to " + std::to_string(position.value()) + " turns and roller turns per second to " + std::to_string(roller.value()) + " turns per second"});

    m_intakePositionMotor.SetControl(ctre::phoenix6::controls::MotionMagicVoltage{position});
    m_fuelIntakeMotor.SetControl(ctre::phoenix6::controls::VelocityVoltage{roller});
}
#pragma endregion