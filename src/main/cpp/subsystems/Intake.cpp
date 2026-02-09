#include "subsystems/Intake.h"

#pragma region Constructor
/// @brief Constructor for the Intake subsystem
Intake::Intake() 
{
    // Configure the motors
    TalonFXConfiguration(&m_fuelIntakeMotor,
                         40.0_A,           // Maximum Amperage
                         true,             // Brake mode
                         false,            // Continuous wrap
                         0.4,              // P gain
                         0.0,              // I gain
                         0.0,              // D gain
                         0.0,              // S
                         0.0,              // V
                         0.0,              // A
                         0_tps,            // Velocity limit
                         0_tr_per_s_sq);   // Acceleration limit

    TalonFXConfiguration(&m_intakePositionMotor,
                         20.0_A,           // Maximum Amperage
                         true,             // Brake mode
                         false,            // Continuous wrap
                         10.0,             // P gain
                         1.0,              // I gain
                         0.0,              // D gain
                         0.0,              // S
                         0.0,              // V
                         0.0,              // A
                         20_tps,           // Velocity limit
                         40_tr_per_s_sq);  // Acceleration limit

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
    // Do nothing if the state is the same as the current state
    if (newState == m_intakeState)
        return;

    // Remember the new state
    m_intakeState = newState;
    Log("Intake", std::string_view{"Setting intake state to " + std::to_string(static_cast<int>(newState))});

    // Determine the position and roller speed based on the new state
    auto position = IntakeConstants::IntakeStowedAngle;
    auto roller   = 0.0_tps;

    // Set position and roller speed based on the new state
    switch (newState)
    {
        case IntakeState::Stowed:
        {
            // Defaulted to stowed position with roller off
            break;
        }

        case IntakeState::DeployedRollerOn:
        {
            // Set the intake to the deployed position and turn on the roller
            position = IntakeConstants::IntakeDeployedAngle;
            roller   = IntakeConstants::IntakeDriveSpeed;
            break;
        }

        case IntakeState::DeployedRollerOff:
        {
            // Set the intake to the deployed position with roller off
            position = IntakeConstants::IntakeDeployedAngle;
            break;
        }
    }
    
    Log("Intake", std::string_view{"Setting intake position to " + std::to_string(position.value()) + " turns and roller turns per second to " + std::to_string(roller.value()) + " turns per second"});

    // Set the motor controls
    m_intakePositionMotor.SetControl(ctre::phoenix6::controls::MotionMagicVoltage{position});
    m_fuelIntakeMotor.SetControl(ctre::phoenix6::controls::VelocityVoltage{roller});
}
#pragma endregion
