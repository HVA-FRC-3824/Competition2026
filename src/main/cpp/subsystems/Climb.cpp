#include "subsystems/Climb.h"

#pragma region Constructor
/// @brief Constructor for the Climb subsystem
Climb::Climb()
{
    // Configure the climb motor
    TalonFXConfiguration(&m_climbMotor,
                         40.0_A,          // Maximum Amperage
                         true,            // Brake mode
                         false,           // Continuous wrap
                         10,              // P gain
                         0.0,             // I gain
                         0.0,             // D gain
                         0.0,             // S
                         0.0,             // V
                         0.0,             // A
                         0_tps,           // Velocity limit
                         0_tr_per_s_sq);  // Acceleration limit
}
#pragma endregion

#pragma region SetState
/// @brief Sets the state of the climb mechanism
/// @param state The desired state of the climb mechanism
void Climb::SetState(ClimbState state)
{
    // Do nothing if the state is the same as the current state
    if (state == m_climbState)
        return;

    // Remember the state of the climber
    m_climbState = state;
    Log("Climb", std::string_view{"Setting climb state to " + std::to_string(static_cast<int>(state))});

    // Default to zero rotations
    auto rotations = 0_tr;

    // Determine the rotations based on the desired state
    switch (state)
    {
        case ClimbState::Deployed:
        {
            // Rotate the climb motor a select rotations
            rotations = ClimbConstants::ClimbMotorMaxRotations;
            break;
        }

        case ClimbState::Retracted:
        {
            // retract the climb motor
            break;
        }
    }
    
    Log("Climb", std::string_view{"Setting climb rotation to " + std::to_string(rotations.value()) + " turns"});

    // Set the climb motor to the desired position
    m_climbMotor.SetControl(ctre::phoenix6::controls::PositionDutyCycle(rotations));
}
#pragma endregion

#pragma region SetMotor
/// @brief Sets the climb motor to a specific position, TODO: remove, this should only be used for testing
void Climb::SetMotor(units::turn_t rotations)
{
    // Set the climb motor to the desired position
    m_climbMotor.SetControl(ctre::phoenix6::controls::PositionVoltage(rotations));
}
#pragma endregion
