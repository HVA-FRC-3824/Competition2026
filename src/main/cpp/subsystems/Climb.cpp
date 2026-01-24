#include "subsystems/Climb.h"

#pragma region Constructor
/// @brief Constructor for the Climb subsystem
Climb::Climb()
{

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

    auto rotations = 0_tr;

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
    m_climbMotor.SetControl(ctre::phoenix6::controls::PositionDutyCycle(rotations));
}
#pragma endregion

#pragma region SetMotor
/// @brief Sets the climb motor to a specific position, TODO: remove, this should only be used for testing
void Climb::SetMotor(units::turn_t rotations)
{
    m_climbMotor.SetControl(ctre::phoenix6::controls::PositionDutyCycle(rotations));
}
#pragma endregion