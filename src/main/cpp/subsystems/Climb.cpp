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
    m_climbState = state;

    // TODO: Add hardware control logic here
}
#pragma endregion