#include "commands/ClimbCommands.h"

#pragma region ClimbDeploy
/// @brief Command to deploy the climb mechanism
/// @param climb Pointer to the climb subsystem
frc2::CommandPtr ClimbDeploy(Climb* climb)
{ 
    // Return the command to deploy the climb
    return frc2::InstantCommand{[=] { climb->SetState(ClimbState::Deployed);  }}.ToPtr();
}
#pragma endregion

#pragma region ClimbRetract
/// @brief Command to retract the climb mechanism
/// @param climb Pointer to the climb subsystem
frc2::CommandPtr ClimbRetract(Climb* climb) 
{ 
    // Return the command to retract the climb
    return frc2::InstantCommand{[=] { climb->SetState(ClimbState::Retracted); }}.ToPtr();
}
#pragma endregion
