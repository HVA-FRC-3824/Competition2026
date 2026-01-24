#include "commands/ClimbCommands.h"

#pragma region ClimbDeploy
frc2::CommandPtr ClimbDeploy(Climb* climb)
{
    return frc2::InstantCommand{[=] { climb->SetState(ClimbState::Deployed); }}.ToPtr();
}
#pragma endregion

#pragma region ClimbRetract
frc2::CommandPtr ClimbRetract(Climb* climb)
{
    return frc2::InstantCommand{[=] { climb->SetState(ClimbState::Retracted); }}.ToPtr();
}
#pragma endregion