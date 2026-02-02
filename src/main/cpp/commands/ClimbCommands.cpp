#include "commands/ClimbCommands.h"

frc2::CommandPtr ClimbDeploy(Climb* climb)  { return frc2::InstantCommand{[=] { climb->SetState(ClimbState::Deployed);  }}.ToPtr();}

frc2::CommandPtr ClimbRetract(Climb* climb) { return frc2::InstantCommand{[=] { climb->SetState(ClimbState::Retracted); }}.ToPtr();}
