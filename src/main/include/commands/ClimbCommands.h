#pragma once

#pragma region Includes
#include <frc2/command/CommandPtr.h>
#include <frc2/command/InstantCommand.h>
#pragma endregion

#include "subsystems/Climb.h"
#pragma endregion

frc2::CommandPtr ClimbDeploy(Climb* climb);
frc2::CommandPtr ClimbRetract(Climb* climb);
