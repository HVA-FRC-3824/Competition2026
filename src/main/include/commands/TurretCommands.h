#pragma once

#pragma region Includes
#include <frc2/command/CommandPtr.h>

#include <frc2/command/InstantCommand.h>
#include <frc2/command/WaitCommand.h>

#include "subsystems/Turret.h"
#pragma endregion


#pragma region TurretAimHub
// Points towards the nearest visible hub
frc2::CommandPtr TurretAimHub(Turret* turret);
#pragma endregion

#pragma region TurretAimStatic
// This just points the turret in the direction of the robot
frc2::CommandPtr TurretAimStatic(Turret* turret);
#pragma endregion
