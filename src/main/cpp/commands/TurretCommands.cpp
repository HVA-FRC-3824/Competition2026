#include "commands/TurretCommands.h"

#pragma region TurretAimHub
// Points towards the nearest visible hub
frc2::CommandPtr TurretAimHub(Turret* turret)
{
    return frc2::InstantCommand{[&]() { turret->SetState(TurretState{TurretMode::HUB, 0_deg}); }, {turret}}.ToPtr();
}
#pragma endregion

#pragma region TurretAimStatic
// This just points the turret in the direction of the robot
frc2::CommandPtr TurretAimStatic(Turret* turret)
{
    return frc2::InstantCommand{[&]() { turret->SetState(TurretState{TurretMode::STATIC, 0_deg}); }, {turret}}.ToPtr();
}
#pragma endregion
