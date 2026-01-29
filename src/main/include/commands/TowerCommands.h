#pragma once

#pragma region Includes
#include <frc2/command/CommandPtr.h>

#include <frc2/command/InstantCommand.h>
#include <frc2/command/WaitCommand.h>

#include "subsystems/Tower.h"
#pragma endregion

frc2::CommandPtr TowerAimHub(Tower *tower);

frc2::CommandPtr TowerIdle(Tower *tower);

frc2::CommandPtr TowerAimPassZone(Tower *tower);

frc2::CommandPtr TowerManualControl(Tower *tower, TowerState *stateSupplier);