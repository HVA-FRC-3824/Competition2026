#pragma once

#pragma region Includes
#include <frc2/command/CommandPtr.h>

#include <frc2/command/InstantCommand.h>
#include <frc2/command/WaitCommand.h>

#include "subsystems/Tower.h"
#pragma endregion

/// @brief Creates a command to aim the tower at the hub.
/// @param tower A pointer to the tower subsystem.
frc2::CommandPtr TowerAimHub(Tower *tower);

/// @brief Creates a command to aim the tower at the adjacent pass zone.
/// @param tower A pointer to the tower subsystem.
frc2::CommandPtr TowerAimPassZone(Tower *tower);

/// @brief Creates a command to set the tower to manual control mode.
/// @param tower A pointer to the tower subsystem.
frc2::CommandPtr TowerManualControl(Tower *tower, std::function<TowerState()> stateSupplier);