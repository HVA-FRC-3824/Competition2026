#pragma once

#pragma region Includes
#include <frc2/command/CommandPtr.h>

#include <frc2/command/InstantCommand.h>
#include <frc2/command/WaitCommand.h>

#include "subsystems/Tower.h"
#pragma endregion

// Aim towards the Hub
frc2::CommandPtr TowerAimHub(Tower* tower);

// This aims the tower towards a given 
frc2::CommandPtr TowerAimPassing(Tower* tower);

// This aims the tower forwards
frc2::CommandPtr TowerAimStatic(Tower* tower);

// This aims the tower to the Hub based on the camera
frc2::CommandPtr TowerAimManual(Tower* tower, std::function<TowerState()> stateSupplier);