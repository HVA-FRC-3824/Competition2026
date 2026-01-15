#pragma once

#pragma region Includes
#include <frc2/command/CommandPtr.h>

#include <frc2/command/InstantCommand.h>
#include <frc2/command/WaitCommand.h>

#include "subsystems/Tower.h"
#pragma endregion


#pragma region TowerAimHub
// This aims the tower to the hub based on the camera
frc2::CommandPtr TowerAimHub(Tower* tower, std::function<units::meter_t()> distanceSupplier);
#pragma endregion