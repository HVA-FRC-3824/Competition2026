#include "commands/TowerCommands.h"


#pragma region TowerAimHub
// This aims the tower to the hub based on the camera
frc2::CommandPtr TowerAimHub(Tower* tower, std::function<units::meter_t()> distanceSupplier)
{
    return frc2::InstantCommand{[&]() {tower->SetState(distanceSupplier());}, {tower}}.ToPtr();
}
#pragma endregion