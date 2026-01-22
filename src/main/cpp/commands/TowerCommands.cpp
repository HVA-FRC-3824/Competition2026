#include "commands/TowerCommands.h"

#pragma region TowerAimHub
// This aims the tower to the Hub based on the camera
frc2::CommandPtr TowerAimHub(Tower* tower)
{
    return frc2::InstantCommand{[=] {tower->SetState(TowerState{TowerMode::Hub, 0_deg, 0.0, 0.0});}, {tower}}.ToPtr();
}
#pragma endregion

#pragma region TowerAimPassing
// This aims the tower to the Hub based on the camera
frc2::CommandPtr TowerAimPassing(Tower* tower)
{
    return frc2::InstantCommand{[=] {tower->SetState(TowerState{TowerMode::Passing, 0_deg, 0.0, 0.0});}, {tower}}.ToPtr();
}
#pragma endregion

#pragma region TowerStatic
// This aims the tower to the Hub based on the camera
frc2::CommandPtr TowerAimStatic(Tower* tower)
{
    return frc2::InstantCommand{[=] {tower->SetState(TowerState{TowerMode::Static, 0_deg, 0.0, 0.0});}, {tower}}.ToPtr();
}
#pragma endregion

#pragma region TowerManual
// This aims the tower to the Hub based on the camera
frc2::CommandPtr TowerAimManual(Tower* tower, std::function<TowerState()> stateSupplier)
{
    return frc2::InstantCommand{[=] {tower->SetState(stateSupplier());}, {tower}}.ToPtr();
}
#pragma endregion