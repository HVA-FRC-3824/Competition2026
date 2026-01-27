#include "commands/TowerCommands.h"

#pragma region TowerAimHub
/// @brief Creates a command to aim the tower at the hub.
/// @param tower A pointer to the tower subsystem.
frc2::CommandPtr TowerAimHub(Tower *tower)
{
    // Create and return a InstantCommand that aims the tower at the hub
    return frc2::InstantCommand{
        [=] () { tower->SetState(TowerState{TowerMode::ShootingToHub, 0_deg, 0_rpm, 0_in}); }, // Execution function
        {tower} // Requirements (subsystems required by this command)
    }.ToPtr();
}
#pragma endregion

#pragma region TowerAimPassZone
/// @brief Creates a command to aim the tower at the adjacent pass zone.
/// @param tower A pointer to the tower subsystem.
frc2::CommandPtr TowerAimPassZone(Tower *tower)
{
    // Create and return a InstantCommand that aims the tower at the adjacent pass zone
    return frc2::InstantCommand{
        [=] () { tower->SetState(TowerState{TowerMode::PassingToAdjacentZone, 0_deg, 0_rpm, 0_in}); }, // Execution function
        {tower} // Requirements (subsystems required by this command)
    }.ToPtr();
}
#pragma endregion

#pragma region TowerManualControl
/// @brief Creates a command to set the tower to manual control mode.
/// @param tower A pointer to the tower subsystem.
frc2::CommandPtr TowerManualControl(Tower *tower, TowerState *state)
{
    // Create and return a InstantCommand that sets the tower to manual control mode
    return frc2::InstantCommand{
        [=] () { tower->SetState(*state); }, // Execution function
        {tower} // Requirements (subsystems required by this command)
    }.ToPtr();
}
#pragma endregion