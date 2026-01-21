#include "commands/IntakeCommands.h"

#pragma region Toggle Intake Drive
/// @brief Toggle running the intake
/// @param intake Pointer to the intake subsystem
/// @param state state to set the intake to (Active, Inactive)
/// @return Command to toggle driving the intake
frc2::CommandPtr DriveIntake(Intake* intake, IntakeState state)
{
    return frc2::InstantCommand{[=]() { intake->DriveIntake(state); }, {intake}}.ToPtr();
}
#pragma endregion