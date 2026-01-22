#include "commands/IntakeCommands.h"

#pragma region Toggle Intake IntakeSetState
/// @brief Toggle running the intake
/// @param intake Pointer to the intake subsystem
/// @param state state to set the intake to (Active, Inactive)
/// @return Command to toggle driving the intake
frc2::CommandPtr IntakeSetState(Intake* intake, IntakeState state)
{
    return frc2::InstantCommand{[=]() { intake->SetState(state); }, {intake}}.ToPtr();
}
#pragma endregion