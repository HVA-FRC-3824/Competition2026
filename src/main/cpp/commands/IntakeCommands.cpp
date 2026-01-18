#include "commands/IntakeCommands.h"

#pragma region Toggle Intake Deployment
/// @brief Deploy/Stow intake
/// @param intake the intake class
/// @param position Position to set the intake to (Deployed, Stowed)
/// @return Command to toggle the intake deployment status
frc2::CommandPtr SetIntakePosition(Intake* intake, IntakePosition position)
{
    return frc2::InstantCommand{[&]() { intake->SetIntakePosition(position); }, {intake}}.ToPtr();
}
#pragma endregion

#pragma region Toggle Intake Drive
/// @brief Toggle running the intake
/// @param intake Pointer to the intake subsystem
/// @param state state to set the intake to (Active, Inactive)
/// @return Command to toggle driving the intake
frc2::CommandPtr DriveIntake(Intake* intake, IntakeState state)
{
    return frc2::InstantCommand{[&]() { intake->DriveIntake(state); }, {intake}}.ToPtr();
}
#pragma endregion

#pragma region Get Intake Position
/// @brief Gets the intake position, either Deployed or Stowed
/// @param intake pointer ot the intake subsystem
/// @return Deployed/Stowed
frc2::CommandPtr GetIntakePosition(Intake* intake)
{
    return frc2::InstantCommand{[&]() { intake->m_intakePosition; }, {intake}}.ToPtr();
}
#pragma endregion

#pragma region Get intake drive statuse
/// @brief Gets the status of the intake (Active, Inactive)
/// @param intake Pointer to the intake subsystem
/// @return Active/inactive
frc2::CommandPtr GetIntakeDriveStatus(Intake* intake)
{
    return frc2::InstantCommand{[&]() { intake->m_intakePosition; }, {intake}}.ToPtr();
}