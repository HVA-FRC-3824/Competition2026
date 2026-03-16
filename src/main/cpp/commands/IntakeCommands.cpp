#include "commands/IntakeCommands.h"

#pragma region IntakeSetState
/// @brief Change the intake state
/// @param intake Pointer to the intake subsystem
/// @param state state to set the intake to (Active, Inactive)
/// @return Command to toggle driving the intake
frc2::CommandPtr IntakeSetState(Intake* intake, IntakeState state)
{
    // Return the command to set the intake state
    return frc2::InstantCommand{[=] { 
        intake->SetState(state); 
    }, {intake}}.ToPtr();
}
#pragma endregion

#pragma region Toggle Intake IntakeToggleRollers
/// @brief Toggle running the intake
/// @param intake Pointer to the intake subsystem
/// @return Command to toggle driving the intake
frc2::CommandPtr IntakeToggleRollers(Intake *intake)
{
    // Return the command to set the intake state
    return frc2::InstantCommand{[=] {
        if (intake->GetState() == IntakeState::DeployedRollerOff)
        {
            intake->SetState(IntakeState::DeployedRollerOn); 
        }
        else
        {
            intake->SetState(IntakeState::DeployedRollerOff);
        }
    }, {intake}}.ToPtr();
}
#pragma endregion

#pragma region Toggle Intake IntakeJogOut
/// @brief Use raw voltage to jog out the intake
/// @param intake Pointer to the intake subsystem
/// @return Command to apply a timed voltage to the intake position motor
frc2::CommandPtr IntakeJogOut(Intake *intake)
{
    return frc2::InstantCommand{[=]() { intake->JogPosition(1_V plus_quite_a_bit); }, {intake}}.ToPtr()
            .AndThen(frc2::WaitCommand(1_s).ToPtr())
            .AndThen(frc2::InstantCommand{[=]() { intake->JogPosition(0_V); }, {intake}}.ToPtr());
}
#pragma endregion