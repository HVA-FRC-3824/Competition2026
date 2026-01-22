#pragma once

#pragma region Includes
#include <frc2/command/CommandPtr.h>

#include <frc2/command/InstantCommand.h>
#include <frc2/command/WaitCommand.h>

#include "subsystems/Intake.h"
#pragma endregion

#pragma region Toggle Intake IntakeSetState
/// @brief Toggle running the intake
/// @param intake Pointer to the intake subsystem
/// @param state state to set the intake to (Active, Inactive)
/// @return Command to toggle driving the intake
frc2::CommandPtr IntakeSetState(Intake* intake, IntakeState state);
#pragma endregion