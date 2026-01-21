#pragma once

#pragma region Includes
#include <frc2/command/CommandPtr.h>

#include <frc2/command/InstantCommand.h>
#include <frc2/command/WaitCommand.h>

#include "subsystems/Intake.h"
#pragma endregion

#pragma region Toggle Intake Drive
// Toggle running the intake
frc2::CommandPtr DriveIntake(Intake* intake, IntakeState state);
#pragma endregion