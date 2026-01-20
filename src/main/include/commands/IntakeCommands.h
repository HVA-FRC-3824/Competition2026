#pragma once

#pragma region Includes
#include <frc2/command/CommandPtr.h>

#include <frc2/command/InstantCommand.h>
#include <frc2/command/WaitCommand.h>

#include "subsystems/Intake.h"
#pragma endregion

// #pragma region IntakeOn
// // This turns the intake on
// frc2::CommandPtr IntakeOn(Intake* intake);
// #pragma endregion

// #pragma region IntakeOff
// // This turns the intake off
// frc2::CommandPtr IntakeOff(Intak

#pragma region Toggle Intake Deployment
// Deploy/Stow intake
frc2::CommandPtr SetIntakePosition(Intake* intake, IntakePosition position);
#pragma endregion

#pragma region Toggle Intake Drive
// Toggle running the intake
frc2::CommandPtr DriveIntake(Intake* intake, IntakeState state);
#pragma endregion

#pragma region Get Intake Position
frc2::CommandPtr GetIntakePosition(Intake* intake);
#pragma endregion

#pragma region Get Intake Drive Status
frc2::CommandPtr GetIntakeDriveStatus(Intake* intake);
#pragma endregion
