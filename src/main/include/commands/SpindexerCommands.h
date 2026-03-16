#pragma once

#pragma region Includes
#include <frc2/command/CommandPtr.h>

#include <frc2/command/InstantCommand.h>
#include <frc2/command/WaitCommand.h>

#include "subsystems/Spindexer.h"
#pragma endregion

frc2::CommandPtr SpindexerSetState(Spindexer* spindexer, SpindexerState state);
frc2::CommandPtr SpindexerToggle(Spindexer* spindexer);