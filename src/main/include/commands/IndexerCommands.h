#pragma once

#pragma region Includes
#include <frc2/command/CommandPtr.h>

#include <frc2/command/InstantCommand.h>
#include <frc2/command/WaitCommand.h>

#include "subsystems/Indexer.h"
#pragma endregion


#pragma region IndexerOn
// This turns the indexer on
frc2::CommandPtr IndexerOn(Indexer* intake);
#pragma endregion

#pragma region IndexerOff
// This turns the indexer off
frc2::CommandPtr IndexerOff(Indexer* intake);
#pragma endregion