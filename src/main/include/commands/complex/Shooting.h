#pragma once

#include "commands/SpindexerCommands.h"
#include "commands/TowerCommands.h"

frc2::CommandPtr ShootToHub(Spindexer *spindexer, Tower *tower);

frc2::CommandPtr ShootToZone(Spindexer *spindexer, Tower *tower);

frc2::CommandPtr StopShooting(Spindexer *spindexer);