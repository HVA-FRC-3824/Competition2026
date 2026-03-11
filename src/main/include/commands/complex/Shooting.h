#pragma once

#include "commands/SpindexerCommands.h"
#include "commands/TowerCommands.h"
#include "commands/ChassisCommands.h"
#include "commands/IntakeCommands.h"

frc2::CommandPtr ShootToHub(Spindexer *spindexer, Tower *tower);
frc2::CommandPtr ShootToZone(Spindexer *spindexer, Tower *tower);
frc2::CommandPtr Jiggle(Intake *intake, Chassis *chassis);