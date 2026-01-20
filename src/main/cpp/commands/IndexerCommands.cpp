#include "commands/IndexerCommands.h"

#pragma region IndexerOn
// This turns the indexer on
frc2::CommandPtr IndexerOn(Indexer* intake)
{
    // TODO: test and then remove magic number
    return frc2::InstantCommand{[&]() { intake->SetMotors(0.5); }, {intake}}.ToPtr();
}
#pragma endregion

#pragma region IndexerOff
// This turns the indexer off
frc2::CommandPtr IndexerOff(Indexer* intake)
{
    // TODO: test and then remove magic number
    return frc2::InstantCommand{[&]() { intake->SetMotors(0.0); }, {intake}}.ToPtr();
}
#pragma endregion
