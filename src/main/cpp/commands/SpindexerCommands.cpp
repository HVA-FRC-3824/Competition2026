#include "commands/SpindexerCommands.h"

#pragma region SpindexerOn
// This turns the indexer on
frc2::CommandPtr SpindexerSetState(Spindexer* spindexer, SpindexerState state)
{
    // Create and return a InstantCommand that sets the spindexer state
    return frc2::InstantCommand{[=] {spindexer->SetState(state);}, {spindexer}}.ToPtr();
}
#pragma endregion