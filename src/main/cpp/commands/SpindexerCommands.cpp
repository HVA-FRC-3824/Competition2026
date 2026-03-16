#include "commands/SpindexerCommands.h"

#pragma region SpindexerSetState
frc2::CommandPtr SpindexerSetState(Spindexer* spindexer, SpindexerState state)
{
    // Create and return a InstantCommand that sets the spindexer state
    return frc2::InstantCommand{[=] {spindexer->SetState(state);}, {spindexer}}.ToPtr();
}
#pragma endregion

#pragma region SpindexerToggle
// This turns the indexer on
frc2::CommandPtr SpindexerToggle(Spindexer* spindexer)
{
    // Create and return a InstantCommand that sets the spindexer state
    return frc2::InstantCommand{[=] {
        if (spindexer->GetState() == SpindexerState::Stopped)
        {
            spindexer->SetState(SpindexerState::Spindexing);
        }
        else
        {
            spindexer->SetState(SpindexerState::Stopped);
        }
    }, {spindexer}}.ToPtr();
}
#pragma endregion