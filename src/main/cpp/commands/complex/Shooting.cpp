#include "commands/complex/Shooting.h"

#pragma region ShootToHub
/// @brief Command to shoot balls to the hub
/// @param spindexer Pointer to the spindexer subsystem
/// @param tower Pointer to the tower subsystem
/// @return CommandPtr that shoots balls to the hub
frc2::CommandPtr ShootToHub(Spindexer *spindexer, Tower *tower)
{
    // Spin up the tower (which takes 2-4 seconds) then start indexing
    return TowerAimHub(tower).Until([tower] { return tower->IsSpunUp(); })
        .AndThen(SpindexerSetState(spindexer, SpindexerState::Spindexing));
}
#pragma endregion

#pragma region ShootToZone
/// @brief Command to shoot balls to the pass zone
/// @param spindexer Pointer to the spindexer subsystem
/// @param tower Pointer to the tower subsystem
/// @return CommandPtr that shoots balls to the pass zone
frc2::CommandPtr ShootToZone(Spindexer *spindexer, Tower *tower)
{
    // Spin up the tower (which takes 2-4 seconds) then start indexing
    return TowerAimPassZone(tower).Until([tower] { return tower->IsSpunUp(); })
        .AndThen(SpindexerSetState(spindexer, SpindexerState::Spindexing));
}
#pragma endregion
