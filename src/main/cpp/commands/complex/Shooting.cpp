#include "commands/complex/Shooting.h"

frc2::CommandPtr ShootToHub(Spindexer *spindexer, Tower *tower)
{
    // Spin up the tower (which takes 2-4 seconds) then start indexing
    return TowerAimHub(tower).Until([tower] { return tower->IsSpunUp(); })
        .AndThen(SpindexerSetState(spindexer, SpindexerState::Spindexing));
}

frc2::CommandPtr ShootToZone(Spindexer *spindexer, Tower *tower)
{
    // Spin up the tower (which takes 2-4 seconds) then start indexing
    return TowerAimPassZone(tower).Until([tower] { return tower->IsSpunUp(); })
        .AndThen(SpindexerSetState(spindexer, SpindexerState::Spindexing));
}

frc2::CommandPtr StopShooting(Spindexer *spindexer)
{
    // Do not stop tower, we dont want to have to constantly spin up and spin down the flywheel
    return SpindexerSetState(spindexer, SpindexerState::Stopped);
}