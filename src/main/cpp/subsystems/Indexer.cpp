#include "subsystems/Indexer.h"

Indexer::Indexer() :
    m_motors{
        hardware::motor::TalonFX{constants::indexer::motorIDs[0], constants::indexer::indexerWheelConfig},
        hardware::motor::TalonFX{constants::indexer::motorIDs[1], constants::indexer::indexerWheelConfig},
        hardware::motor::TalonFX{constants::indexer::motorIDs[2], constants::indexer::indexerWheelConfig},
        hardware::motor::TalonFX{constants::indexer::motorIDs[3], constants::indexer::indexerWheelConfig}
    }
{

}

// Sets the indexing motors
void Indexer::SetMotors(double input)
{
    for (auto& motor : m_motors) {
        motor.SetReferenceState(input);
    }
}