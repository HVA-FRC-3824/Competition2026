#include "subsystems/Indexer.h"

Indexer::Indexer() :
    m_motors{
        hardware::motor::TalonFX{constants::indexer::motorIDs[0], constants::indexer::indexerWheelConfig, hardware::motor::MotorType::KrakenX60},
        hardware::motor::TalonFX{constants::indexer::motorIDs[1], constants::indexer::indexerWheelConfig, hardware::motor::MotorType::KrakenX60},
    }
{

}

// Sets the indexing motors
void Indexer::SetMotors(double input)
{
    for (auto& motor : m_motors) {
        motor.SetReferenceState(input, hardware::motor::MotorInput::ARBITRARY);
    }
}