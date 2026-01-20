#include "subsystems/Indexer.h"

Indexer::Indexer() : m_motors{ctre::phoenix6::hardware::TalonFX{IndexerConstants::motorIDs[0]},
                              ctre::phoenix6::hardware::TalonFX{IndexerConstants::motorIDs[1]}}
{

}

// Sets the indexing motors
void Indexer::SetMotors(double input)
{
    ctre::phoenix6::controls::DutyCycleOut request{input};
    
    for (auto& motor : m_motors) 
    {
        motor.SetControl(request);
    }
}