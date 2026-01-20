#include "subsystems/Indexer.h"

#pragma region Constructor
/// @brief Constructor for the Indexer subsystem
Indexer::Indexer() : m_motors{ctre::phoenix6::hardware::TalonFX{IndexerConstants::motorIDs[0]},
                              ctre::phoenix6::hardware::TalonFX{IndexerConstants::motorIDs[1]}}
{
    // Configure each motor in the indexer
    for (auto& motor : m_motors) 
    {
        // Configure the motor settings
        ConfigureMotor(motor);
    }
}
#pragma endregion

#pragma region ConfigureMotor
/// @brief Configures the motor settings for the indexer motors
void Indexer::ConfigureMotor(ctre::phoenix6::hardware::TalonFX& motor)
{

}
#pragma endregion

#pragma region SetMotors
/// @brief Sets the motors to their initial states 
/// @param input The input value for the motors
void Indexer::SetMotors(double input)
{
    // Create a DutyCycleOut control request with the input value
    ctre::phoenix6::controls::DutyCycleOut request{input};
    
    // Set the control request for each motor in the indexer
    for (auto& motor : m_motors) 
    {
        // Set the motor control to the specified duty cycle
        motor.SetControl(request);
    }
}
#pragma endregion
