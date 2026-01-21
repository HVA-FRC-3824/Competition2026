#include "subsystems/Indexer.h"

Indexer::Indexer()
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
