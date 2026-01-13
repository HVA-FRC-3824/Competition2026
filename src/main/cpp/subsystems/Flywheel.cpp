#include "subsystems/Flywheel.h"

        
Flywheel::Flywheel() :
    m_spinUpMotor{constants::flywheel::motorID, constants::flywheel::flywheelConfig}
{

}

// Starts to spin up the flywheel motor
// This may need to be a -1,1 or some other kind of input
void Flywheel::SetMotor(units::turns_per_second_t input)
{
    m_spinUpMotor.SetReferenceState(input);
}