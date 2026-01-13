#include "subsystems/Turret.h"


Turret::Turret() :
    m_angleMotor{constants::turret::motorID, constants::turret::turretConfig}
{
    
}

// Sets the desired angle of the turret relative to the robot
void Turret::SetAngle(units::degree_t angle)
{
    m_angleMotor.SetReferenceState(units::turn_t{angle.value() / 360});
}