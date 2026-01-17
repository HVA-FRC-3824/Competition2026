#include "subsystems/Tower.h"

Tower::Tower()
{
    m_hoodActuator.SetBounds(2.0_us, 1.8_us, 1.5_us, 1.2_us, 1.0_us);
}

void Tower::SetState(units::foot_t distance)
{
    SetState(Interpolate(distance));
}
 
void Tower::SetState(TowerState newState)
{
    SetFlywheel(newState.flyWheelSpeed);

    SetActuator(newState.hoodActuatorPercentInput);
}

TowerState Tower::GetState()
{
    return m_state;
}

// Starts to spin up the flywheel motor
// This is arbitary since its 
void Tower::SetFlywheel(double input)
{
    m_flywheelMotor.SetReferenceState(input, hardware::motor::MotorInput::ARBITRARY);
}

// Input 0-1
void Tower::SetActuator(double position)
{
    // range: 0-1
	position = std::clamp(position, 0.0, 1.0);

    // range: 0-2.0
    position *= 2.0;

    // range: -1, 1
    position -= 1;

    // Although this says SetSpeed, this actually does position
	m_hoodActuator.SetSpeed(std::clamp(position, constants::tower::ActuatorLowerBound, constants::tower::ActuatorUpperBound));
}

TowerState Tower::Interpolate(units::foot_t distance) 
{
    // If the distance is very close, closer than we've tested, just do the closest tested shot 
    if (distance <= experimentTables[0].distance)
        return TowerState{experimentTables[0].flywheelInput, experimentTables[0].hoodInput};

    // You have to go down from the max by 2 because 
    // you need to look ahead to the next test in the array
    for (int i = 0; i < experimentTables.size() - 2U; i++) 
    {
        TestedTowerStates a = experimentTables[i];
        TestedTowerStates b = experimentTables[i + 1];

        // Checks if the real distance is in between the two tests
        // if not, go to the next one
        if (distance <= b.distance) {
            // This represents the midpoint for interpolation
            double t = (distance - a.distance) / (b.distance - a.distance);

            // Do the actual interpolations and return
            return TowerState{
                std::lerp(a.flywheelInput, b.flywheelInput, t),
                std::lerp(a.hoodInput,     b.hoodInput,     t)
            };
        }
    }

    // If a suitable pair isn't found, work with the most distant calculation
    // Ideally this doesn't happen so we should work out the correct numbers from right up against the hub to about half-field
    auto& furthestTest = experimentTables[experimentTables.size() - 1];
    return TowerState{furthestTest.flywheelInput, furthestTest.hoodInput};
}