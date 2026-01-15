#pragma once

#pragma region Includes
#include <functional>
#include <numeric>

#include <frc2/command/SubsystemBase.h>
#include <frc/geometry/Pose2d.h>

#include <frc/Servo.h>

#include <units/angle.h>
#include <units/length.h>

#include "lib/hardware/motors/TalonFX.h"

#include "Constants.h"
#pragma endregion

#pragma StateStructures
struct TowerState
{
    double flyWheelSpeed;
    double hoodActuatorPercentInput;
};

struct TestedTowerStates 
{
    units::foot_t distance;
    double        hoodInput;
    double        flywheelInput;
};
#pragma endregion

#pragma region Data
constexpr std::array<TestedTowerStates, 1U> experimentTables{
    TestedTowerStates{5_ft, 1, 1}
};
#pragma endregion

class Tower : public frc2::SubsystemBase
{
    public:
        
        Tower();

        void        SetState(TowerState newState);

        void        SetState(units::foot_t distance);

        TowerState  GetState();

    private:

        // https://andymark.com/products/linear-servo-actuators check the manual
        // Input in between 0 and 1
        void SetActuator(double position);

        // Starts to spin up the flywheel motor
        // This may need to be a -1,1 or some other kind of input
        void SetFlywheel(double input);

        // Interpolates between two known working points, the std::vector should become an array once we're done testing
        // This is temporary until polynomial method is made
        TowerState Interpolate(units::foot_t distance);

        hardware::motor::TalonFX m_flywheelMotor{constants::tower::flywheelMotorID, constants::tower::flywheelConfig};
        frc::Servo               m_hoodActuator{0}; // TODO: Remove magic number

        TowerState m_state{0, 0};
};