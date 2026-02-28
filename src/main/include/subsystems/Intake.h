#pragma once

#pragma region Includes
#include <functional>

#include <ctre/phoenix6/TalonFX.hpp>
#include <frc2/command/SubsystemBase.h>

#include "lib/TalonFXConfiguration.h"
#include "lib/Logging.h"

#include "Constants.h"
#include "ConstantsRoboRio.h"
#pragma endregion

#pragma region IntakeEnums
enum IntakeState
{
    Stowed,

    DeployedRollerOn,
    DeployedRollerOff
};
#pragma endregion

#pragma region IntakeConstants
namespace IntakeConstants
{
    // TODO: test these
    constexpr auto IntakeStowedAngle   = 0.0_tr;
    constexpr auto IntakeDeployedAngle = 0.25_tr;
    constexpr auto IntakeStartingAngle = -21_deg; // This is about the initial offset of the intake at the start

    constexpr auto IntakeDriveSpeed    = 10_tps;

    constexpr auto IntakePositionGearReduction = 25.0;
    constexpr auto IntakeRollerGearReduction   = 10.0;

}
#pragma endregion

class Intake : public frc2::SubsystemBase
{
    public:
    
        explicit    Intake();
        
        void        SetState(IntakeState newState);

        IntakeState GetState() const { return m_intakeState; }

    private:

        // Motor that will extend and retract the intake (Magic motion position controlled)
        ctre::phoenix6::hardware::TalonFX m_intakePositionMotor{ConstantsCanIds::IntakePositionMotorId, ConstantsCanIds::CanBus};

        // Motor that will drive the intake to take in fuel (Velocity PID controlled)
        ctre::phoenix6::hardware::TalonFX m_fuelIntakeMotor{ConstantsCanIds::FuelIntakeMotorId, ConstantsCanIds::CanBus};
        
        // Current intake drive state, starts as Stowed
        IntakeState m_intakeState = IntakeState::Stowed;
};
