#pragma once

#pragma region Includes
#include <functional>

#include <frc2/command/SubsystemBase.h>

#include <ctre/phoenix6/TalonFX.hpp>

#include "lib/TalonFXConfiguration.h"

#include "Constants.h"
#include "ConstantsCanIds.h"
#pragma endregion

#pragma region SpindexerConstants
namespace SpindexerConstants
{
    // TODO: test and tune these values
    constexpr auto spinnerWheelVoltage = 10_V;
    constexpr auto kickerWheelVoltage  = 6_V;
}
#pragma endregion

#pragma region StateStruct
enum SpindexerState
{
    Indexing,
    Paused
};
#pragma endregion

class Spindexer : public frc2::SubsystemBase
{
    public:
        
        explicit Spindexer();

        // Sets the indexing motors
        void SetState(SpindexerState newState);

        void Periodic() override;

    private:

        void ConfigureMotor(ctre::phoenix6::hardware::TalonFX& motor);

        ctre::phoenix6::hardware::TalonFX m_spinnerMotor{ConstantsCanIds::spinnerMotorID};
        ctre::phoenix6::hardware::TalonFX m_kickerMotor {ConstantsCanIds::kickerMotorID};
        
        SpindexerState m_state = SpindexerState::Paused;
};
