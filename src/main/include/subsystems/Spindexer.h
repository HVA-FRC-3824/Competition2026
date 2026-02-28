#pragma once

#pragma region Includes
#include <functional>

#include <frc2/command/SubsystemBase.h>

#include <ctre/phoenix6/TalonFX.hpp>

#include "lib/TalonFXConfiguration.h"
#include "lib/Logging.h"

#include "Constants.h"
#include "ConstantsRoboRio.h"
#pragma endregion

#pragma region SpindexerState
enum SpindexerState
{
    Stopped,
    Spindexing
};
#pragma endregion

#pragma region SpindexerConstants
namespace SpindexerConstants
{
    constexpr auto SpinnerWheelTurns = 130_tps; // Maximum Velocity: 130 tps
    constexpr auto KickerWheelTurns  = 120_tps; // Maximum Velocity: 120 tps
}
#pragma endregion

class Spindexer : public frc2::SubsystemBase
{
    public:
        
        explicit       Spindexer();

        void           SetState(SpindexerState newState);

        SpindexerState GetState() const { return m_state; }

    private:
        
        SpindexerState                    m_state{SpindexerState::Stopped};

        ctre::phoenix6::hardware::TalonFX m_spinnerMotor       {ConstantsCanIds::SpinnerMotorId};
        ctre::phoenix6::hardware::TalonFX m_kickerFollowerMotor{ConstantsCanIds::KickerFollowerMotorId};
        ctre::phoenix6::hardware::TalonFX m_kickerMotor        {ConstantsCanIds::KickerMotorId};
};
