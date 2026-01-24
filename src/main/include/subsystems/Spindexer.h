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

#pragma region StateStruct
enum SpindexerState
{
    Stopped,
    Spindexing
};
#pragma endregion

#pragma region SpindexerConstants
namespace SpindexerConstants
{
    // TODO: test and tune these values
    constexpr auto spinnerWheelTurns = 10_tps;
    constexpr auto kickerWheelTurns  = 10_tps;
}
#pragma endregion

class Spindexer : public frc2::SubsystemBase
{
    public:
        
        explicit Spindexer();

        // Sets the indexing motors
        void SetState(SpindexerState newState);

        SpindexerState GetState() const { return m_state; }

    private:

        ctre::phoenix6::hardware::TalonFX m_spinnerMotor{ConstantsCanIds::spinnerMotorId};
        ctre::phoenix6::hardware::TalonFX m_kickerMotor {ConstantsCanIds::kickerMotorId};
        
        SpindexerState m_state = SpindexerState::Stopped;
};
