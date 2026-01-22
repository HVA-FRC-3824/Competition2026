#include "subsystems/Spindexer.h"

Spindexer::Spindexer()
{
    TalonFXConfiguration(&m_spinnerMotor,
                          20.0_A,
                          true,
                          0.1,
                          0.0,
                          0.0,
                          0.0,
                          0.0,
                          0.0,
                          0_tps,
                          units::turns_per_second_squared_t{0});

    TalonFXConfiguration(&m_kickerMotor,
                          20.0_A,
                          true,
                          0.1,
                          0.0,
                          0.0,
                          0.0,
                          0.0,
                          0.0,
                          0_tps,
                          units::turns_per_second_squared_t{0});
}
#pragma endregion

#pragma region SetState
/// @brief Sets the motors to their initial states 
/// @param input The input value for the motors
void Spindexer::SetState(SpindexerState newState)
{
    m_state = newState;

    // This is in SetState because we dont need to set the control every cycle, only every time its changed
    // Set the control request for each motor in the indexer
    switch (m_state) {
        case SpindexerState::Indexing:
            m_spinnerMotor.SetControl(ctre::phoenix6::controls::VoltageOut{SpindexerConstants::spinnerWheelVoltage});
            m_kickerMotor.SetControl(ctre::phoenix6::controls::VoltageOut{SpindexerConstants::kickerWheelVoltage});
            break;
        case SpindexerState::Paused:
            m_spinnerMotor.SetControl(ctre::phoenix6::controls::VoltageOut{0_V});
            m_kickerMotor.SetControl(ctre::phoenix6::controls::VoltageOut{0_V});
            break;
    }
}
#pragma endregion

#pragma region Periodic
void Spindexer::Periodic()
{
    
}
#pragma endregion