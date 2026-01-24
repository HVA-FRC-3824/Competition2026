#include "subsystems/Spindexer.h"

Spindexer::Spindexer()
{
    TalonFXConfiguration(&m_spinnerMotor,
                          20.0_A, // Maximum Amperage
                          true, // Break mode
                          0.1, // P
                          0.0, // I
                          0.0, // D
                          0.0, // S
                          0.0, // V
                          0.0, // A
                          0_tps, // Velocity Limit
                          units::turns_per_second_squared_t{0}); // Acceleration Limit

    TalonFXConfiguration(&m_kickerMotor,
                          20.0_A, // Maximum Amperage
                          true, // Break mode
                          0.1, // P
                          0.0, // I
                          0.0, // D
                          0.0, // S
                          0.0, // V
                          0.0, // A
                          0_tps, // Velocity Limit
                          units::turns_per_second_squared_t{0}); // Acceleration Limit
}
#pragma endregion

#pragma region SetState
/// @brief Sets the motors to their initial states 
/// @param input The input value for the motors
void Spindexer::SetState(SpindexerState newState)
{
    if (newState == m_state)
    {
        // Do nothing if the state is the same as the current state
        return;
    }

    // Remember the state of the spindexer
    m_state = newState;
    Log("Spindexer", std::string_view{"Setting spindexer state to " + std::to_string(static_cast<int>(newState))});

    auto spindexerSpeed = 0.0_tps;
    auto kickerSpeed    = 0.0_tps;

    switch (m_state) 
    {
        case SpindexerState::Stopped:
        {
            // Stop both motors
            break;
        }

        case SpindexerState::Spindexing:
        {
            // Set both motors to their respective speeds
            spindexerSpeed = SpindexerConstants::spinnerWheelTurns;
            kickerSpeed    = SpindexerConstants::kickerWheelTurns;
            break;
        }
    }

    Log("Spindexer", std::string_view{"Setting spinner speed to " + std::to_string(spindexerSpeed.value()) + " turns per second and kicker speed to " + std::to_string(kickerSpeed.value()) + " turns per second"});
    m_spinnerMotor.SetControl(ctre::phoenix6::controls::VelocityVoltage{spindexerSpeed});
    m_kickerMotor.SetControl(ctre::phoenix6::controls::VelocityVoltage{kickerSpeed});
}
#pragma endregion