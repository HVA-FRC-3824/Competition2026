#include "subsystems/Spindexer.h"

#pragma region Spindexer
/// @brief Constructor for the Spindexer subsystem
Spindexer::Spindexer()
{
    // Configure the spindexer motor
    TalonFXConfiguration(&m_spinnerMotor,
                         20.0_A,          // Maximum Amperage
                         true,            // Inverted
                         false,           // Brake mode
                         false,           // Continuous wrap
                         0.4,             // P
                         0.0,             // I
                         0.0,             // D
                         0.0,             // S
                         0.0,             // V
                         0.0,             // A
                         0_tps,           // Velocity Limit
                         0_tr_per_s_sq);  // Acceleration Limit

    // Configure the kicker motor
    TalonFXConfiguration(&m_kickerMotor,
                         20.0_A,          // Maximum Amperage
                         false,           // Inverted
                         false,           // Brake mode
                         false,           // Continuous wrap
                         0.1,             // P
                         0.0,             // I
                         0.0,             // D
                         0.0,             // S
                         0.0,             // V
                         0.0,             // A
                         0_tps,           // Velocity Limit
                         0_tr_per_s_sq);  // Acceleration Limit

    TalonFXConfiguration(&m_kickerFollowerMotor,
                         20.0_A,          // Maximum Amperage
                         false,           // Inverted
                         false,            // Brake mode
                         false,           // Continuous wrap
                         0.1,             // P
                         0.0,             // I
                         0.0,             // D
                         0.0,             // S
                         0.0,             // V
                         0.0,             // A
                         0_tps,           // Velocity Limit
                         0_tr_per_s_sq);  // Acceleration Limit

    // TODO: VERIFY THESE CONFIGS
    // Set the second motor to follow the other motor
    m_kickerFollowerMotor.SetControl(
        ctre::phoenix6::controls::Follower(m_kickerMotor.GetDeviceID(), 
                                            ctre::phoenix6::signals::MotorAlignmentValue::Opposed)
    );
}
#pragma endregion

#pragma region SetState
/// @brief Sets the motors to their initial states 
/// @param input The input value for the motors
void Spindexer::SetState(SpindexerState newState)
{
    // Do nothing if the state is the same as the current state
    if (newState == m_state)
        return;

    // Remember the state of the spindexer
    m_state = newState;
    Log("Spindexer", std::string_view{"Setting spindexer state to " + std::to_string(static_cast<int>(newState))});

    // Default the speeds to zero
    auto spindexerSpeed = 0.0_tps;
    auto kickerSpeed    = 0.0_tps;

    // Set the speeds based on the state
    switch (m_state) 
    {
        case SpindexerState::Stopped:
        {
            // Stop both motors (speeds are already default to zero)
            break;
        }

        case SpindexerState::Spindexing:
        {
            // Set both motors to their respective speeds
            spindexerSpeed = SpindexerConstants::SpinnerWheelTurns;
            kickerSpeed    = SpindexerConstants::KickerWheelTurns;
            break;
        }
    }

    Log("Spindexer", std::string_view{"Setting spinner speed to " + std::to_string(spindexerSpeed.value()) + " turns per second and kicker speed to " + std::to_string(kickerSpeed.value()) + " turns per second"});

    // Set the motor speeds
    m_spinnerMotor.SetControl(ctre::phoenix6::controls::VelocityVoltage{spindexerSpeed});
    m_kickerMotor.SetControl(ctre::phoenix6::controls::VelocityVoltage{kickerSpeed});
}
#pragma endregion
