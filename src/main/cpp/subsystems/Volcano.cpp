#include "subsystems/Volcano.h"

#pragma region Volcano
/// @brief Constructor for the Volcano subsystem.
Volcano::Volcano()
{

}
#pragma endregion

#pragma region KickBall(bool running)
/// @brief Method to kick the ball to the flywheel.
/// @param running Whether the kicker is running or not.
void Volcano::SetKicker(bool running)
{
    // Set the kicker motor speed based on the running state
    m_kickMotor.SetReferenceState(running ? constants::volcano::targetKickerSpeed : 0.0_tps);
}
#pragma endregion

#pragma region IndexBall(bool running)
/// @brief Method to control the indexer motors.
/// @param running Whether the indexers are running or not.
void Volcano::SetIndexers(bool ruinning)
{
    // Loop through all indexer motors and set their speed based on the running state
    for (auto& indexer : m_indexerMotors)
    {
        indexer.SetReferenceState(ruinning ? constants::volcano::targetIndexerSpeed : 0.0_tps);
    }
}
#pragma endregion

#pragma region SetFlywheel
/// @brief Method to set the flywheel speed.
/// @param targetSpeed The desired speed for the flywheel in turns per second.
void Volcano::SetFlywheel(units::turns_per_second_t targetSpeed)
{
    // Set the flywheel motor to the target speed
    m_flywheelMotor.SetReferenceState(targetSpeed);

    // Store the target speed for later use or comparison
    m_targetSpeed = targetSpeed;
}
#pragma endregion

#pragma region IsBallDetected
/// @brief Method to check if a ball is detected.
/// @return True if the ball sensor detects a ball, false otherwise.
bool Volcano::IsBallDetected()
{
    // TODO: Check if the sensor returns true or false when a ball is detected
    return m_ballSensor.Get();
}
#pragma endregion

#pragma region IsFlywheelAtSpeed
/// @brief Method to check if the flywheel is at the target speed.
/// @return True if the flywheel speed is within the tolerance range of the target speed.
bool Volcano::IsFlywheelAtSpeed()
{
    // Get the flywheel velocity
    auto flyWheelSpeed = m_flywheelMotor.GetVelocity();

    // Check if the flywheel speed is within 5% of the target speed
    return std::abs(flyWheelSpeed.value() - m_targetSpeed.value()) <= m_targetSpeed.value() * constants::volcano::flywheelTolerancePercent;
}
#pragma endregion

#pragma region Periodic
/// @brief Periodic method called periodically by the scheduler.
/// Logs various subsystem states to the console or dashboard.
void Volcano::Periodic()
{
    Log("Flywheel Speed (TPS)",        m_flywheelMotor.GetVelocity().value());
    Log("Target Flywheel Speed (TPS)", m_targetSpeed.value());
    Log("Ball Detected: ",             IsBallDetected());
    Log("Flywheel At Speed: ",         IsFlywheelAtSpeed());
}
#pragma endregion