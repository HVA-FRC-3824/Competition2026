#include "subsystems/Intake.h"

#pragma region Constructor
/// @brief Constructor for the Intake subsystem
Intake::Intake() 
{
    // Configure the motors
    ConfigureFuelIntakeMotor();
    ConfigureIntakePositonMotor();

    // Initially zero all motors
    m_fuelIntakeMotor.SetPosition(0.0_tr);
    m_intakePositonMotor.Set(0.00);

    // Set values to 0
    m_intakePosition = Stowed;
    m_intakeState    = Inactive;
}
#pragma endregion

#pragma region ConfigureIntakePositonMotor
/// @brief Configures the intake position motor
void Intake::ConfigureIntakePositonMotor()
{

}
#pragma endregion

#pragma region ConfigureFuelIntakeMotor
/// @brief Configures the fuel intake motor
void Intake::ConfigureFuelIntakeMotor()
{

}
#pragma endregion

#pragma region Deploy Intake
/// @brief Toggles the intake between 0 and 0.25 rotations based on position argument
/// @param position Deployed or Stowed
void Intake::SetIntakePosition(IntakePosition position)
{
    // Remember intake position
    m_intakePosition = position;

    switch (m_intakePosition)
    {
        case IntakePosition::Stowed:
        {
            // Use PositionDutyCycle or PositionVoltage for position control
            m_intakePositonMotor.SetControl(ctre::phoenix6::controls::PositionDutyCycle{IntakeConstants::IntakeMaxAngle});
            break;
        }

        case IntakePosition::Deployed:
        {
            m_intakePositonMotor.SetControl(ctre::phoenix6::controls::PositionDutyCycle{0.00_tr}); // 0 rotations
            break;
        }
    }
}
#pragma endregion

#pragma region Drive
/// @brief Changes drive intake to the selected state, either Inactive or Active
/// @param state Inactive or Active
void Intake::DriveIntake(IntakeState state)
{
    // Remember the state
    m_intakeState = state;

    switch (m_intakeState)
    {
        case IntakeState::Inactive:
        {
            m_fuelIntakeMotor.Set(0.0);
            break;
        }

        case IntakeState::Active:
        {
            m_fuelIntakeMotor.Set(IntakeConstants::IntakeDriveVoltage.value());
            break;
        }
    }
}
#pragma endregion
