#include "subsystems/Intake.h"

#pragma region Constructor
Intake::Intake() 
{
    SetMotors();
}
#pragma endregion

#pragma region SetMotors
/// @brief Configure motors and stuff
void Intake::SetMotors()
{
    // Initially zero all motors
    m_driveMotor.SetPosition(0.0_tr);
    m_turnMotor.Set(0.00);

    // Set values to 0
    m_intakePosition = Stowed;
    m_intakeState    = Inactive;
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
            m_turnMotor.SetControl(ctre::phoenix6::controls::PositionDutyCycle{IntakeConstants::IntakeMaxAngle});
            break;
        }

        case IntakePosition::Deployed:
        {
            m_turnMotor.SetControl(ctre::phoenix6::controls::PositionDutyCycle{0.00_tr}); // 0 rotations
            break;
        }
    }
}
#pragma endregion

#pragma region Drive
/// @brief Changes drive intake to the selected state, either on or off
/// @param state On or Off
void Intake::DriveIntake(IntakeState state)
{
    // Remember the state
    m_intakeState = state;

    switch (m_intakeState)
    {
        case IntakeState::Inactive:
            m_driveMotor.Set(0.0);
            break;
        case IntakeState::Active:
            m_driveMotor.Set(IntakeConstants::IntakeDriveVoltage.value());
            break;
    }
}
#pragma endregion

