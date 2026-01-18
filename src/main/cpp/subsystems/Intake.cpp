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
    m_driveMotor.SetReferenceState(0.00, hardware::motor::MotorInput::POSITION);
    m_turnMotor.SetReferenceState(0.00, hardware::motor::MotorInput::POSITION);

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
            m_turnMotor.SetReferenceState(IntakeConstants::IntakeMaxAngle, hardware::motor::MotorInput::POSITION); // set motor to turn however many rotations it needs to
            break;
        case IntakePosition::Deployed:
            m_turnMotor.SetReferenceState(0.00, hardware::motor::MotorInput::POSITION); // Set motor to go to 0 rotations
            break;
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
            m_driveMotor.SetReferenceState(0, hardware::motor::MotorInput::VOLTAGE);
            break;
        case IntakeState::Active:
            m_driveMotor.SetReferenceState(IntakeConstants::IntakeDriveVoltage.value(), hardware::motor::MotorInput::VOLTAGE);
            break;
    }
}
#pragma endregion

