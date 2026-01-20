#include "lib/SwerveModule.h"

#pragma region SwerveModule
/// @brief Class constructor for the SwerveModule class.
/// @param driveMotorCanId The CAN ID for the swerve module drive motor.
/// @param angleMotorCanId The CAN ID for the swerve module angle motor.
/// @param angleEncoderCanId The CAN ID for the swerve module angle encoder.
/// @param driveConfig The motor configuration for the drive motor.
/// @param turnConfig The motor configuration for the angle motor.
/// @param driveMotorConversion The conversion factor for the drive motor (wheel circumference / (gear ratio * motor revolutions)).
/// @param angleMotorConversion The conversion factor for the angle motor (motor revolutions / (2 * pi)).
SwerveModule::SwerveModule(int driveMotorCanId, int angleMotorCanId, int angleEncoderCanId, 
                           hardware::motor::MotorConfiguration driveConfig, hardware::motor::MotorConfiguration turnConfig) :
        m_driveMotor          {driveMotorCanId, driveConfig, hardware::motor::MotorType::KrakenX60, frc::DCMotor::KrakenX60()},
        m_angleMotor          {angleMotorCanId, turnConfig,  hardware::motor::MotorType::KrakenX44, frc::DCMotor::KrakenX44()},
        m_angleAbsoluteEncoder{angleEncoderCanId}

{
    // Ensure the drive motor encoder is reset to zero
    m_driveMotor.OffsetEncoder(0);
}
#pragma endregion

#pragma region SetDesiredState
/// @brief Method to set the swerve module state to the desired state.
/// @param desiredState The desired swerve module velocity and angle.
/// @param description String to show the module state on the SmartDashboard.
void SwerveModule::SetDesiredState(frc::SwerveModuleState& desiredState)
{
    // Optimize the reference state to avoid spinning further than 90 degrees.
    desiredState.Optimize(GetPosition().angle);

    // Some WPI magic math cosine to prevent jittering
    // desiredState.speed = units::meters_per_second_t{desiredState.speed.value() * std::cos(desiredState.angle.Radians().value() - GetPosition().angle.Radians().value())};

    // Set the motor speed and angle
    if (frc::RobotBase::IsSimulation())
    {
        m_driveMotor.SimPeriodic();
        m_angleMotor.SimPeriodic();
    }

    // Set the motor reference states
    m_driveMotor.SetReferenceState(desiredState.speed.value() / constants::swerve::wheelCircumference.value(), hardware::motor::MotorInput::VELOCITY);
    m_angleMotor.SetReferenceState(desiredState.angle.Radians().value() / (2 * std::numbers::pi), hardware::motor::MotorInput::POSITION);

    Log("Absolute Encoder ", (double) m_angleAbsoluteEncoder.GetAbsolutePosition().GetValue() * 360);
}
#pragma endregion

#pragma region GetState
/// @brief  Method to retrieve the swerve module state.
/// @return The swerve module speed and angle state.
frc::SwerveModuleState SwerveModule::GetState()
{   
    if (frc::RobotBase::IsSimulation())
        return {
            1_mps * m_driveMotor.GetVelocity().value(),
            1_rad * m_angleMotor.GetPosition().value()
        };

    // Determine the module wheel velocity 
    units::meters_per_second_t driveVelocity{m_driveMotor.GetVelocity().value() * constants::swerve::wheelCircumference.value()};
    units::radian_t            anglePosition{m_angleMotor.GetPosition().value() * 2 * std::numbers::pi};
        
    // Return the swerve module state
    return {driveVelocity, anglePosition};
}
#pragma endregion

#pragma region GetPosition
/// @brief Method to retrieve the swerve module position.
frc::SwerveModulePosition SwerveModule::GetPosition()
{
    if (frc::RobotBase::IsSimulation())
        return {
            1_m   * m_driveMotor.GetVelocity().value(),
            1_rad * m_angleMotor.GetPosition().value()
        };
    
    // Determine the module wheel position
    units::meter_t  drivePosition{m_driveMotor.GetPosition().value() * constants::swerve::wheelCircumference.value()};
    units::radian_t anglePosition{m_angleMotor.GetPosition().value() * 2 * std::numbers::pi};

    // Return the swerve module position
    return {drivePosition, anglePosition};
}
#pragma endregion

#pragma region ResetDriveEncoder
// Reset the drive encoder position.
void SwerveModule::ResetDriveEncoder()
{
    // Ensure the drive motor encoder is reset to zero
    m_driveMotor.OffsetEncoder(0);
}
#pragma endregion

#pragma region SetWheelAngleToForward
/// @brief Method to set the swerve wheel encoder to the forward angle.
/// @param forwardAngle The absolute angle for the forward direction.
void SwerveModule::SetWheelAngleToForward(units::angle::radian_t forwardAngle)
{
    // Ensure the drive motor encoder is reset to zero
    m_driveMotor.OffsetEncoder(0);

    // Set the motor angle encoder position to the forward direction
    m_angleMotor.OffsetEncoder(GetAbsoluteEncoderAngle().value() - forwardAngle.value());

    // Set the motor angle to the forward direction
    m_angleMotor.SetReferenceState(0, hardware::motor::MotorInput::POSITION);
}
#pragma endregion

#pragma region GetAbsoluteEncoderAngle
/// @brief Method to read the absolute encode in radians.
/// @return The absolute angle value in radians.
units::angle::radian_t SwerveModule::GetAbsoluteEncoderAngle()
{
    // The GetAbsolutePosition() method returns a value from -1 to 1
    double encoderValue = (double) m_angleAbsoluteEncoder.GetAbsolutePosition().GetValue();

    // To convert to radians
    return encoderValue * (2.0_rad * std::numbers::pi);
}
#pragma endregion
