#include "lib/SwerveModule.h"

#pragma region SwerveModule
/// @brief Class constructor for the SwerveModule class.
/// @param driveMotorCanId The CAN ID for the swerve module drive motor.
/// @param angleMotorCanId The CAN ID for the swerve module angle motor.
/// @param angleEncoderCanId The CAN ID for the swerve module angle encoder.
SwerveModule::SwerveModule(int driveMotorCanId, int angleMotorCanId, int angleEncoderCanId) :
        m_driveMotor          {driveMotorCanId},
        m_angleMotor          {angleMotorCanId},
        m_angleAbsoluteEncoder{angleEncoderCanId}
{
    // Configure the drive and angle motors
    ConfigureDriveMotor();
    ConfigureAngleMotor();

    // Ensure the drive motor encoder is reset to zero
    m_driveMotor.SetPosition(0.0_tr);
}
#pragma endregion

#pragma region ConfigureDriveMotor
/// @brief Method to configure the drive motor.
void SwerveModule::ConfigureDriveMotor()
{
    // Create the drive motor configuration
    ctre::phoenix6::configs::TalonFXConfiguration talonFXConfiguration{};

    // Add the "Motor Output" section settings
    ctre::phoenix6::configs::MotorOutputConfigs &motorOutputConfigs = talonFXConfiguration.MotorOutput;
    motorOutputConfigs.NeutralMode = ctre::phoenix6::signals::NeutralModeValue::Brake;

    // Add the "Current Limits" section settings
    ctre::phoenix6::configs::CurrentLimitsConfigs &currentLimitsConfigs = talonFXConfiguration.CurrentLimits;
    currentLimitsConfigs.StatorCurrentLimit       = SwerveConstants::DriveMaximumAmperage;
    currentLimitsConfigs.StatorCurrentLimitEnable = true;

    // Add the "Slot0" section settings
    ctre::phoenix6::configs::Slot0Configs &slot0Configs = talonFXConfiguration.Slot0;
    slot0Configs.kP = SwerveConstants::DriveP;
    slot0Configs.kI = SwerveConstants::DriveI;
    slot0Configs.kD = SwerveConstants::DriveD;
    slot0Configs.kV = SwerveConstants::DriveV;
    slot0Configs.kA = SwerveConstants::DriveA;

    // Add the "Feedback" section settings
    // ctre::phoenix6::configs::FeedbackConfigs &feedbackConfigs = talonFXConfiguration.Feedback;
    // feedbackConfigs.SensorToMechanismRatio = SwerveConstants::WheelCircumference.value() / SwerveConstants::DriveMotorReduction;

    // Apply the configuration to the drive motor
    ctre::phoenix::StatusCode status = ctre::phoenix::StatusCode::StatusCodeNotInitialized;
    for (int attempt = 0; attempt < SwerveConstants::MotorConfigurationAttempts; attempt++)
    {
        // Apply the configuration to the drive motor
        status = m_driveMotor.GetConfigurator().Apply(talonFXConfiguration);

        // Check if the configuration was successful
        if (status.IsOK())
           break;
    }

    // Determine if the last configuration load was successful
    if (!status.IsOK())
        std::cout << "***** ERROR: Could not configure swerve motor. Error: " << status.GetName() << std::endl;
}
#pragma endregion

#pragma region ConfigureAngleMotor
/// @brief Method to configure the angle motor and encoder.
void SwerveModule::ConfigureAngleMotor()
{
   // Create the Angle motor configuration
    ctre::phoenix6::configs::TalonFXConfiguration talonFXConfiguration{};

    // Add the "Motor Output" section settings
    ctre::phoenix6::configs::MotorOutputConfigs &motorOutputConfigs = talonFXConfiguration.MotorOutput;
    motorOutputConfigs.NeutralMode = ctre::phoenix6::signals::NeutralModeValue::Brake;

    // Add the "Current Limits" section settings
    ctre::phoenix6::configs::CurrentLimitsConfigs &currentLimitsConfigs = talonFXConfiguration.CurrentLimits;
    currentLimitsConfigs.StatorCurrentLimit       = SwerveConstants::AngleMaximumAmperage;
    currentLimitsConfigs.StatorCurrentLimitEnable = true;

    // Add the "Slot0" section settings
    ctre::phoenix6::configs::Slot0Configs &slot0Configs = talonFXConfiguration.Slot0;
    slot0Configs.kP = SwerveConstants::AngleP;
    slot0Configs.kI = SwerveConstants::AngleI;
    slot0Configs.kD = SwerveConstants::AngleD;

    // Add the "Feedback" section settings
    // ctre::phoenix6::configs::FeedbackConfigs &feedbackConfigs = talonFXConfiguration.Feedback;
    // feedbackConfigs.SensorToMechanismRatio = SwerveConstants::WheelCircumference.value() / SwerveConstants::DriveMotorReduction;

    // Apply the configuration to the angle motor
    ctre::phoenix::StatusCode status = ctre::phoenix::StatusCode::StatusCodeNotInitialized;
    for (int attempt = 0; attempt < SwerveConstants::MotorConfigurationAttempts; attempt++)
    {
        // Apply the configuration to the angle motor
        status = m_angleMotor.GetConfigurator().Apply(talonFXConfiguration);

        // Check if the configuration was successful
        if (status.IsOK())
           break;
    }

    // Determine if the last configuration load was successful
    if (!status.IsOK())
        std::cout << "***** ERROR: Could not configure swerve motor. Error: " << status.GetName() << std::endl;
}
#pragma endregion

#pragma region SetDesiredState
/// @brief Method to set the swerve module state to the desired state.
/// @param desiredState The desired swerve module velocity and angle.
/// @param description String to show the module state on the SmartDashboard.
void SwerveModule::SetDesiredState(frc::SwerveModuleState& desiredState, std::string description)
{
    frc::SmartDashboard::PutNumber(description + "Drive", (double) desiredState.speed);
    frc::SmartDashboard::PutNumber(description + "Angle", (double) desiredState.angle.Degrees().value());

    // Optimize the reference state to avoid spinning further than 90 degrees.
    desiredState.Optimize(GetPosition().angle);

    // Some WPI magic math cosine to prevent jittering
    // desiredState.speed = units::meters_per_second_t{desiredState.speed.value() * std::cos(desiredState.angle.Radians().value() - GetPosition().angle.Radians().value())};

    // // Set the motor speed and angle
    // if (frc::RobotBase::IsSimulation())
    // {
    //     m_driveMotor.SimPeriodic();
    //     m_angleMotor.SimPeriodic();
    // }

    // Set the motor reference states
    // Convert wheel linear velocity to motor rotations per second
    double wheelRotationsPerSecond = desiredState.speed.value() / SwerveConstants::wheelCircumference.value();
    units::angular_velocity::turns_per_second_t motorVelocity{wheelRotationsPerSecond * SwerveConstants::DriveMotorReduction};
    m_driveMotor.SetControl(ctre::phoenix6::controls::VelocityDutyCycle{motorVelocity});
    
    // Convert angle from radians to rotations (0 to 1 for 0 to 2π)
    units::angle::turn_t angleRotations{desiredState.angle.Radians().value() / (2 * std::numbers::pi)};
    m_angleMotor.SetControl(ctre::phoenix6::controls::PositionDutyCycle{angleRotations});

    Log("Absolute Encoder ", (double) m_angleAbsoluteEncoder.GetAbsolutePosition().GetValue() * 360);
}
#pragma endregion

#pragma region GetState
/// @brief  Method to retrieve the swerve module state.
/// @return The swerve module speed and angle state.
frc::SwerveModuleState SwerveModule::GetState()
{
    // if (frc::RobotBase::IsSimulation())
    //     return {
    //         1_mps * m_driveMotor.GetVelocity().value(),
    //         1_rad * m_angleMotor.GetPosition().value()
    //     };

    // Determine the module wheel velocity
    auto driveVelocity = units::meters_per_second_t {
        (double) m_driveMotor.GetVelocity().GetValue() * SwerveConstants::DriveMotorConversion.value()};

    auto anglePosition = units::radian_t{m_angleMotor.GetPosition().GetValue()};

    // Return the swerve module state
    return {driveVelocity, anglePosition};
}
#pragma endregion

#pragma region GetPosition
/// @brief Method to retrieve the swerve module position.
frc::SwerveModulePosition SwerveModule::GetPosition()
{
    // if (frc::RobotBase::IsSimulation())
    //     return {
    //         1_m   * m_driveMotor.GetVelocity().value(),
    //         1_rad * m_angleMotor.GetPosition().value()
    //     };
    
    // Determine the module wheel position
    auto drivePosition = units::meter_t{
        ((double) m_driveMotor.GetPosition().GetValue()) * SwerveConstants::DriveMotorConversion.value()};

    auto anglePosition = units::radian_t{m_angleMotor.GetPosition().GetValue()};

    // Return the swerve module position
    return {drivePosition, anglePosition};
}
#pragma endregion

#pragma region ResetDriveEncoder
// Reset the drive encoder position.
void SwerveModule::ResetDriveEncoder()
{
    // Ensure the drive motor encoder is reset to zero
    m_driveMotor.SetPosition(0_tr);
}
#pragma endregion

#pragma region SetWheelAngleToForward
/// @brief Method to set the swerve wheel encoder to the forward angle.
/// @param forwardAngle The absolute angle for the forward direction.
void SwerveModule::SetWheelAngleToForward(units::angle::radian_t forwardAngle)
{
    // Ensure the drive motor encoder is reset to zero
    m_driveMotor.SetPosition(0_tr);

    // // Set the motor angle encoder position to the forward direction
    // m_angleMotor.GetEncoder().SetPosition(GetAbsoluteEncoderAngle().value() - forwardAngle.value());

    // // Set the motor angle to the forward direction
    // m_angleMotor.SetControl(0, hardware::motor::MotorInput::POSITION);
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
