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
    // Ensure the drive and angle motor encoders are reset to zero
    m_driveMotor.SetPosition(0.0_tr);
    m_angleMotor.SetPosition(0.0_tr);

    if (frc::RobotBase::IsSimulation())
    {
        auto& driveSim = m_driveMotor.GetSimState();
        auto& angleSim = m_angleMotor.GetSimState();
        
        driveSim.Orientation = ctre::phoenix6::sim::ChassisReference::CounterClockwise_Positive;
        driveSim.SetMotorType(ctre::phoenix6::sim::TalonFXSimState::MotorType::KrakenX60);
        
        angleSim.Orientation = ctre::phoenix6::sim::ChassisReference::CounterClockwise_Positive;
        angleSim.SetMotorType(ctre::phoenix6::sim::TalonFXSimState::MotorType::KrakenX44);
        return;
    }

    // Configure the motors
    TalonFXConfiguration(&m_driveMotor,    // Drive motor configuration
                          60_A,            // Maximum Amperage
                          true,            // Brake mode enabled
                          false,           // Continuous wrap
                          0.03,            // P gain
                          1.5,             // I gain
                          0.0,             // D gain
                          0.0,             // V gain
                          0.0,             // A gain
                          0.0,             // S gain
                          0_tps,           // Velocity limit
                          0_tr_per_s_sq);  // Acceleration limit

    TalonFXConfiguration(&m_angleMotor,    // Angle motor configuration
                          20_A,            // Maximum Amperage
                          true,            // Brake mode enabled
                          true,            // Continuous wrap
                          8.0,             // P gain
                          0.0,             // I gain
                          0.2,             // D gain
                          0.0,             // V gain
                          0.0,             // A gain
                          0.0,             // S gain
                          0_tps,           // Velocity limit
                          0_tr_per_s_sq,   // Acceleration limit
                          150.0 / 7.0);    // Sensor to mechanism ratio
}
#pragma endregion

#pragma region SetDesiredState
/// @brief Method to set the swerve module state to the desired state.
/// @param desiredState The desired swerve module velocity and angle.
/// @param description String to show the module state on the SmartDashboard.
void SwerveModule::SetDesiredState(frc::SwerveModuleState &desiredState, std::string description)
{
    frc::SmartDashboard::PutNumber(description + "Drive", (double) desiredState.speed.value());
    frc::SmartDashboard::PutNumber(description + "Angle", (double) desiredState.angle.Degrees().value());

    // Optimize the reference state to avoid spinning further than 90 degrees
    // desiredState.Optimize(frc::Rotation2d(GetPosition().angle.Degrees()));
    desiredState.Optimize(GetPosition().angle);

    // Convert angle to motor posiition (1.0 turn = 360 degrees)
    m_angleMotor.SetControl(ctre::phoenix6::controls::PositionDutyCycle((units::turn_t) (desiredState.angle.Degrees().value() / 360.0)));

    // Convert wheel linear velocity to motor rotations per second						   
    m_driveMotor.SetControl(ctre::phoenix6::controls::VelocityVoltage((units::angular_velocity::turns_per_second_t)
                           (desiredState.speed.value() / SwerveConstants::DriveMotorConversion.value())));

    Log(description + "Absolute Encoder ", (double) m_angleAbsoluteEncoder.GetAbsolutePosition().GetValue() * 360.0);
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
    auto driveVelocity = units::meters_per_second_t {(double) m_driveMotor.GetVelocity().GetValue()  * SwerveConstants::DriveMotorConversion.value()};
    auto anglePosition = units::degree_t{           ((double) m_angleMotor.GetPosition().GetValue()) * 360.0_deg};

    // Return the swerve module state
    return {driveVelocity, anglePosition};
}
#pragma endregion

#pragma region GetPosition
/// @brief Method to retrieve the swerve module position.
/// @return The swerve module position.
frc::SwerveModulePosition SwerveModule::GetPosition()
{   
    // Determine the module drive and angle positions
    auto drivePosition = units::meter_t {((double) m_driveMotor.GetPosition().GetValue()) * SwerveConstants::DriveMotorConversion.value()};
    auto anglePosition = units::degree_t{((double) m_angleMotor.GetPosition().GetValue()) * 360.0_deg};

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
void SwerveModule::SetWheelAngleToForward(units::angle::degree_t forwardAngle)
{
    // Ensure the drive and angle motor encoders are reset to zero
    m_driveMotor.SetPosition(0_tr);
    m_angleMotor.SetPosition(0_tr);

    // Determine the move angle for the forward direction
    units::angle::degree_t moveDegrees = forwardAngle - GetAbsoluteEncoderAngle();

    // Determine the shortest move angle (wrap to -180 to +180)
    while (moveDegrees > 180.0_deg)  moveDegrees -= 360.0_deg;
    while (moveDegrees < -180.0_deg) moveDegrees += 360.0_deg;

    // Determine the angle motor position value (-0.5 to 1.0)
    units::angle::turn_t anglePosition = (units::angle::turn_t) (moveDegrees.value() / 360.0);

    // Set the angle motor to the calculated position
    m_angleMotor.SetPosition(anglePosition);

    // Set the motor angle to the forward direction (position 0)
    m_angleMotor.SetControl(ctre::phoenix6::controls::PositionDutyCycle{0_tr});
}
#pragma endregion

#pragma region GetAbsoluteEncoderAngle
/// @brief Method to read the absolute encode in degrees.
/// @return The absolute angle value in degrees.
units::angle::degree_t SwerveModule::GetAbsoluteEncoderAngle()
{
    // The GetAbsolutePosition() method returns a value from -0.5 to 0.5
    double encoderValue = (double) m_angleAbsoluteEncoder.GetAbsolutePosition().GetValue();

    // To convert to degrees, multiply by 360
    return encoderValue * 360_deg;
}
#pragma endregion

#pragma SimPeriodic
void SwerveModule::SimPeriodic()
{
    auto& driveSim = m_driveMotor.GetSimState();

   // set the supply voltage of the TalonFX
   driveSim.SetSupplyVoltage(frc::RobotController::GetBatteryVoltage());

   // get the motor voltage of the TalonFX
   auto motorVoltage = driveSim.GetMotorVoltage();

   // use the motor voltage to calculate new position and velocity
   // using WPILib's DCMotorSim class for physics simulation
   m_simDriveModel.SetInputVoltage(motorVoltage);
   m_simDriveModel.Update(20_ms); // assume 20 ms loop time

   // apply the new rotor position and velocity to the TalonFX;
   // note that this is rotor position/velocity (before gear ratio), but
   // DCMotorSim returns mechanism position/velocity (after gear ratio)
   driveSim.SetRawRotorPosition((6.75) * m_simDriveModel.GetAngularPosition());
   driveSim.SetRotorVelocity(   (6.75) * m_simDriveModel.GetAngularVelocity());

   auto& angleSim = m_angleMotor.GetSimState();

   // set the supply voltage of the TalonFX
   angleSim.SetSupplyVoltage(frc::RobotController::GetBatteryVoltage());

   // get the motor voltage of the TalonFX
   motorVoltage = angleSim.GetMotorVoltage();

   // use the motor voltage to calculate new position and velocity
   // using WPILib's DCMotorSim class for physics simulation
   m_simTurnModel.SetInputVoltage(motorVoltage);
   m_simTurnModel.Update(20_ms); // assume 20 ms loop time

   // apply the new rotor position and velocity to the TalonFX;
   // note that this is rotor position/velocity (before gear ratio), but
   // DCMotorSim returns mechanism position/velocity (after gear ratio)
   angleSim.SetRawRotorPosition((150.0 / 7.0) * m_simTurnModel.GetAngularPosition());
   angleSim.SetRotorVelocity(   (150.0 / 7.0) * m_simTurnModel.GetAngularVelocity());
}
#pragma endregion