#include "RobotContainer.h"

// Reference to the RobotContainer singleton class
RobotContainer *RobotContainer::m_robotContainer = nullptr;

#pragma region GetInstance
/// @brief Method to return a pointer to the RobotContainer class.
/// @return Pointer to the RobotContainer class.
RobotContainer *RobotContainer::GetInstance()
{
    // Detrermine if the class has already been instantiated
    if (m_robotContainer == nullptr)
    {
        // Instantiate the class
        m_robotContainer = new RobotContainer();
    }

    // Return the class pointer
    return m_robotContainer;
}
#pragma endregion

#pragma region RobotContainer
/// @brief Method to configure the robot and SmartDashboard configuration.
RobotContainer::RobotContainer()
{
    m_leds.SetDefaultCommand(SetLedStatus(&m_leds, [this]() { return m_robotStatus;}));
  
    // ******************* //
    // * DRIVER CONTROLS * //
    // ******************* //

    // Array of run-once controls, organized like this for simplicity and readability
    std::pair<Button, frc2::CommandPtr> runOnceControls[] =
    {
        {constants::controller::A, ChassisZeroHeading(&m_chassis)},
        {constants::controller::B, FlipFieldCentricity(&m_chassis)},
        {constants::controller::X, ChassisXMode(&m_chassis)},

        {constants::controller::LeftStickButton,  ClimbDeploy(&m_climb)},
        {constants::controller::RightStickButton, ClimbRetract(&m_climb)}
    };

    // Configure the run-once controls
    for (auto& [button, command] : runOnceControls)
    {
        frc2::JoystickButton(&m_driveController, int(button)).OnTrue(std::move(command));
    }

    // This takes the axis inputs and drives the robot
    m_chassis.SetDefaultCommand(ChassisDrive(&m_chassis, GetSpeeds()));

    // This is effectively a shoot command, the flywheel should already be spun up
    // and the rest of the tower should be configured by the operator
    frc2::JoystickButton(&m_driveController, constants::controller::RightBumper)
        .OnTrue( std::move(SpindexerSetState(&m_spindexer, SpindexerState::Spindexing)))
        .OnFalse(std::move(SpindexerSetState(&m_spindexer, SpindexerState::Stopped)));

    // ********************* //
    // * OPERATOR CONTROLS * //
    // ********************* //

    // Array of run-once controls, organized like this for simplicity and readability
    std::pair<Button, frc2::CommandPtr> runOnceControlsOperator[] =
    {
        // Intake Controls
        {constants::controller::LeftBumper,  IntakeSetState(&m_intake, IntakeState::DeployedRollerOn)},
        {constants::controller::RightBumper, IntakeSetState(&m_intake, IntakeState::Stowed)},
        
        // Tower state
        {constants::controller::A, TowerAimHub(&m_tower)},
        {constants::controller::B, TowerAimPassZone(&m_tower)},
        {constants::controller::X, TowerManualControl(&m_tower, &m_manualTowerState)},

        {constants::controller::LeftStickButton,  frc2::InstantCommand{[&] { m_manualTowerState.hoodActuatorInches -= 2_in;}, {&m_tower}}.AndThen(TowerManualControl(&m_tower, &m_manualTowerState))},
        {constants::controller::RightStickButton, frc2::InstantCommand{[&] { m_manualTowerState.hoodActuatorInches += 2_in;}, {&m_tower}}.AndThen(TowerManualControl(&m_tower, &m_manualTowerState))},
    };

    // Configure the run-once controls
    for (auto& [button, command] : runOnceControlsOperator)
    {
        frc2::JoystickButton(&m_operatorController, int(button)).OnTrue(std::move(command));
    }

    // Operator POV controls
    std::pair<int, frc2::CommandPtr> runOnceControlsPOV[] =
    {
        // Manual tower controls
        // TODO: remove magic numbers via testing
        {constants::controller::Pov_0,   frc2::InstantCommand{[&] { m_manualTowerState.flywheelSpeed += 100_rpm;}, {&m_tower}}.AndThen(TowerManualControl(&m_tower, &m_manualTowerState))},
        {constants::controller::Pov_90,  frc2::InstantCommand{[&] { m_manualTowerState.turretAngle += 10_deg;}, {&m_tower}}.AndThen(TowerManualControl(&m_tower, &m_manualTowerState))},

        {constants::controller::Pov_180, frc2::InstantCommand{[&] { m_manualTowerState.flywheelSpeed -= 100_rpm;}, {&m_tower}}.AndThen(TowerManualControl(&m_tower, &m_manualTowerState))},
        {constants::controller::Pov_270, frc2::InstantCommand{[&] { m_manualTowerState.turretAngle -= 10_deg;}, {&m_tower}}.AndThen(TowerManualControl(&m_tower, &m_manualTowerState))},

    };

    for (auto& [button, command] : runOnceControlsPOV)
    {
        frc2::POVButton(&m_operatorController, button).OnTrue(std::move(command));
    }
}
#pragma endregion

#pragma region GetSpeeds
/// @brief Method to return the chassis speeds based on joystick inputs.
/// @return The chassis speeds based on joystick inputs.
std::function<frc::ChassisSpeeds()> RobotContainer::GetSpeeds()
{
    return [&]
    {
        // Return the chassis speeds based on joystick inputs
        return frc::ChassisSpeeds{
            -ChassisConstants::maxSpeed           * frc::ApplyDeadband( m_driveController.GetRawAxis(1), constants::controller::TranslationDeadZone),
            -ChassisConstants::maxSpeed           * frc::ApplyDeadband( m_driveController.GetRawAxis(0), constants::controller::TranslationDeadZone),
             ChassisConstants::maxAngularVelocity * frc::ApplyDeadband(-m_driveController.GetRawAxis(4), constants::controller::RotateDeadZone)
        };
    };
}
#pragma endregion

#pragma region GetExponentialValue
/// @brief Method to convert a joystick value from -1.0 to 1.0 to exponential mode.
/// @param joystickValue The raw joystick value.
/// @param exponent The exponential value.
/// @return The resulting exponential value.
double RobotContainer::GetExponentialValue(double joystickValue, double exponent)
{
    int    direction = (joystickValue < 0.0) ? -1 : 1;
    double output    = std::pow(std::abs(joystickValue), exponent) * direction;

    // Ensure the range of the output
    output = std::clamp(output, -1.0, 1.0);

    // Return the output value
    return output;
}
#pragma endregion

#pragma region ResetWheelAnglesToZero
/// @brief Method to reset the swerve wheel angles to zero position.
void RobotContainer::ResetWheelAnglesToZero()
{
    m_chassis.ResetWheelAnglesToZero();
}
#pragma endregion
