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
    // Initialize the Path Planner configuration
    InitializePathPlanner();

    // Initialize the driver and operator controls
    InitializeDriverControls();
    InitializeOperatorControls();

    m_chassis.SetDefaultCommand(ChassisDrive(&m_chassis, GetSpeeds()));
    m_leds.SetDefaultCommand(frc2::InstantCommand{[&]() 
        { 
            m_leds.SetRobotState(m_tower.GetState().mode, 
                                 m_climb.GetState() == ClimbState::Deployed, 
                                 m_spindexer.GetState() == SpindexerState::Spindexing, 
                                 m_tower.IsOnTarget()); 
        }, {&m_leds}}
        .AndThen(SetLedStatus(&m_leds, &m_ledMode))
    );

}
#pragma endregion

#pragma region InitializePathPlanner
/// @brief Method to initialize the Path Planner configuration.
void RobotContainer::InitializePathPlanner()
{
    // Register Commands
    pathplanner::NamedCommands::registerCommand("Shoot All",        std::move(ShootToHub(&m_spindexer, &m_tower)));
    pathplanner::NamedCommands::registerCommand("Stop Shooting",    std::move(SpindexerSetState(&m_spindexer, SpindexerState::Stopped)));

    pathplanner::NamedCommands::registerCommand("Spin Up For Hub",  std::move(TowerAimHub(&m_tower)));
    pathplanner::NamedCommands::registerCommand("Spin Up For Zone", std::move(TowerAimPassZone(&m_tower)));

    pathplanner::NamedCommands::registerCommand("Deploy Intake",    std::move(IntakeSetState(&m_intake, IntakeState::DeployedRollerOn)));
    pathplanner::NamedCommands::registerCommand("Stow Intake",      std::move(IntakeSetState(&m_intake, IntakeState::Stowed)));

    pathplanner::NamedCommands::registerCommand("Deploy Climb",     std::move(ClimbDeploy(&m_climb)));
    pathplanner::NamedCommands::registerCommand("Retract Climb",    std::move(ClimbRetract(&m_climb)));

    pathplanner::NamedCommands::registerCommand("X MODE",           std::move(ChassisXMode(&m_chassis)));

    // Send the Auto-Chooser
    m_autoChooser = pathplanner::AutoBuilder::buildAutoChooser();
    frc::SmartDashboard::PutData("Auto Chooser", &m_autoChooser);
}
#pragma endregion

#pragma region InitializeDriverControls
/// @brief Method to initialize the driver controls.
void RobotContainer::InitializeDriverControls()
{
    /// *** Bindings *** ///
    // A            - Zeros the gyro heading
    // B            - Toggles between field centric and robot centric driving
    // X            - Locks the chassis in a defensive position
    // Y            - Retracts the climb mechanism, double tap to deploy
    // Right Bumper - Deploy the intake mechanism, double tap to retract
    // Left Bumper  - Start spindexer, double tap to stop

    // A tuple of a button, a press once command, and a double-tap command
    std::tuple<Button, frc2::CommandPtr, std::optional<frc2::CommandPtr>> driverBindings[] =
    {
        // Chassis heading controls
        {constants::controller::A,          ChassisZeroHeading(&m_chassis),    std::nullopt},
        {constants::controller::B,          ToggleFieldCentricity(&m_chassis), std::nullopt},

        // Chassis module controls
        {constants::controller::X,          ChassisXMode(&m_chassis),          std::nullopt},

        // Climb controls
        {constants::controller::Y,          ClimbRetract(&m_climb), ClimbDeploy(&m_climb)},

        // Intake controls
        {constants::controller::RightBumper, IntakeSetState(&m_intake, IntakeState::DeployedRollerOn),
                                             IntakeSetState(&m_intake, IntakeState::Stowed)},
    
        // Spindexer Controls
        {constants::controller::LeftBumper,  SpindexerSetState(&m_spindexer, SpindexerState::Spindexing),
                                             SpindexerSetState(&m_spindexer, SpindexerState::Stopped)}
    };

    // Add the bindings to the driver controller
    for (auto& [button, once, twice] : driverBindings)
    {
        frc2::JoystickButton(&m_driveController, int(button)).Debounce(50_ms)
            .OnTrue(std::move(once))
        .MultiPress(2, 0.4_s)
            .OnTrue(twice.has_value() ? std::move(twice.value()) : frc2::cmd::None());
    }
}
#pragma endregion

#pragma region InitializeOperatorControls
void RobotContainer::InitializeOperatorControls()
{
    /// *** Bindings *** ///
    // A - Aim the tower to the hub
    // B - Aim the tower to pass to our alliance zone
    // Y - Idles the tower, double tap to make the tower automatic based on the current position of the bot
    // X - Aims the tower based on manual parameters
    //
    // *** Manual Controls ***
    // Left Stick Button  - Lower the hood
    // Right Stick Button - Raise the hood
    // Up POV             - Increases flywheel speed
    // Right POV          - Turns the turret right
    // Down POV           - Decreases flywheel speed
    // Left POV           - Turns the turret left

    // A tuple of a button, a press once command and a double-tap command
    std::tuple<Button, frc2::CommandPtr, std::optional<frc2::CommandPtr>> operatorBindings[] =
    {   
        // Tower state
        {constants::controller::A, TowerAimHub(&m_tower),                             std::nullopt},
        {constants::controller::B, TowerAimPassZone(&m_tower),                        std::nullopt},
        {constants::controller::Y, TowerIdle(&m_tower),                               TowerAutomatic(&m_tower)},
        {constants::controller::X, TowerManualControl(&m_tower, &m_manualTowerState), std::nullopt},

        {constants::controller::LeftStickButton,  frc2::InstantCommand{[&] { m_tower.TestActuator(-0.2);}, {&m_tower}}.ToPtr(), std::nullopt},
        {constants::controller::RightStickButton, frc2::InstantCommand{[&] { m_tower.TestActuator(0.2);}, {&m_tower}}.ToPtr(), std::nullopt},

        // {constants::controller::LeftStickButton,  frc2::InstantCommand{[&] { m_manualTowerState.hoodActuatorInches -= 2_in;}, {&m_tower}}.AndThen(TowerManualControl(&m_tower, &m_manualTowerState)), std::nullopt},
        // {constants::controller::RightStickButton, frc2::InstantCommand{[&] { m_manualTowerState.hoodActuatorInches += 2_in;}, {&m_tower}}.AndThen(TowerManualControl(&m_tower, &m_manualTowerState)), std::nullopt},
    };

    // Add the bindings to the operator controller
    for (auto& [button, once, twice] : operatorBindings)
    {
        frc2::JoystickButton(&m_driveController, int(button)).Debounce(50_ms)
            .OnTrue(std::move(once))
        .MultiPress(2, 0.4_s)
            .OnTrue(twice.has_value() ? std::move(twice.value()) : frc2::cmd::None());
    }

    // Operator POV controls
    std::pair<int, frc2::CommandPtr> operatorPOVBindings[] =
    {
        // Manual tower controls
        {constants::controller::Pov_0,   frc2::InstantCommand{[&] { m_manualTowerState.flywheelSpeed += 10_rpm;}, {&m_tower}}.AndThen(TowerManualControl(&m_tower, &m_manualTowerState))},
        {constants::controller::Pov_90,  frc2::InstantCommand{[&] { m_manualTowerState.turretAngle += 10_deg;}, {&m_tower}}.AndThen(TowerManualControl(&m_tower, &m_manualTowerState))},

        {constants::controller::Pov_180, frc2::InstantCommand{[&] { m_manualTowerState.flywheelSpeed -= 10_rpm;}, {&m_tower}}.AndThen(TowerManualControl(&m_tower, &m_manualTowerState))},
        {constants::controller::Pov_270, frc2::InstantCommand{[&] { m_manualTowerState.turretAngle -= 10_deg;}, {&m_tower}}.AndThen(TowerManualControl(&m_tower, &m_manualTowerState))},

    };

    for (auto& [direction, command] : operatorPOVBindings)
    {
        frc2::POVButton(&m_operatorController, direction).OnTrue(std::move(command));
    }
}
#pragma endregion

#pragma region ResetWheelAnglesToZero
/// @brief Method to reset the swerve wheel angles to zero position.
void RobotContainer::ResetWheelAnglesToZero()
{
    // Reset the wheel angles to zero position
    m_chassis.ResetWheelAnglesToZero();
}
#pragma endregion

#pragma region ResetGyroAngle
/// @brief Method to reset the gyro angle to zero position.
void RobotContainer::ResetGyroAngle()
{
    // Reset the gyro angle to zero position
    m_chassis.ResetGyroAngle();
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
            -ChassisConstants::MaximumSpeed           * frc::ApplyDeadband( m_driveController.GetRawAxis(1), constants::controller::TranslationDeadZone),
            -ChassisConstants::MaximumSpeed           * frc::ApplyDeadband( m_driveController.GetRawAxis(0), constants::controller::TranslationDeadZone),
             ChassisConstants::MaximumAngularVelocity * frc::ApplyDeadband(-m_driveController.GetRawAxis(4), constants::controller::RotateDeadZone)
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
