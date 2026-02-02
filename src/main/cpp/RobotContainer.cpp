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

    // Set the default command for the LEDs to reflect the robot status
    m_leds.SetDefaultCommand(SetLedStatus(&m_leds, [this]() { return m_robotStatus;}));
}
#pragma endregion

#pragma region InitializePathPlanner
/// @brief Method to initialize the Path Planner configuration.
void RobotContainer::InitializePathPlanner()
{
    // Register Commands
    pathplanner::NamedCommands::registerCommand("Shoot All",     std::move(ShootToHub(&m_spindexer, &m_tower)));
    pathplanner::NamedCommands::registerCommand("Stop Shooting", std::move(SpindexerSetState(&m_spindexer, SpindexerState::Stopped)));

    pathplanner::NamedCommands::registerCommand("Spin Up For Hub", std::move(TowerAimHub(&m_tower)));
    pathplanner::NamedCommands::registerCommand("Spin Up For Zone", std::move(TowerAimPassZone(&m_tower)));

    pathplanner::NamedCommands::registerCommand("Deploy Intake", std::move(IntakeSetState(&m_intake, IntakeState::DeployedRollerOn)));
    pathplanner::NamedCommands::registerCommand("Stow Intake", std::move(IntakeSetState(&m_intake, IntakeState::Stowed)));

    pathplanner::NamedCommands::registerCommand("Deploy Climb", std::move(ClimbDeploy(&m_climb)));
    pathplanner::NamedCommands::registerCommand("Retract Climb", std::move(ClimbRetract(&m_climb)));

    pathplanner::NamedCommands::registerCommand("X MODE", std::move(ChassisXMode(&m_chassis)));

    // Send the Auto-Chooser
    m_autoChooser = pathplanner::AutoBuilder::buildAutoChooser();

    auto location = frc::DriverStation::GetLocation();

    auto isCompetition = true;

    // TODO: Is the following source needed?
    // if (location)
    // {
    //     switch (location.value())
    //     {
    //         case 1:
    //         {
    //             m_autoChooser = pathplanner::AutoBuilder::buildAutoChooserFilter(
    //                 [=](const pathplanner::PathPlannerAuto& autoCommand)
    //                 {
    //                     return isCompetition ? autoCommand.GetName().starts_with("Close") : true;
    //                 }
    //             );
    //             break;
    //         }
    //         case 2:
    //         {
    //             m_autoChooser = pathplanner::AutoBuilder::buildAutoChooserFilter(
    //                 [=](const pathplanner::PathPlannerAuto& autoCommand)
    //                 {
    //                     return isCompetition ? autoCommand.GetName().starts_with("Middle") : true;
    //                 }
    //             );
    //             break;
    //         }
    //         case 3:
    //         {
    //             m_autoChooser = pathplanner::AutoBuilder::buildAutoChooserFilter(
    //                 [=](const pathplanner::PathPlannerAuto& autoCommand)
    //                 {
    //                     return isCompetition ? autoCommand.GetName().starts_with("Far") : true;
    //                 }
    //             );
    //             break;
    //         }
    //         default:
    //             break;
    //     }
    // }

    m_autoChooser = pathplanner::AutoBuilder::buildAutoChooser();

    frc::SmartDashboard::PutData("Auto Chooser", &m_autoChooser);    
}
#pragma endregion

#pragma region InitializeDriverControls
/// @brief Method to initialize the driver controls.
///    A                  - Zero Heading
///    B                  - Flip Field Centricity
///    X                  - Chassis X Mode
///    Left Stick Button  - Climb Deploy
///    Right Stick Button - Climb Retract
///    Right Bumper       - Spindexer Spindexing (while held)
void RobotContainer::InitializeDriverControls()
{
    // Set the default command for the chassis to be driving with the joystick inputs
    m_chassis.SetDefaultCommand(ChassisDrive(&m_chassis, GetSpeeds()));

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

    // This is effectively a shoot command, the flywheel should already be spun up
    // and the rest of the tower should be configured by the operator
    frc2::JoystickButton(&m_driveController, constants::controller::RightBumper)
        .OnTrue( std::move(SpindexerSetState(&m_spindexer, SpindexerState::Spindexing)))
        .OnFalse(std::move(SpindexerSetState(&m_spindexer, SpindexerState::Stopped)));
}
#pragma endregion

#pragma region InitializeOperatorControls
/// @brief Method to initialize the operator controls.
///    A                  - Tower Aim Hub
///    B                  - Tower Aim Pass Zone
///    X                  - Tower Manual Control
///    Left Bumper        - Intake Deployed with Roller On
///    Right Bumper       - Intake Stowed
///    Left Stick Button  - Decrease Hood Actuator Length by 2 inches
///    Right Stick Button - Increase Hood Actuator Length by 2 inches
///    POV Up             - Increase Flywheel Speed by 100 rpm
///    POV Right          - Increase Turret Angle by 10 degrees
///    POV Down           - Decrease Flywheel Speed by 100 rpm
///    POV Left           - Decrease Turret Angle by 10 degrees
void RobotContainer::InitializeOperatorControls()
{
    // Configure the X-box controller buttons to commands
    std::pair<Button, frc2::CommandPtr> runOnceControlsOperator[] =
    {
        // Tower state
        {constants::controller::A, TowerAimHub(&m_tower)},
        {constants::controller::B, TowerAimPassZone(&m_tower)},
        {constants::controller::B, TowerIdle(&m_tower)},
        {constants::controller::X, TowerManualControl(&m_tower, &m_manualTowerState)},

        // Intake Controls
        {constants::controller::LeftBumper,  IntakeSetState(&m_intake, IntakeState::DeployedRollerOn)},
        {constants::controller::RightBumper, IntakeSetState(&m_intake, IntakeState::Stowed)},
        
        // Manual tower controls
        {constants::controller::LeftStickButton,  frc2::InstantCommand{[&] { m_manualTowerState.hoodActuatorInches -= 2_in;}, {&m_tower}}.AndThen(TowerManualControl(&m_tower, &m_manualTowerState))},
        {constants::controller::RightStickButton, frc2::InstantCommand{[&] { m_manualTowerState.hoodActuatorInches += 2_in;}, {&m_tower}}.AndThen(TowerManualControl(&m_tower, &m_manualTowerState))},
    };

    // Configure the the X-box controller buttons
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

    // Configure the the X-box controller POV buttons
    for (auto& [button, command] : runOnceControlsPOV)
    {
        frc2::POVButton(&m_operatorController, button).OnTrue(std::move(command));
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
