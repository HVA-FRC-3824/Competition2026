#include "RobotContainer.h"

#define PATHFINDER_COMMAND(name, command) pathplanner::NamedCommands::registerCommand(name, std::move(command));

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

    // Set the Chassis default command to the drive command, which will run whenever no other command is using the
    // chassis subsystem. The drive command will take joystick inputs and convert them to chassis speeds.
    m_chassis.SetDefaultCommand(ChassisDrive(&m_chassis, GetSpeeds()));

    // Set the LEDs default command to an instant command that will update the LED state based on the current state
    // of the robot, and then set the LED mode based on the current LED mode. This will run whenever no other command
    // is using the LEDs subsystem, and will ensure that the LEDs are always displaying the correct state of the robot.
    m_leds.SetDefaultCommand(frc2::InstantCommand{[&]() 
        { 
            m_leds.SetRobotState(m_tower.GetState().mode, 
                                 m_climb.GetState() == ClimbState::Deployed, 
                                 m_spindexer.GetState() == SpindexerState::Spindexing, 
                                 m_tower.IsOnTarget()); 
        }, {&m_leds}}
        .AndThen(SetLedStatus(&m_leds, &m_ledMode))
    );

    m_intake.SetDefaultCommand(frc2::InstantCommand{[&]() { m_intake.JogPosition(0_V); }, {&m_intake}}.ToPtr());
}
#pragma endregion

#pragma region InitializePathPlanner
/// @brief Method to initialize the Path Planner configuration.
void RobotContainer::InitializePathPlanner()
{
    // Register Commands
    PATHFINDER_COMMAND("Shoot All",        ShootToHub(&m_spindexer, &m_tower));
    PATHFINDER_COMMAND("Stop Shooting",    SpindexerSetState(&m_spindexer, SpindexerState::Stopped));
    PATHFINDER_COMMAND("Spin Up For Hub",  TowerAimHub(&m_tower));
    PATHFINDER_COMMAND("Spin Up For Zone", TowerAimPassZone(&m_tower));
    PATHFINDER_COMMAND("Deploy Intake",    IntakeSetState(&m_intake, IntakeState::DeployedRollerOn).AndThen(frc2::InstantCommand{[=]() { m_intake.JogPosition(2_V); }, {&m_intake}}.ToPtr()).WithTimeout(0.5_s).AndThen(frc2::InstantCommand{[=]() { m_intake.JogPosition(0_V); }, {&m_intake}}.ToPtr()));
    PATHFINDER_COMMAND("Stow Intake",      IntakeSetState(&m_intake, IntakeState::Stowed));
    PATHFINDER_COMMAND("Deploy Climb",     ClimbDeploy(&m_climb));
    PATHFINDER_COMMAND("Retract Climb",    ClimbRetract(&m_climb));
    PATHFINDER_COMMAND("X MODE",           ChassisXMode(&m_chassis));

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
    // B            - Toggle between field centric and robot centric driving
    // X            - tap to toggle the chassis in a defensive position
    // Y            - Retracts the climb mechanism, double tap to deploy
    // Right Bumper - Deploy the intake mechanism, double tap to retract
    // Left Bumper  - Start spindexer, double tap to stop
    // POV Up       - Jog intake forwards (deploy)
    // POV Down     - Jog intake backwards (retract)
    // POV Left     - Tap to toggle between slow mode and sonic mode

    // A tuple of a button, a press once command, and a double-tap command
    std::tuple<Button_t, frc2::CommandPtr, std::optional<frc2::CommandPtr>> driverBindings[] =
    {
        // Chassis heading controls
        {constants::controller::A,          ChassisZeroHeading(&m_chassis),    std::nullopt},
        {constants::controller::B,          ToggleFieldCentricity(&m_chassis), std::nullopt},

        // Chassis module controls
        {constants::controller::X,          ChassisXMode(&m_chassis), std::nullopt},

        // Climb controls
        {constants::controller::Y,          ClimbRetract(&m_climb), ClimbDeploy(&m_climb)},
    };

    // Add the bindings to the driver controller
    for (auto &[button, command, command2] : driverBindings)
    {
        frc2::JoystickButton(&m_driveController, int(button)).Debounce(50_ms)
            .OnTrue(std::move(command))
        .MultiPress(2, 0.4_s)
            .OnTrue(command2.has_value() ? std::move(command2.value()) : frc2::cmd::None());
    }

    // Intake controls
    frc2::JoystickButton(&m_driveController, int(constants::controller::RightBumper)).
                         OnTrue(IntakeSetState(&m_intake, IntakeState::DeployedRollerOn)).
                         OnFalse(IntakeSetState(&m_intake, IntakeState::Stowed));

    // Spindexer Controls
    frc2::JoystickButton(&m_driveController, int(constants::controller::LeftBumper)).
                         OnTrue(SpindexerSetState(&m_spindexer, SpindexerState::Spindexing)).
                         OnFalse(SpindexerSetState(&m_spindexer, SpindexerState::Stopped));

    // Driver POV controls
    std::tuple<int, frc2::CommandPtr, std::optional<frc2::CommandPtr>> driverPOVBindings[] =
    {
        // Manual tower controls
        {constants::controller::Pov_0, frc2::InstantCommand{[&]() { m_intake.JogPosition(2_V); }, {&m_intake}}.ToPtr(), std::nullopt},

        // {constants::controller::Pov_90,  },

        {constants::controller::Pov_180, frc2::InstantCommand{[&]() { m_intake.JogPosition(-2_V); }, {&m_intake}}.ToPtr(), std::nullopt},
                
        {constants::controller::Pov_270, ToggleSlowMode(&m_chassis), std::nullopt},
    };

    for (auto &[direction, command, command2] : driverPOVBindings)
    {
        frc2::POVButton(&m_driveController, direction)
            .OnTrue(std::move(command))
            .OnFalse(command2.has_value() ? std::move(command2.value()) : frc2::cmd::None());
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
    
    // *** Manual Controls ***
    // Left Stick Button  - Lower the hood
    // Right Stick Button - Raise the hood
    // Up POV             - Increases flywheel speed
    // Right POV          - Turns the turret right
    // Down POV           - Decreases flywheel speed
    // Left POV           - Turns the turret left

    // A tuple of a button, a press once command and a double-tap command
    std::tuple<Button_t, frc2::CommandPtr, std::optional<frc2::CommandPtr>> operatorBindings[] =
    {   
        // Tower state
        {constants::controller::A, TowerAimHub(&m_tower),                             frc2::InstantCommand{[&]() {m_tower.m_turretOffset = 0_deg;}, {&m_tower}}.ToPtr()},
        {constants::controller::B, TowerAimPassZone(&m_tower),                        std::nullopt},
        {constants::controller::Y, TowerIdle(&m_tower),                               TowerAutomatic(&m_tower)},
        {constants::controller::X, TowerManualControl(&m_tower, &m_manualTowerState), std::nullopt},

        {constants::controller::LeftStickButton,  frc2::InstantCommand{[&] { m_manualTowerState.hoodActuatorDistance -= 0.1;}, {&m_tower}}
                                                    .AndThen(TowerManualControl(&m_tower, &m_manualTowerState)), std::nullopt},
        
        {constants::controller::RightStickButton, frc2::InstantCommand{[&] { m_manualTowerState.hoodActuatorDistance += 0.1;}, {&m_tower}}
                                                    .AndThen(TowerManualControl(&m_tower, &m_manualTowerState)), std::nullopt},
    
        {constants::controller::RightBumper, frc2::InstantCommand{[&]() {m_tower.m_turretOffset -= 5_deg;}, {&m_tower}}.ToPtr(), std::nullopt},
        {constants::controller::LeftBumper,  frc2::InstantCommand{[&]() {m_tower.m_turretOffset += 5_deg;}, {&m_tower}}.ToPtr(), std::nullopt},
    };

    // Add the bindings to the operator controller
    for (auto &[button, once, twice] : operatorBindings)
    {
        frc2::JoystickButton(&m_operatorController, int(button)).Debounce(50_ms)
            .OnTrue(std::move(once))
        .MultiPress(2, 0.4_s)
            .OnTrue(twice.has_value() ? std::move(twice.value()) : frc2::cmd::None());
    }

    // Operator POV controls
    std::pair<int, frc2::CommandPtr> operatorPOVBindings[] =
    {
        // Manual tower controls
        {constants::controller::Pov_0,   frc2::InstantCommand{[&] { m_manualTowerState.flywheelSpeed += 2.5_tps;}, {&m_tower}}
                                            .AndThen(TowerManualControl(&m_tower, &m_manualTowerState))},

        {constants::controller::Pov_90,  frc2::InstantCommand{[&] { m_manualTowerState.turretAngle += 5_deg;}, {&m_tower}}
                                            .AndThen(TowerManualControl(&m_tower, &m_manualTowerState))},

        {constants::controller::Pov_180, frc2::InstantCommand{[&] { m_manualTowerState.flywheelSpeed -= 2.5_tps;}, {&m_tower}}
                                            .AndThen(TowerManualControl(&m_tower, &m_manualTowerState))},

        {constants::controller::Pov_270, frc2::InstantCommand{[&] { m_manualTowerState.turretAngle -= 5_deg;}, {&m_tower}}
                                            .AndThen(TowerManualControl(&m_tower, &m_manualTowerState))},
    };

    for (auto &[direction, command] : operatorPOVBindings)
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
/// The left joystick forward sets the forward speed, the left joystick right sets the strafe speed,
/// and the right joystick right sets the rotation speed.
/// The inputs are also passed through a deadband function to prevent small joystick inputs from
/// causing the robot to move. 
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
