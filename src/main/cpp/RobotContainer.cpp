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
    // // Configure the chassis default command
    m_chassis.SetDefaultCommand(ChassisDrive(&m_chassis, GetChassisSpeeds()));
    m_leds.SetDefaultCommand(SetLedStatus(&m_leds, [this]() { return m_robotStatus;}));
  
    // ******************* //
    // * DRIVER CONTROLS * //
    // ******************* //

    // Array of run-once controls, organized like this for simplicity and readability
    std::pair<Button, frc2::CommandPtr> runOnceControls[] =
    {
        {constants::controller::A,           ChassisZeroHeading(&m_chassis)},
        {constants::controller::B,           FlipFieldCentricity(&m_chassis)},
        {constants::controller::Y,           frc2::InstantCommand{ [&] {m_robotStatus;}, {&m_leds} }.ToPtr()},
        {constants::controller::X,           ChassisXMode(&m_chassis)}, // Toggle
        {constants::controller::LeftBumper,  frc2::WaitCommand{1_s}.ToPtr()}, // When pressed, shoot one ball
    };

    // Configure the run-once controls
    for (auto& [button, command] : runOnceControls)
    {
        frc2::JoystickButton(&m_driveController, int(button)).OnTrue(std::move(command));
    }

    // frc2::JoystickButton(&m_driveController, constants::controller::A).OnTrue(new frc2::InstantCommand{[this] { m_chassis.ZeroHeading(); }, {&m_chassis}});

    // ********************* //
    // * OPERATOR CONTROLS * //
    // ********************* //


    // ********** //
    // * CAMERA * //
    // ********** //


    // cs::UsbCamera camera = frc::CameraServer::StartAutomaticCapture();

    // // Set the resolution and frame rate of the camera
    // camera.SetResolution(640, 480); // Set resolution to 640x480
    // camera.SetFPS(30);             // Set frame rate to 30 FPS
}
#pragma endregion

#pragma region GetChassisSpeeds
/// @brief Method to return the chassis speeds based on joystick inputs.
/// @return The chassis speeds based on joystick inputs.
std::function<frc::ChassisSpeeds()> RobotContainer::GetChassisSpeeds()
{
    return [&]()
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
    double absValue  = std::abs(joystickValue);
    double output    = std::pow(absValue, exponent) * direction;

    // Ensure the range of the output
    if (output < -1.0) output = -1.0;
    if (output > 1.0)  output =  1.0;

    // Return the output value
    return output;
}
#pragma endregion
