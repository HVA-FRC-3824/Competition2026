#include "lib/TalonFXConfiguration.h"

#pragma region TalonFXConfiguration
/// @brief Configures a TalonFX motor with the specified parameters
/// @param motor Pointer to the TalonFX motor to configure 
/// @param currentLimit The current limit for the motor
/// @param breakMode True to set the motor to brake mode, false for coast mode 
/// @param continuousWrap Tre to set the motor controller to continuous wrap 
/// @param P The proportional gain for the motor's PID controller
/// @param I The integral gain for the motor's PID controller
/// @param D The derivative gain for the motor's PID controller
/// @param S The static feedforward gain for the motor
/// @param V The velocity feedforward gain for the motor
/// @param A The acceleration feedforward gain for the motor
/// @param velocityLimit The maximum velocity for Motion Magic control
/// @param accelerationLimit The maximum acceleration for Motion Magic control
/// @param sensorToMechanismRatio The ratio of sensor rotation to mechanism rotation (default: 1.0)
void TalonFXConfiguration(ctre::phoenix6::hardware::TalonFX *motor,
                          units::ampere_t                    currentLimit,
                          bool                               breakMode,
                          bool                               continuousWrap,
                          double P, double I, double D,
                          double S, double V, double A,
                          units::turns_per_second_t         velocityLimit,
                          units::turns_per_second_squared_t accelerationLimit,
                          double                            sensorToMechanismRatio)
{
    constexpr int MAX_CONFIG_RETRIES = 3;
    
    // Create the TalonFX configuration
    ctre::phoenix6::configs::TalonFXConfiguration talonFXConfiguration{};

    // Configure Closed Loop General settings
    talonFXConfiguration.ClosedLoopGeneral.ContinuousWrap = continuousWrap;

    // Configure Feedback settings
    ctre::phoenix6::configs::FeedbackConfigs &feedbackConfigs = talonFXConfiguration.Feedback;
    feedbackConfigs.SensorToMechanismRatio = sensorToMechanismRatio;

    // Configure Motor Output settings
    ctre::phoenix6::configs::MotorOutputConfigs &motorOutputConfigs = talonFXConfiguration.MotorOutput;
    motorOutputConfigs.NeutralMode = breakMode
        ? ctre::phoenix6::signals::NeutralModeValue::Brake
        : ctre::phoenix6::signals::NeutralModeValue::Coast;

    // Configure Current Limits
    ctre::phoenix6::configs::CurrentLimitsConfigs &currentLimitsConfigs = talonFXConfiguration.CurrentLimits;
    currentLimitsConfigs.SupplyCurrentLimit       = currentLimit;
    currentLimitsConfigs.SupplyCurrentLimitEnable = true;

    // Configure PID and Feedforward (Slot 0)
    ctre::phoenix6::configs::Slot0Configs &slot0Configs = talonFXConfiguration.Slot0;
    slot0Configs.kP = P;
    slot0Configs.kI = I;
    slot0Configs.kD = D;
    slot0Configs.kS = S;
    slot0Configs.kV = V;
    slot0Configs.kA = A;

    // Configure MotionMagic parameters
    ctre::phoenix6::configs::MotionMagicConfigs &motionMagicConfigs = talonFXConfiguration.MotionMagic;
    motionMagicConfigs.MotionMagicCruiseVelocity = units::turns_per_second_t{velocityLimit};
    motionMagicConfigs.MotionMagicAcceleration   = units::turns_per_second_squared_t{accelerationLimit};
    motionMagicConfigs.MotionMagicJerk           = units::turns_per_second_cubed_t{0.0}; // Do not limit acceleration growth

    // Try to apply the configuration with retries
    ctre::phoenix::StatusCode status = ctre::phoenix::StatusCode::StatusCodeNotInitialized;
    for (int attempt = 0; attempt < MAX_CONFIG_RETRIES; attempt++)
    {
        // Apply the configuration
        status = motor->GetConfigurator().Apply(talonFXConfiguration);
        if (status.IsOK())
        {
            break;
        }

        // Small delay before retry
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    // Report configuration status
    if (!status.IsOK())
    {
        std::cerr << "***** ERROR: Could not configure TalonFX motor (CAN ID: " 
                    << motor->GetDeviceID() << "). Error: " << status.GetName() 
                    << " (" << status.GetDescription() << ")" << std::endl;
    }
    else
    {
        std::cout << "TalonFX motor (CAN ID: " << motor->GetDeviceID() 
                    << ") configured successfully." << std::endl;
    }
}
#pragma endregion