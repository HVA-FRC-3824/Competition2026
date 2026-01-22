#include "lib/TalonFXConfiguration.h"

#pragma region TalonFXConfiguration
void TalonFXConfiguration(ctre::phoenix6::hardware::TalonFX *motor,
                          units::ampere_t currentLimit,
                          bool   breakMode,
                          double P,
                          double I,
                          double D,
                          double S,
                          double V,
                          double A,
                          units::turns_per_second_t         velocityLimit,
                          units::turns_per_second_squared_t accelerationLimit)
{
    constexpr int MAX_CONFIG_RETRIES = 3;
    
    // Create the TalonFX configuration
    ctre::phoenix6::configs::TalonFXConfiguration talonFXConfiguration{};

    // Configure Motor Output settings
    ctre::phoenix6::configs::MotorOutputConfigs &motorOutputConfigs = talonFXConfiguration.MotorOutput;
    motorOutputConfigs.NeutralMode = breakMode
        ? ctre::phoenix6::signals::NeutralModeValue::Brake
        : ctre::phoenix6::signals::NeutralModeValue::Coast;

    // Configure Current Limits
    ctre::phoenix6::configs::CurrentLimitsConfigs &currentLimitsConfigs = talonFXConfiguration.CurrentLimits;
    // Also set supply current limit for battery protection
    currentLimitsConfigs.SupplyCurrentLimit = currentLimit;
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

    /// *** SIMULATION STUFF *** ///
    auto& talonFXSim = motor->GetSimState();
    talonFXSim.Orientation = ctre::phoenix6::sim::ChassisReference::CounterClockwise_Positive;
    // TODO: This only supports x60 and x44 krakens, I'm not sure how we want to handle this
    // This isnt a big issue, and the todo can be removed without much error or impact
    talonFXSim.SetMotorType(ctre::phoenix6::sim::TalonFXSimState::MotorType::KrakenX60);
}
#pragma endregion