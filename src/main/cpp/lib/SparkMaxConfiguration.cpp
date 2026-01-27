#include "lib/SparkMaxConfiguration.h"

#pragma region SparkMaxConfiguration
void SparkMaxConfiguration(rev::spark::SparkMax *motor,
                          units::ampere_t currentLimit,
                          bool   breakMode,
                          bool   continuousWrap,
                          double P,
                          double I,
                          double D,
                          double S,
                          double V,
                          double A,
                          units::turns_per_second_t         velocityLimit,
                          units::turns_per_second_squared_t accelerationLimit)
{
    // Configure the angle motor
    rev::spark::SparkMaxConfig sparkMaxConfig{};

    // Configure the motor controller
    sparkMaxConfig
        .SetIdleMode(breakMode 
                            ? rev::spark::SparkBaseConfig::IdleMode::kBrake 
                            : rev::spark::SparkBaseConfig::IdleMode::kCoast)
        .SmartCurrentLimit(currentLimit.value());

    // Configure the closed loop controller
    sparkMaxConfig.closedLoop
        .SetFeedbackSensor(rev::spark::FeedbackSensor::kPrimaryEncoder)
        .Pid(P, I, D)
        .PositionWrappingEnabled(continuousWrap);
        
    sparkMaxConfig.closedLoop.maxMotion
            .CruiseVelocity(velocityLimit.value())
            .MaxAcceleration(accelerationLimit.value());

    // Write the configuration to the motor controller
    auto status = motor->Configure(sparkMaxConfig, 
                                  rev::ResetMode::kResetSafeParameters, 
                                  rev::PersistMode::kPersistParameters);

    // Report configuration status
    if (status != rev::REVLibError::kOk)
    {
        std::cerr << "***** ERROR: Could not configure SparkMax motor (CAN ID: " 
                    << motor->GetDeviceId() << "). Error code: " << static_cast<int>(status) << std::endl;
    }
    else
    {
        std::cout << "SparkMax motor (CAN ID: " << motor->GetDeviceId() 
                    << ") configured successfully." << std::endl;
    }
}
#pragma endregion