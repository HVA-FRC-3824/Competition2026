#pragma once

#include "lib/Logging.h"

#include "lib/hardware/motors/TalonFX.h"
#include "lib/hardware/motors/SparkMax.h"

#include "lib/hardware/sensors/DigitalInput.h"

#include "Constants.h"

class Volcano : public frc2::SubsystemBase
{
public:

    explicit Volcano();

    void SetKicker(bool running);

    void SetIndexers(bool running);

    void SetFlywheel(units::turns_per_second_t targetSpeed = constants::volcano::targetFlywheelSpeed);

    bool IsBallDetected();

    bool IsFlywheelAtSpeed();

    void Periodic() override;

private:

    hardware::motor::TalonFX  m_flywheelMotor
    {
        constants::volcano::flywheelMotorCANid, constants::volcano::flywheelMotorConfig, frc::DCMotor::Falcon500()
    };

    hardware::motor::SparkMax m_indexerMotors[2]
    {
         hardware::motor::SparkMax{constants::volcano::firstIndexerMotorCANid,
                                  constants::volcano::indexerMotorConfig, frc::DCMotor::NEO()},
          hardware::motor::SparkMax{constants::volcano::secondIndexerMotorCANid,
                                   constants::volcano::indexerMotorConfig, frc::DCMotor::NEO()}
    };

    hardware::motor::SparkMax m_kickMotor{constants::volcano::kickerMotorCANid,
                                          constants::volcano::kickMotorConfig, frc::DCMotor::NEO()};

    hardware::sensor::DigitalInput m_ballSensor{constants::volcano::ballSensorDIOPort};

    units::turns_per_second_t m_targetSpeed;
};