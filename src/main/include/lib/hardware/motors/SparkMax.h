#pragma once

#include "Motor.h"

#include <iostream>
#include <numbers>

#include <frc/RobotController.h>
#include <frc/RobotBase.h>

#include <frc/system/plant/DCMotor.h>
#include <frc/simulation/DCMotorSim.h>
#include <frc/system/plant/LinearSystemId.h>

#include <frc/controller/SimpleMotorFeedforward.h>

#include <rev/SparkMax.h>
#include <rev/sim/SparkMaxSim.h>
#include <rev/SparkLowLevel.h>
#include <rev/config/SparkMaxConfig.h>

namespace hardware
{

namespace motor
{
    
    class SparkMax : public Motor
    {
        
        public:

            inline SparkMax(CANid_t CANid, MotorConfiguration config, MotorType type, frc::DCMotor motorModel, units::kilogram_square_meter_t simMomentOfInertia = 0.001_kg_sq_m) 
            : Motor{
                frc::sim::DCMotorSim{
                    frc::LinearSystemId::DCMotorSystem(
                            motorModel,
                            simMomentOfInertia,
                            1
                        ),
                        motorModel
                    }
                },
                m_motor{CANid, rev::spark::SparkLowLevel::MotorType::kBrushless},
                m_angleEncoder{m_motor.GetEncoder()}, 
                m_turnClosedLoopController{m_motor.GetClosedLoopController()},

                m_feedforward{config.S * 1_V, config.V * 1_V * 1_s / 1_tr, config.A * 1_V * 1_s * 1_s / 1_tr},

                m_motorModel{motorModel},
                m_sparkSim{&m_motor, &m_motorModel},
                m_config{config}
            {
                ConfigureMotor(config);
            }

            inline void ConfigureMotor(MotorConfiguration config) override
            {
                // Configure the angle motor
                rev::spark::SparkMaxConfig sparkMaxConfig{};

                // Configure the motor controller
                sparkMaxConfig
                    .SetIdleMode(config.breakMode 
                                        ? rev::spark::SparkBaseConfig::IdleMode::kBrake 
                                        : rev::spark::SparkBaseConfig::IdleMode::kCoast)
                    .SmartCurrentLimit(config.CurrentLimit.value());

                // Configure encoder conversion factors (ensure units are in turns and turns per second)
                sparkMaxConfig.encoder
                    .PositionConversionFactor(1) // Position in turns
                    .VelocityConversionFactor(1); // Velocity in turns per second (RPM / 60)

                // Configure the closed loop controller
                sparkMaxConfig.closedLoop
                    .SetFeedbackSensor(rev::spark::FeedbackSensor::kPrimaryEncoder)
                    .Pid(config.P, config.I, config.D);
                    
                sparkMaxConfig.closedLoop.maxMotion
                        .MaxVelocity(config.velocityLimit)
                        .MaxAcceleration(config.accelerationLimit);

                // Write the configuration to the motor controller
                auto status = m_motor.Configure(sparkMaxConfig, 
                                               rev::ResetMode::kResetSafeParameters, 
                                               rev::PersistMode::kPersistParameters);

                // Report configuration status
                if (status != rev::REVLibError::kOk)
                {
                    std::cerr << "***** ERROR: Could not configure SparkMax motor (CAN ID: " 
                              << m_motor.GetDeviceId() << "). Error code: " << static_cast<int>(status) << std::endl;
                }
                else
                {
                    std::cout << "SparkMax motor (CAN ID: " << m_motor.GetDeviceId() 
                              << ") configured successfully." << std::endl;
                }
            }

            inline void SetReferenceState(double motorInput, MotorInput inputType) override
            {   
                switch (inputType)
                {
                    case MotorInput::ARBITRARY:
                        // Set duty cycle output [-1, 1]
                        m_turnClosedLoopController.SetSetpoint(motorInput, 
                                                                rev::spark::SparkMax::ControlType::kDutyCycle,
                                                                rev::spark::ClosedLoopSlot::kSlot0, 
                                                                m_feedforward.Calculate(0_tps).value(), // TODO: make this work with V and A
                                                                rev::spark::SparkClosedLoopController::ArbFFUnits::kVoltage);
                        break;

                    case MotorInput::VELOCITY:
                        // Set velocity control with feedforward
                        m_turnClosedLoopController.SetSetpoint(motorInput, 
                                                                rev::spark::SparkMax::ControlType::kVelocity,
                                                                rev::spark::ClosedLoopSlot::kSlot0, 
                                                                m_feedforward.Calculate(0_tps).value(), // TODO: make this work with V and A
                                                                rev::spark::SparkClosedLoopController::ArbFFUnits::kVoltage);
                        break;

                    case MotorInput::VOLTAGE:
                        // Set voltage control with static friction compensation
                        m_turnClosedLoopController.SetSetpoint(motorInput + m_feedforward.Calculate(0_tps).value(), // TODO: make this work with V and A
                                                                rev::spark::SparkMax::ControlType::kVoltage);
                        break;

                    case MotorInput::POSITION:
                        // Use standard position control
                        m_turnClosedLoopController.SetSetpoint(motorInput, 
                                                                rev::spark::SparkMax::ControlType::kPosition,
                                                                rev::spark::ClosedLoopSlot::kSlot0, 
                                                                m_feedforward.Calculate(0_tps).value(), // TODO: make this work with V and A
                                                                rev::spark::SparkClosedLoopController::ArbFFUnits::kVoltage);
                        break;
                }
            }

            inline units::turn_t GetPosition() override
            {
                if (frc::RobotBase::IsSimulation())
                    return 1_tr * m_motorSim.GetAngularPosition().value() / (2 * std::numbers::pi);
                
                return 1_tr * m_angleEncoder.GetPosition();
            }

            inline units::turns_per_second_t GetVelocity() override
            {
                if (frc::RobotBase::IsSimulation())
                    return 1_tps * m_motorSim.GetAngularVelocity().value() / (2 * std::numbers::pi);

                // REV encoder returns velocity in RPM by default, convert to turns per second
                return 1_tps * m_angleEncoder.GetVelocity() / 60.0;
            }

            // This is in turns, not sure if this is affected by any conversion factors
            inline void OffsetEncoder(double offset) override
            {
                if (frc::RobotBase::IsSimulation())
                {
                    m_motorSim.SetAngle(units::radian_t{offset * 2 * std::numbers::pi});
                } else
                {
                    m_angleEncoder.SetPosition(offset);
                }
            }

            inline void SimPeriodic() override
            {
                // Get the applied output from the SparkMax simulation
                auto appliedOutput = m_sparkSim.GetAppliedOutput();
                auto batteryVoltage = frc::RobotController::GetBatteryVoltage();
                
                // Apply voltage to the motor simulation
                m_motorSim.SetInputVoltage(appliedOutput * batteryVoltage);
                m_motorSim.Update(20_ms);
                
                // Update the SparkMax simulation with new motor state
                // Convert radians per second to RPM for SparkMax sim
                auto velocityRPM = m_motorSim.GetAngularVelocity().value() * 60.0 / (2.0 * std::numbers::pi);
                m_sparkSim.iterate(velocityRPM, batteryVoltage.value(), 0.02);
                
                // Update encoder position in the SparkMax sim
                // Convert radians to turns
                auto positionTurns = m_motorSim.GetAngularPosition().value() / (2.0 * std::numbers::pi);
                m_sparkSim.SetPosition(positionTurns);
            }

        private:

            rev::spark::SparkMax                      m_motor;
            rev::spark::SparkRelativeEncoder          m_angleEncoder;
            rev::spark::SparkClosedLoopController     m_turnClosedLoopController;

            frc::SimpleMotorFeedforward<units::turns> m_feedforward;

            frc::DCMotor                              m_motorModel;
            rev::spark::SparkMaxSim                   m_sparkSim;

            MotorConfiguration                        m_config;

    };

}

}