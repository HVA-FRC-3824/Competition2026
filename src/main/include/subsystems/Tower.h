#pragma once

#pragma region Includes
#include <functional>
#include <numeric>

#include <units/angle.h>
#include <units/length.h>

#include <frc2/command/SubsystemBase.h>
#include <frc/DriverStation.h>

#include <frc/geometry/Pose2d.h>
#include <frc/kinematics/ChassisSpeeds.h>

#include <frc/Servo.h>

#include <ctre/phoenix6/TalonFX.hpp>

#include <photon/PhotonCamera.h>

#include "lib/TalonFXConfiguration.h"
#include "lib/SparkMaxConfiguration.h"
#include "lib/Logging.h"

#include "Constants.h"
#include "ConstantsRoboRio.h"
#pragma endregion

#pragma region StateStructures
enum TowerMode
{
    Idle,
    ShootingToHub,
    PassingToAdjacentZone,
    ManualControl
};

struct TowerState
{
    TowerMode mode;
    units::degree_t           turretAngle;
    units::turns_per_second_t flywheelSpeed;
    units::inch_t             hoodActuatorInches;
};
#pragma endregion

#pragma region TowerConstants
namespace TowerConstants
{
    // This is -1 to 1, really 0 to 1
    constexpr auto constantFlywheelSpeed    = 0.8;

    // TODO: Make these real. These values will be changed over the course of the season probably
    constexpr auto MinAngle        = -180_deg;
    constexpr auto MaxAngle        =  180_deg;

    // TODO: test these lengths, they're most likely accurate
    // I got these from team 102 from 2022, they used the same actuator
    constexpr auto MaxLength        = 14.336_in;
    constexpr auto MinLength        = 8.946_in;

    // Comes from 102 too
    constexpr auto ActuatorLowerBound = -0.95;
    constexpr auto ActuatorUpperBound =  0.95;

    // From inches to 0-1 range
    constexpr auto ActuatorDistanceConversionFactor = (TowerConstants::MaxLength - TowerConstants::MinLength);
}
#pragma endregion

class Tower : public frc2::SubsystemBase
{
    public:
        
        explicit    Tower(std::function<frc::Pose2d()> chassisPoseSupplier, std::function<frc::ChassisSpeeds()> chassisSpeedsSupplier);

        void        SetState(TowerState newState);
        TowerState  GetState();

        void        Periodic() override;

        void        TestActuator(double position) { m_hoodActuator.SetSpeed(position); }

        void        AimUsingTurretCamera(bool usingTurretCamera) { m_usingTurretCamera = usingTurretCamera; }

    private: 
    
        void SetActuator(units::inch_t position);

        void SetFlywheel(units::turns_per_second_t input);
        
        void SetTurret(units::degree_t angle);

        TowerState CalculateShot(frc::Translation2d relativeDistance, frc::ChassisSpeeds chassisSpeed);

        bool                                    m_isBlue = frc::DriverStation::GetAlliance().value_or(frc::DriverStation::Alliance::kBlue) 
                                                                == frc::DriverStation::Alliance::kBlue;

        bool                                    m_usingTurretCamera = true;

        // Do not use this to do pose estimatation. Because its on a turret, it is unreliable
        photon::PhotonCamera                    m_turretCam{"turretCam"};

        std::function<frc::Pose2d()>            m_chassisPoseSupplier;
        std::function<frc::ChassisSpeeds()>     m_chassisSpeedsSupplier;   

        TowerState                              m_state{TowerMode::ManualControl, 0_deg, 0_rpm, 0_in}; 

        ctre::phoenix6::hardware::TalonFX       m_turretMotor  {ConstantsCanIds::turretMotorId};
        ctre::phoenix6::hardware::TalonFX       m_flywheelMotor{ConstantsCanIds::flywheelMotorId};
        frc::Servo                              m_hoodActuator {ConstantsPwmPorts::actuatorPort};
};
