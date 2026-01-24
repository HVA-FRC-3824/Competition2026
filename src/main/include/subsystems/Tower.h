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

#include "Constants.h"
#include "ConstantsRoboRio.h"
#pragma endregion

#pragma region StateStructures
enum TowerMode
{
    ShootingToHub,
    PassingToAdjacentZone,
    ManualControl
};

struct TowerState
{
    TowerMode       mode;
    units::degree_t turretAngle;
    double          flywheelSpeed;
    double          hoodActuatorPercentInput;
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

    // Whether or not to use the turret camera for aiming, alternate aiming implementations
    constexpr auto usingTurretCamera = false;
}
#pragma endregion

class Tower : public frc2::SubsystemBase
{
    public:
        
        explicit    Tower(std::function<frc::Pose2d()> poseSupplier, std::function<frc::ChassisSpeeds()> speedsSupplier);

        void        SetState(TowerState newState);
        TowerState  GetState();

        void        Periodic() override;

    private: 

        // https://andymark.com/products/linear-servo-actuators check the manual
        // Input in between 0 and 1
        void SetActuator(double position);

        // Starts to spin up the flywheel motor
        // This may need to be a -1,1 or some other kind of input
        void SetFlywheel(double input);
        
        // Sets the desired angle of the turret relative to the robot
        void SetTurret(units::degree_t angle);

        // Changes the turret angle, flywheel speed, and hood actuator position based on distance and speed to target
        TowerState CalculateShot(frc::Translation2d relativeDistance, frc::ChassisSpeeds speed);

        // Do not use this to do pose estimatation. Because its on a turret, it is unreliable
        photon::PhotonCamera                    m_turretCam{"turretCam"};

        std::function<frc::Pose2d()>            m_poseSupplier;
        std::function<frc::ChassisSpeeds()>     m_speedsSupplier;

        TowerState                              m_state{TowerMode::ManualControl, 0_deg, 0, 0}; 

        ctre::phoenix6::hardware::TalonFX       m_turretMotor  {ConstantsCanIds::turretMotorId};
        ctre::phoenix6::hardware::TalonFX       m_flywheelMotor{ConstantsCanIds::flywheelMotorId};
        frc::Servo                              m_hoodActuator {ConstantsPwmPorts::actuatorPort};
};
