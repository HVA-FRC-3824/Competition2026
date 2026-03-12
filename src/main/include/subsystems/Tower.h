#pragma once

#pragma region Includes
#include <functional>
#include <numeric>

#include <units/angle.h>
#include <units/length.h>

#include <frc2/command/SubsystemBase.h>
#include <frc/DriverStation.h>

#include <frc/smartdashboard/Mechanism2d.h>
#include <frc/smartdashboard/MechanismLigament2d.h>
#include <frc/smartdashboard/SmartDashboard.h>

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
    ManualControl,
    Automatic
};

struct TowerState
{
    TowerMode                 mode;
    units::degree_t           turretAngle;
    units::turns_per_second_t flywheelSpeed;
    double                    hoodActuatorDistance;
};
#pragma endregion

#pragma region TowerConstants
namespace TowerConstants
{
    // Default is facing forwards, 10 degree deadzone straight right
    // 1 turret rotation - 24 motor rotations - 360 degrees, so 1 motor rotation is 15 degrees
    // Min: -4, max: 14.5 (rotations)
    constexpr auto MinAngle           =  15_deg * -4; 
    constexpr auto MaxAngle           =  15_deg * 14.45;
 
    constexpr auto MaxLength          = 14.336_in;  // TODO: test these lengths, they're most likely accurate
    constexpr auto MinLength          =  8.946_in;  // I got these from team 102 from 2022, they used the same actuator

    constexpr auto ActuatorLowerBound = -0.95;      // Comes from 102 too
    constexpr auto ActuatorUpperBound =  0.95;

    constexpr auto TargetTolerance    = 0.10;       // Percent of the target

    constexpr auto HoodA              = 0.0;
    constexpr auto HoodB              = 0.0;
    constexpr auto HoodC              = 0.0;
    
    constexpr auto FlywheelA          = 0.0;
    constexpr auto FlywheelB          = 13.571;
    constexpr auto FlywheelC          = 50;  
    
    constexpr auto TurretGearReduction              = 24.0; // There's a gearbox, and then a pulley, and then a small gear and a big gear, too many numbers for programming

    constexpr auto OffsetTurretFromRobotCenter      = frc::Transform3d{frc::Translation3d{-6.0_in, 0.0_m, 20.0_in}, frc::Rotation3d{0.0_deg, 0.0_deg, 0.0_deg}};    
}
#pragma endregion

class Tower : public frc2::SubsystemBase
{
    public:
        
        explicit         Tower(std::function<frc::Pose2d()> chassisPoseSupplier, std::function<frc::ChassisSpeeds()> chassisSpeedsSupplier);

        void             SetState(TowerState newState);
        TowerState       GetState();
 
        bool             IsOnTarget();

        void             AimUsingTurretCamera(bool usingTurretCamera) { m_usingTurretCamera = usingTurretCamera; }

        void             TestActuator(double position) { m_hoodActuator.SetSpeed(position); }

        void             Periodic() override;

    private: 
    
        void             SetFlywheel(units::turns_per_second_t input);
        void             SetActuator(double position);

        void             SetTurretAngle(units::degree_t angle);
        units::degree_t  GetTurretAngle();

        TowerState       CalculateShot(TowerMode towerMode, frc::Translation2d relativeDistance, frc::ChassisSpeeds chassisSpeed, frc::Rotation2d chassisRotation);
        double           CalculatePolynomial(units::inch_t distance, double a, double b, double c);

        bool                                m_isBlue = frc::DriverStation::GetAlliance().value_or(frc::DriverStation::Alliance::kBlue) == frc::DriverStation::Alliance::kBlue;

        bool                                m_usingTurretCamera = false;

        photon::PhotonCamera                m_turretCamera{"CameraTurret"};

        std::function<frc::Pose2d()>        m_chassisPoseSupplier;
        std::function<frc::ChassisSpeeds()> m_chassisSpeedsSupplier;   

        TowerState                          m_state{TowerMode::Idle, 0_deg, 0_rpm, 0.0}; 

        ctre::phoenix6::hardware::TalonFX   m_turretMotor          {ConstantsCanIds::TurretMotorId};
        ctre::phoenix6::hardware::TalonFX   m_flywheelMotor        {ConstantsCanIds::FlywheelMotorId};
        ctre::phoenix6::hardware::TalonFX   m_flywheelFollowerMotor{ConstantsCanIds::FlywheelFollowerMotorId};
        
        frc::Servo                          m_hoodActuator{ConstantsPwmPorts::ActuatorPort};
};
