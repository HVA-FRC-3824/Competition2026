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
    ManualControl
};

struct TowerState
{
    TowerMode                 mode;
    units::degree_t           turretAngle;
    units::turns_per_second_t flywheelSpeed;
    units::inch_t             hoodActuatorInches;
};
#pragma endregion

#pragma region TowerConstants
namespace TowerConstants
{
    constexpr auto MinAngle           = -180_deg;  // TODO: Make these real. 
    constexpr auto MaxAngle           =  180_deg;
 
    constexpr auto MaxLength          = 14.336_in;  // TODO: test these lengths, they're most likely accurate
    constexpr auto MinLength          =  8.946_in;  // I got these from team 102 from 2022, they used the same actuator

    constexpr auto ActuatorLowerBound = -0.95;      // Comes from 102 too
    constexpr auto ActuatorUpperBound =  0.95;

    constexpr auto FlywheelTolerance = 100_rpm;

    // From inches to 0-1 range
    constexpr auto ActuatorDistanceConversionFactor = (TowerConstants::MaxLength - TowerConstants::MinLength);

    constexpr auto OffsetTurretFromRobotCenter = frc::Transform3d{frc::Translation3d{0.0_m, 0.0_m, 0.0_m}, frc::Rotation3d{0.0_deg, 0.0_deg, 0.0_deg}};
}
#pragma endregion

class Tower : public frc2::SubsystemBase
{
    public:
        
        explicit         Tower(std::function<frc::Pose2d()> chassisPoseSupplier, std::function<frc::ChassisSpeeds()> chassisSpeedsSupplier);

        void             SetState(TowerState newState);
        TowerState       GetState();

        void             TestActuator(double position) { m_hoodActuator.SetSpeed(position); }

<<<<<<< Updated upstream
        void             AimUsingTurretCamera(bool usingTurretCamera) { m_usingTurretCamera = usingTurretCamera; }
=======
        bool        IsSpunUp();

        void        TestActuator(double position) { m_hoodActuator.SetSpeed(position); }
>>>>>>> Stashed changes

        void             Periodic() override;

    private: 
    
        void            SetActuator(units::inch_t position);
        void            SetFlywheel(units::turns_per_second_t input);
    
        void            SetTurretAngle(units::degree_t angle);
        units::degree_t GetTurretAngle();

        TowerState      CalculateShot(TowerMode towerMode, frc::Translation2d relativeDistance, frc::ChassisSpeeds chassisSpeed);

        bool                                m_isBlue = frc::DriverStation::GetAlliance().value_or(frc::DriverStation::Alliance::kBlue) 
                                                            == frc::DriverStation::Alliance::kBlue;

        bool                                m_usingTurretCamera = true;

        frc::Mechanism2d                    m_logMechanism{20, 20, frc::Color{0.0, 0.0, 0.0}}; // Width height
        frc::MechanismRoot2d               *m_logMechanismRoot = m_logMechanism.GetRoot("Tower", 10, 10);
        
        frc::MechanismLigament2d           *m_logHoodFlywheel = m_logMechanismRoot->Append<frc::MechanismLigament2d>("Hood&Flywheel", 3, 0_deg);
        frc::MechanismLigament2d           *m_logTurret       = m_logMechanismRoot->Append<frc::MechanismLigament2d>("Turret",        3, 90_deg);

        photon::PhotonCamera                m_turretCamera{"CameraTurret"};

        std::function<frc::Pose2d()>        m_chassisPoseSupplier;
        std::function<frc::ChassisSpeeds()> m_chassisSpeedsSupplier;   

        TowerState                          m_state{TowerMode::ManualControl, 0_deg, 0_rpm, 0_in}; 

        ctre::phoenix6::hardware::TalonFX   m_turretMotor  {ConstantsCanIds::turretMotorId};
        ctre::phoenix6::hardware::TalonFX   m_flywheelMotor{ConstantsCanIds::flywheelMotorId};
        frc::Servo                          m_hoodActuator {ConstantsPwmPorts::actuatorPort};
};
