#pragma once

#pragma region Includes
#include <functional>
#include <numeric>

#include <units/angle.h>
#include <units/length.h>

#include <frc2/command/SubsystemBase.h>
#include <frc/DriverStation.h>

#include <frc/geometry/Pose2d.h>

#include <ctre/phoenix6/TalonFX.hpp>
#include <frc/Servo.h>

#include <photon/PhotonCamera.h>

#include "Constants.h"
#include "ConstantsCanIds.h"
#pragma endregion

#pragma region StateStructures
enum TowerMode
{
    HUB,
    PASSING,
    STATIC,
    MANUAL
};

struct TowerState
{
    TowerMode       mode;
    units::degree_t turretAngle;
    double          flyWheelSpeed;
    double          hoodActuatorPercentInput;
};
#pragma endregion

class Tower : public frc2::SubsystemBase
{
    public:
        
        explicit    Tower(std::function<frc::Pose2d()> poseSupplier);

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
        
        // Sets the desired angle of the turret relative to the field
        void SetTurret(units::degree_t angle, units::degree_t gyroAngle);

        TowerState CalculateShot(units::meter_t distance, frc::Translation2d speed);

        ctre::phoenix6::hardware::TalonFX m_turretMotor  {ConstantsCanIds::turretMotorID};
        ctre::phoenix6::hardware::TalonFX m_flywheelMotor{ConstantsCanIds::flywheelMotorID};
        frc::Servo                        m_hoodActuator {ConstantsCanIds::actuatorID};

        // Do not use this to do pose estimatation. Because its on a turret, it is unreliable
        photon::PhotonCamera     m_turretCam{"turretCam"};

        std::function<frc::Pose2d()>            m_poseSupplier;
        std::pair<frc::Pose2d, units::second_t> m_pose;

        TowerState m_state{TowerMode::STATIC, 0_deg, 0, 0};

        
};