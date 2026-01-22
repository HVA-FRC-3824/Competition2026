#pragma once

#pragma region Includes
#include <functional>
#include <numeric>

#include <units/angle.h>
#include <units/length.h>

#include <frc2/command/SubsystemBase.h>
#include <frc/DriverStation.h>

#include <frc/geometry/Pose2d.h>

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
    Hub,
    Passing,
    Static,
    Manual
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
    constexpr auto AngleMaximumAmperage       = 20.0_A;

    constexpr auto AngleP                     = 0.1;
    constexpr auto AngleI                     = 0.0;
    constexpr auto AngleD                     = 0.0;

    constexpr auto FlywheelMaximumAmperage    = 40.0_A;

    constexpr auto FlywheelP                  = 0.1;
    constexpr auto FlywheelI                  = 0.0;
    constexpr auto FlywheelD                  = 0.0;

    constexpr auto MotorConfigurationAttempts = 5;

    // This is -1 to 1, really 0 to 1
    constexpr double constantFlywheelSpeed    = 0.8;

    // TODO: test these angles, likely isn't correct as we have a different than the bot, team 102 in 2022, I got it from
    constexpr units::degree_t MinAngle        = 0_deg;
    constexpr units::degree_t MaxAngle        = 50_deg;

    // TODO: test these lengths, they're most likely accurate
    // I got these from team 102 from 2022, they used the same actuator
    constexpr units::inch_t   MaxLength        = 14.336_in;
    constexpr units::inch_t   MinLength        = 8.946_in;

    // Comes from 102 too
    constexpr double          ActuatorLowerBound = -0.95;
    constexpr double          ActuatorUpperBound =  0.95;
}
#pragma endregion

class Tower : public frc2::SubsystemBase
{
    public:
        
        explicit    Tower(std::function<frc::Pose2d()> poseSupplier);

        void        SetState(TowerState newState);
        TowerState  GetState();

        void        Periodic() override;

    private:

        // Motor configuration methods
        void ConfigureTurretMotor();
        void ConfigureFlywheelMotor();

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

        // Do not use this to do pose estimatation. Because its on a turret, it is unreliable
        photon::PhotonCamera                    m_turretCam{"turretCam"};

        std::function<frc::Pose2d()>            m_poseSupplier;
        std::pair<frc::Pose2d, units::second_t> m_pose;

        TowerState                              m_state{TowerMode::Static, 0_deg, 0, 0}; 

        ctre::phoenix6::hardware::TalonFX       m_turretMotor  {ConstantsCanIds::turretMotorID};
        ctre::phoenix6::hardware::TalonFX       m_flywheelMotor{ConstantsCanIds::flywheelMotorID};
        frc::Servo                              m_hoodActuator {ConstantsPwmPorts::actuatorPort};
};
