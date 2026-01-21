#pragma once

#pragma region Includes
#include <frc2/command/button/JoystickButton.h>
#include <frc2/command/InstantCommand.h>
#include <frc2/command/RunCommand.h>

#include <frc/Joystick.h>
#include <frc/XboxController.h>

#include <frc/filter/SlewRateLimiter.h>

#include <photon/PhotonCamera.h>

#include "subsystems/Leds.h"

#include "subsystems/Chassis.h"
#include "subsystems/Indexer.h"
#include "subsystems/Tower.h"
#include "subsystems/Climb.h"
#include "subsystems/Intake.h"

#include "commands/ChassisCommands.h"
#include "commands/IndexerCommands.h"
#include "commands/LedsCommands.h"
#include "commands/IntakeCommands.h"
#include "commands/TowerCommands.h"
#include "commands/IntakeCommands.h"
#include "commands/ClimbCommands.h"

#include "Constants.h"
#pragma endregion

/// @brief Class to instantiate the robot subsystems and commands along with the operator controls
class RobotContainer
{
    public:

        // Method that returns a pointer to the singleton instance of the RobotContainer class
        static RobotContainer *GetInstance();

        

    private:

        // Static pointer to singleton instance
        static RobotContainer              *m_robotContainer;

        // Private class constructor to configure the robot and SmartDashboard configuration
        RobotContainer();

        std::function<frc::ChassisSpeeds()> GetChassisSpeeds();

        double                              GetExponentialValue(double joystickValue, double exponent);

        frc::XboxController                 m_driveController{constants::controller::DrivePort};

        // Instantiate the robot subsystems
        Chassis                             m_chassis{};
        Indexer                             m_indexer{};
        Leds                                m_leds{};
        Tower                               m_tower{[&]() {return m_chassis.GetPose();}};
        Climb                               m_climb{};
        Intake                              m_intake{};

        // Instantiate subsystem states
        TowerState                          m_manualTowerState{TowerMode::MANUAL, 0_deg, 0.0, 0.0};

        LedMode                             m_robotStatus = LedMode::Off;
};
