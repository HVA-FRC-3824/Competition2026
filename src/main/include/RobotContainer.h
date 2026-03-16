#pragma once

#pragma region Includes
#include <pathplanner/lib/auto/NamedCommands.h>
#include <pathplanner/lib/commands/PathPlannerAuto.h>
#include <pathplanner/lib/auto/AutoBuilder.h>

#include <frc2/command/button/JoystickButton.h>
#include <frc2/command/button/POVButton.h>
#include <frc2/command/InstantCommand.h>
#include <frc2/command/RunCommand.h>

#include <frc/Joystick.h>
#include <frc/XboxController.h>

#include <frc/filter/SlewRateLimiter.h>

#include "subsystems/Leds.h"

#include "subsystems/Chassis.h"
#include "subsystems/Spindexer.h"
#include "subsystems/Tower.h"
#include "subsystems/Climb.h"
#include "subsystems/Intake.h"

#include "commands/ChassisCommands.h"
#include "commands/IntakeCommands.h"
#include "commands/LedsCommands.h"
#include "commands/SpindexerCommands.h"
#include "commands/TowerCommands.h"
#include "commands/ClimbCommands.h"
#include "commands/complex/Shooting.h"

#include "Constants.h"
#pragma endregion

/// @brief Class to instantiate the robot subsystems and commands along with the operator controls
class RobotContainer
{
    public:

        // Method that returns a pointer to the singleton instance of the RobotContainer class
        static RobotContainer *GetInstance();

        void                   ResetWheelAnglesToZero();  // Method to reset swerve wheel angles to zero

        void                   ResetGyroAngle();

        frc2::Command*         GetAutonomousCommand() { return m_autoChooser.GetSelected(); }; 

        void                   SetUpChassis() { m_chassis.ResetGyroAngle(); m_chassis.ToggleFieldCentric(); }

    private:

        // Private class constructor to configure the robot and SmartDashboard configuration
        RobotContainer();

        void                                  InitializePathPlanner();
        void                                  InitializeDriverControls();
        void                                  InitializeOperatorControls();
 
        std::function<frc::ChassisSpeeds()>   GetSpeeds();

        double                                GetExponentialValue(double joystickValue, double exponent);

        // Static pointer to singleton instance
        static RobotContainer                *m_robotContainer;

        frc::SendableChooser<frc2::Command *> m_autoChooser;

        frc::XboxController                   m_driveController   {ConstantsUsbPort::DrivePort};
        frc::XboxController                   m_operatorController{ConstantsUsbPort::OperatorPort};

        // Instantiate the robot subsystems
        Chassis   m_chassis{};
        Spindexer m_spindexer{};
        Leds      m_leds{};
        Tower     m_tower{[&] {return m_chassis.GetPose();}, [&] {return m_chassis.GetSpeeds();}};
        Climb     m_climb{};
        Intake    m_intake{};

        // Instantiate subsystem states
        TowerState m_manualTowerState{TowerMode::ManualControl, 0_deg, 50_tps, 0.0};
        LedMode    m_ledMode{LedMode::MatchMode};
};