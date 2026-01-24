#pragma once

#pragma region Includes
#include <hal/FRCUsageReporting.h>

#include <frc/TimedRobot.h>
#include <frc/DriverStation.h>

#include "lib/Logging.h"
#include "RobotContainer.h"

#include "Constants.h"
#pragma endregion

class Robot : public frc::TimedRobot
{
    public:

        void RobotInit()          override;
        void RobotPeriodic()      override;
        void AutonomousInit()     override;
        void AutonomousPeriodic() override;
        void TeleopInit()         override;
        void TeleopPeriodic()     override;
        void DisabledInit()       override;
        void DisabledPeriodic()   override;
        void TestInit()           override;
        void TestPeriodic()       override;
        void SimulationInit()     override;
        void SimulationPeriodic() override;

    private:

        // Pointer to the autonomous command
        frc2::Command  *m_autonomousCommand = nullptr;

        // A pointer to the robot container class
        RobotContainer *m_robotContainer    = nullptr;
};
