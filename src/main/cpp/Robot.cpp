#include "Robot.h"

#pragma region RobotInit
/// @brief Method called when the robot class is instantiated.
void Robot::RobotInit()
{
    // Enable LiveWindow in test mode
    EnableLiveWindowInTest(true);

    // Report the robot framework usage
    HAL_Report(HALUsageReporting::kResourceType_Framework, HALUsageReporting::kFramework_RobotBuilder);

    // Instantiate after robot init
    // This must be done here so that we can actually use the motors
    m_robotContainer = RobotContainer::GetInstance();
}
#pragma endregion

#pragma region RobotPeriodic
/// @brief Method is called every robot packet, no matter the mode.
void Robot::RobotPeriodic()
{
    // Run the command scheduler
    frc2::CommandScheduler::GetInstance().Run();

    // Check which hub is active
    std::string gameData;
    gameData = frc::DriverStation::GetGameSpecificMessage();

    if(gameData.length() > 0)
    {
        switch (gameData[0])
        {
            case 'B':
            {
                //Blue case code
                break;
            }

            case 'R':
            {
                //Red case code
                break;
            }

            default:
            {
                //This is corrupt data
                break;
            } 
        }
    }
    else 
    {
        //Code for no data received yet
    }
}
#pragma endregion

#ifndef RUNNING_FRC_TESTS
int main()
{
    return frc::StartRobot<Robot>();
}
#endif
