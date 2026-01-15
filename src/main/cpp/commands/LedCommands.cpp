#include "commands/LedsCommands.h"

frc2::CommandPtr SetRobotStatus(Leds* leds, RobotStatus robotStatus)
{
    return frc2::InstantCommand{[&]() { leds->SetStatus(robotStatus); }, {leds}}.ToPtr();
}