#include "commands/LedsCommands.h"

frc2::CommandPtr SetRobotStatus(Leds* leds, std::function<RobotStatus()> robotStatusSupplier)
{
    return frc2::InstantCommand{[&]() { leds->SetStatus(robotStatusSupplier()); }, {leds}}.ToPtr();
}