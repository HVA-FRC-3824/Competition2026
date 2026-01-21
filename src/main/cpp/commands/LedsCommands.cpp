#include "commands/LedsCommands.h"

frc2::CommandPtr SetLedStatus(Leds* leds, std::function<LedMode()> robotStatusSupplier)
{
    return frc2::RunCommand{[&]() { leds->SetMode(robotStatusSupplier()); }, {leds}}.ToPtr();
}