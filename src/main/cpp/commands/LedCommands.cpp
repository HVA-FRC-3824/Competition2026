#include "commands/LedsCommands.h"


#pragma region SetLeds
frc2::CommandPtr SetLeds(Leds* leds)
{
    return frc2::InstantCommand{[&]() { leds->SetMode(LedMode::Rainbow); }, {leds}}.ToPtr();
}
#pragma endregion