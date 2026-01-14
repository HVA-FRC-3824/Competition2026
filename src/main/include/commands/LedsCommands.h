#pragma once

#include <functional>

#include <frc2/command/CommandPtr.h>
#include <frc2/command/InstantCommand.h>

#include "subsystems/Leds.h"


frc2::CommandPtr SetLeds(Leds* leds)
{
    return frc2::InstantCommand{[&]() { leds->SetMode(LedMode::Rainbow); }, {leds}}.ToPtr();
}