#pragma once

#include <functional>

#include <frc2/command/CommandPtr.h>
#include <frc2/command/InstantCommand.h>

#include "subsystems/Leds.h"

#pragma region SetLeds
frc2::CommandPtr SetLeds(Leds* leds);
#pragma endregion