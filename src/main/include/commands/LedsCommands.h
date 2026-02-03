#pragma once

#include <functional>

#include <frc2/command/CommandPtr.h>
#include <frc2/command/RunCommand.h>

#include "subsystems/Leds.h"

frc2::CommandPtr SetLedStatus(Leds* leds, LedMode *robotStatus);