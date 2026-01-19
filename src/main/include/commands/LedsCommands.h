#pragma once

#include <functional>

#include <frc2/command/CommandPtr.h>
#include <frc2/command/InstantCommand.h>

#include "lib/Leds.h"

/// @brief Command to set robot status for status LEDs
/// @param leds The LED subsystem
/// @param robotStatus The status to set the leds to
/// @return Command that sets the LEDs to the desired state
frc2::CommandPtr SetRobotStatus(Leds* leds, std::function<RobotStatus()> robotStatus);