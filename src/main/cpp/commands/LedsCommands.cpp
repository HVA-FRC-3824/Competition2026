#include "commands/LedsCommands.h"

/// @brief Command to set robot status for status LEDs
/// @param leds The LED subsystem
/// @param ledMode The status to set the leds to
/// @return Command that sets the LEDs to the desired state
frc2::CommandPtr SetLedStatus(Leds* leds, LedMode *ledMode)
{
    // Create and return a RunCommand that sets the LED mode
    return frc2::RunCommand{[=]() { leds->SetMode(*ledMode); }, {leds}}.ToPtr();
}
