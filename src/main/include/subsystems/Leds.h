#pragma once

#include <array>
#include <optional>

#include <frc/RobotController.h>
#include <frc2/command/SubsystemBase.h>

#include <frc/AddressableLED.h>
#include <frc/LEDPattern.h>
#include <frc/LEDPattern.h>

#include "Constants.h"
#include "ConstantsRoboRio.h"

#pragma region LedConstants
namespace LedConstants
{
    constexpr auto Length      = 60U;  // The length of the LED string
    constexpr auto Brightness  = 0.5;

    constexpr auto Red         = 255;
    constexpr auto Green       = 255;
    constexpr auto Blue        = 255;

    constexpr auto StrobeDelay =  250_ms;  // The delay between strobe flashes
    constexpr auto HvaDelay    =    1_Hz;  // The cycle speed for HVA colors
    constexpr auto ShootingSpeed = 2_Hz;   // The cycle speed for shooting
}
#pragma endregion

/// @brief modes for the LED string.
enum LedMode
{
    Off,
    SolidGreen,
    SolidRed,
    HvaColors,
    Strobe,
    ShootingOnTarget,
    ShootingOffTarget,
    Rainbow
};

class Leds : public frc2::SubsystemBase
{
    public:

        explicit Leds();

        void     SetMode(LedMode ledMode);

        LedMode  GetMode() const { return m_ledMode; }
        
        void     Periodic() override;

    private:

        void SetLeds(frc::LEDPattern pattern, std::optional<frc::LEDPattern> inside = std::nullopt);

        void SolidColor(int red, int green, int blue);
        void HvaColors();
        void Strobe();
        void ShootingAnimation();

        LedMode             m_ledMode = LedMode::HvaColors;            // The LED mode

        int                 m_firstPixelHue = 0;  // Store the hue of the first pixel for rainbow mode
        int                 m_cycleCounter  = 0;  // Counter for dynamic LED modes

        // Create an LED pattern that will display a rainbow across all hues at maximum saturation and half brightness and
        // that scrolls the rainbow pattern across the LED strip, moving at a speed of 1 meter per second.
        frc::LEDPattern     m_scrollingRainbow = frc::LEDPattern::Rainbow(255, 128).ScrollAtAbsoluteSpeed(0.1_mps, units::meter_t{1 / 120.0});

        // Create an LED pattern that displays a red-to-blue gradient, then scroll at one quarter of the LED strip's length per second.
        // For a half-meter length of a 120 LED-per-meter strip, this is equivalent to scrolling at 12.5 centimeters per second.
        frc::LEDPattern     m_shooting = frc::LEDPattern::Gradient(frc::LEDPattern::kDiscontinuous, std::array<frc::Color, 2>{frc::Color::kBlack, frc::Color::kRed}).
                                                          ScrollAtRelativeSpeed(LedConstants::ShootingSpeed);

        frc::LEDPattern     m_shootingOnTarget = frc::LEDPattern::Solid(frc::Color::kGreen);
  
        frc::LEDPattern     m_shootingOffTarget = frc::LEDPattern::Solid(frc::Color::kYellow);

        frc::LEDPattern     m_hvaColors = frc::LEDPattern::Steps({std::pair{0.0, frc::Color::kWhite}, std::pair{0.5, frc::Color::kBlue}}).
                                                          ScrollAtRelativeSpeed(LedConstants::HvaDelay);  
                                                          
        frc::LEDPattern     m_strobe = frc::LEDPattern::Solid(frc::Color::kWhite).Blink(LedConstants::StrobeDelay);  

        frc::AddressableLED m_led{ConstantsPwmPorts::LedPort};

        std::array<frc::AddressableLED::LEDData, LedConstants::Length> m_ledBuffer;  // Instatntiate the LED data buffer
};
