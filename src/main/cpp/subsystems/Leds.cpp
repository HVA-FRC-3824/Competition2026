#include "subsystems/Leds.h"

using namespace std;

#pragma region Leds (constructor)
/// @brief Class to support an addressable LED string.
Leds::Leds()
{
    // Length is expensive to set, so only set it once, then just update data
    m_led.SetLength(LedConstants::Length);

    // Set the default mode
    SetMode(LedMode::Off);

    // Intialize the LED data
    m_led.SetData(m_ledBuffer);

    // Start the addressable LED communications
    m_led.Start();
}
#pragma endregion

#pragma region SetMode
/// @brief Setting the Led's mode to the given parameter.
/// @param ledMode mode to set the Leds.
void Leds::SetMode(LedMode ledMode)
{
    // Remember the LED mode
    m_ledMode = ledMode;
}
#pragma endregion

#pragma region SetLeds
void Leds::SetLeds(frc::LEDPattern pattern)
{
    std::array<frc::AddressableLED::LEDData, 15> sectionBuffer;
    pattern.ApplyTo(sectionBuffer);
    
    // Normal sections
    std::copy(sectionBuffer.begin(), sectionBuffer.end(), 
              m_ledBuffer.begin());      // left up
    std::copy(sectionBuffer.begin(), sectionBuffer.end(), 
              m_ledBuffer.begin() + 15); // middle first
    
    // Mirrored sections
    std::reverse_copy(sectionBuffer.begin(), sectionBuffer.end(), 
                      m_ledBuffer.begin() + 30); // middle second (mirrored)
    std::reverse_copy(sectionBuffer.begin(), sectionBuffer.end(), 
                      m_ledBuffer.begin() + 45); // right down (mirrored)
    
    m_led.SetData(m_ledBuffer);
}
#pragma endregion

#pragma region Periodic
/// @brief This method will be called once periodically.
void Leds::Periodic()
{
    switch (m_ledMode)
    {
        case LedMode::Off:
        {
            SetLeds(frc::LEDPattern::Solid(frc::Color::kBlack));
            break;
        }
        case LedMode::SolidGreen:
        {
            SetLeds(frc::LEDPattern::Solid(frc::Color::kGreen));
            break;
        }
        case LedMode::SolidRed:
        {
            SetLeds(frc::LEDPattern::Solid(frc::Color::kRed));
            break;
        }

        case LedMode::HvaColors:
        {
            SetLeds(frc::LEDPattern::Steps({std::pair{0.0, frc::Color::kWhite}, std::pair{0.5, frc::Color::kBlue}}).ScrollAtRelativeSpeed(LedConstants::HvaDelay));
            break;
        }

        case LedMode::Strobe:
        {
            SetLeds(frc::LEDPattern::Solid(frc::Color::kWhite).Blink(LedConstants::StrobeDelay));
            break;
        }
        
        case LedMode::ShootingAnimation:
        {
            SetLeds(m_shooting);
            break;
        }

        case LedMode::Rainbow:
        {
            // Run the rainbow pattern and apply it to the buffer
            SetLeds(m_scrollingRainbow);
            break;
        }
    }
}
#pragma endregion
