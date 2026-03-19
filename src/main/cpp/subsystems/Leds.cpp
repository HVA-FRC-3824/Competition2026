#include "subsystems/Leds.h"

using namespace std;

#pragma region Leds (constructor)
/// @brief Class to support an addressable LED string.
Leds::Leds()
{
    // Length is expensive to set, so only set it once, then just update data
    m_ledTurret.SetLength(LedConstants::Length);
    // m_ledUnderGlow.SetLength(LedConstants::Length);

    // Set the default mode
    SetMode(LedMode::Off);

    // Intialize the LED data
    m_ledTurret.SetData(m_ledTurretBuffer);
    // m_ledUnderGlow.SetData(m_ledUnderGlowBuffer);

    // Start the addressable LED communications
    m_ledTurret.Start();
    // m_ledUnderGlow.Start();
}
#pragma endregion

void Leds::SetRobotState(TowerMode shootingMode, bool isClimbing, bool isShooting, bool isOnTarget)
{
    m_isOnTarget = isClimbing;
    m_isShooting = isOnTarget;
}

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
void Leds::SetLeds(frc::LEDPattern turret, Blinker::Pattern underglow)
{
    turret.ApplyTo(m_ledTurretBuffer);
    
    m_ledTurret.SetData(m_ledTurretBuffer);

    m_ledUnderGlow.Set(underglow);
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
            SetLeds(frc::LEDPattern::Solid(frc::Color::kBlack), Blinker::Pattern::SolidColorsBlack);
            break;
        }
        case LedMode::SolidGreen:
        {
            SetLeds(frc::LEDPattern::Solid(frc::Color::kGreen), Blinker::Pattern::SolidColorsGreen);
            break;
        }
        case LedMode::SolidRed:
        {
            SetLeds(frc::LEDPattern::Solid(frc::Color::kRed), Blinker::Pattern::SolidColorsRed);
            break;
        }

        case LedMode::HvaColors:
        {
            SetLeds(m_hvaColors, Blinker::Pattern::SolidColorsBlue);
            break;
        }

        case LedMode::Strobe:
        {
            SetLeds(m_strobe, Blinker::Pattern::ColorOneBreathFast);
            break;
        }

        case LedMode::Rainbow:
        {
            // Run the rainbow pattern and apply it to the buffer
            SetLeds(m_scrollingRainbow, Blinker::Pattern::FixedPaletteRainbowRainbowPalette);
            break;
        }
        
        case LedMode::MatchMode:
        {
            if (isActiveShift())
            {
                m_ledUnderGlow.Set(Blinker::Pattern::SolidColorsGreen);
            }
            else
            {
                m_ledUnderGlow.Set(Blinker::Pattern::SolidColorsRed);
            }

            if (lastIsShooting != m_isShooting || lastIsOnTarget != m_isOnTarget)
            {
                lastIsShooting = m_isShooting;
                lastIsOnTarget = m_isOnTarget;

                turretPattern = m_isOnTarget ? frc::LEDPattern::Solid(frc::Color::kGreen) : frc::LEDPattern::Solid(frc::Color::kRed);
                if (m_isShooting)
                {
                    turretPattern = turretPattern.Blink(250_ms);
                }
            }

            // Set the pattern based on the shooting mode and climbing status
            SetLeds(turretPattern, underGlowPattern);
            break;
        }
    }
}
#pragma endregion
