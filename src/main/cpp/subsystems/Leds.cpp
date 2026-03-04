#include "subsystems/Leds.h"

using namespace std;

#pragma region Leds (constructor)
/// @brief Class to support an addressable LED string.
Leds::Leds(std::function<std::pair<TowerMode, bool>()> towerStateIsOnTargetSupplier, std::function<bool()> isClimbingSupplier, std::function<bool()> isShootingSupplier) : 
    m_towerStateIsOnTargetSupplier(towerStateIsOnTargetSupplier), 
    m_isClimbingSupplier(isClimbingSupplier), 
    m_isShootingSupplier(isShootingSupplier)
{
    // Length is expensive to set, so only set it once, then just update data
    m_ledTurret.SetLength(LedConstants::Length);
    m_ledUnderGlow.SetLength(LedConstants::Length);

    // Set the default mode
    SetMode(LedMode::Off);

    // Intialize the LED data
    m_ledTurret.SetData(m_ledTurretBuffer);
    m_ledUnderGlow.SetData(m_ledUnderGlowBuffer);

    // Start the addressable LED communications
    m_ledTurret.Start();
    m_ledUnderGlow.Start();
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
void Leds::SetLeds(frc::LEDPattern turret, std::optional<frc::LEDPattern> underglow)
{
    std::array<frc::AddressableLED::LEDData, 15> turretBuffer;
    turret.ApplyTo(turretBuffer);
    
    std::array<frc::AddressableLED::LEDData, 15> underglowBuffer;
    underglow.value_or(turret).ApplyTo(underglowBuffer);
    
    m_ledTurret.SetData(turretBuffer);
    m_ledUnderGlow.SetData(underglowBuffer);
}
// {
//     std::array<frc::AddressableLED::LEDData, 15> outsideBuffer;
//     outside.ApplyTo(outsideBuffer);
    
//     std::array<frc::AddressableLED::LEDData, 15> insideBuffer;
//     inside.value_or(outside).ApplyTo(insideBuffer);
    
//     // Normal sections
//     std::copy(outsideBuffer.begin(), outsideBuffer.end(), 
//               m_ledBuffer.begin());      // left up
//     std::copy(insideBuffer.begin(), insideBuffer.end(), 
//               m_ledBuffer.begin() + 15); // middle first
    
//     // Mirrored sections
//     std::reverse_copy(insideBuffer.begin(), insideBuffer.end(), 
//                       m_ledBuffer.begin() + 30); // middle second (mirrored)
//     std::reverse_copy(outsideBuffer.begin(), outsideBuffer.end(), 
//                       m_ledBuffer.begin() + 45); // right down (mirrored)
    
//     m_led.SetData(m_ledBuffer);
// }
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
            SetLeds(m_hvaColors);
            break;
        }

        case LedMode::Strobe:
        {
            SetLeds(m_strobe);
            break;
        }

        case LedMode::Rainbow:
        {
            // Run the rainbow pattern and apply it to the buffer
            SetLeds(m_scrollingRainbow);
            break;
        }
        
        case LedMode::MatchMode:
        {
            static frc::LEDPattern sidesPattern = m_shootingModeIsClimbing(m_towerStateIsOnTargetSupplier().first, m_isClimbingSupplier());
            static frc::LEDPattern topPattern   = m_isShootingIsOnTarget  (m_isShootingSupplier(),                 m_towerStateIsOnTargetSupplier().second);

            static auto lastTowerState = m_towerStateIsOnTargetSupplier();
            static auto lastIsShooting = m_isShootingSupplier();
            
            if (lastTowerState != m_towerStateIsOnTargetSupplier())
            {
                lastTowerState = m_towerStateIsOnTargetSupplier();
                sidesPattern = m_shootingModeIsClimbing(lastTowerState.first, m_isClimbingSupplier());
                topPattern   = m_isShootingIsOnTarget  (m_isShootingSupplier(), lastTowerState.second);
            }
            else if (lastIsShooting != m_isShootingSupplier())
            {
                lastIsShooting = m_isShootingSupplier();
                topPattern   = m_isShootingIsOnTarget  (lastIsShooting, lastTowerState.second);
            }

            // Set the pattern based on the shooting mode and climbing status
            SetLeds(sidesPattern, 
                    topPattern);
            break;
        }
    }
}
#pragma endregion
