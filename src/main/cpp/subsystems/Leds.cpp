#include "subsystems/Leds.h"

using namespace std;

#pragma region Leds (constructor)
/// @brief Class to support an addressable LED string.
Leds::Leds()
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

void Leds::SetRobotState(TowerMode shootingMode, bool isClimbing, bool isShooting, bool isOnTarget)
{
    m_towerState = shootingMode;
    m_isOnTarget = isClimbing;
    m_isClimbing = isShooting;
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
void Leds::SetLeds(frc::LEDPattern turret, std::optional<frc::LEDPattern> underglow)
{
    turret.ApplyTo(m_ledTurretBuffer);
    underglow.value_or(turret).ApplyTo(m_ledUnderGlowBuffer);
    
    m_ledTurret.SetData(m_ledTurretBuffer);
    m_ledUnderGlow.SetData(m_ledUnderGlowBuffer);
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
            static frc::LEDPattern underGlowPattern = frc::LEDPattern::Solid(frc::Color::kBlack);
            static frc::LEDPattern turretPattern   = frc::LEDPattern::Solid(frc::Color::kBlack);

            static auto lastTowerState = m_towerState;
            static auto lastIsShooting = m_isShooting;
            static auto lastIsClimbing = m_isClimbing;
            static auto lastIsOnTarget = m_isOnTarget;

            if (lastTowerState != m_towerState || lastIsClimbing != m_isClimbing)
            {
                lastTowerState = m_towerState;
                lastIsClimbing = m_isClimbing;

                switch (m_towerState)
                {
                    case TowerMode::Idle:
                        underGlowPattern = frc::LEDPattern::Solid(frc::Color::kCyan);
                        break;
                    case TowerMode::ManualControl:
                        underGlowPattern = frc::LEDPattern::Solid(frc::Color::kYellow);
                        break;
                    case TowerMode::ShootingToHub:
                        underGlowPattern = frc::LEDPattern::Solid(frc::Color::kGreen);
                        break;
                    case TowerMode::PassingToAdjacentZone:
                        underGlowPattern = frc::LEDPattern::Solid(frc::Color::kPink);
                        break;
                    default:
                        underGlowPattern = m_scrollingRainbow;
                        break;
                }
                if (m_isClimbing)
                {
                    underGlowPattern = underGlowPattern.Blink(500_ms);
                }
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
            SetLeds(underGlowPattern, 
                    turretPattern);
            break;
        }
    }
}
#pragma endregion
