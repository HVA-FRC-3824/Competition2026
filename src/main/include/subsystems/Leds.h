#pragma once

#include <array>
#include <optional>

#include <frc/RobotController.h>
#include <frc2/command/SubsystemBase.h>

#include <frc/AddressableLED.h>
#include <frc/LEDPattern.h>
#include <frc/LEDPattern.h>

#include "subsystems/Tower.h"

#include "Constants.h"
#include "ConstantsRoboRio.h"

#pragma region LedConstants
namespace LedConstants
{
    constexpr auto Length        = 60U;     // The length of the LED string
    constexpr auto Brightness    = 0.5;

    constexpr auto Red           = 255;
    constexpr auto Green         = 255;
    constexpr auto Blue          = 255;

    constexpr auto StrobeDelay   = 250_ms;  // The delay between strobe flashes
    constexpr auto HvaDelay      =   1_Hz;  // The cycle speed for HVA colors
    constexpr auto ShootingSpeed = 1.5_Hz;   // The cycle speed for shooting
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
    Rainbow,
    MatchMode
};

class Leds : public frc2::SubsystemBase
{
    public:

        explicit Leds();

        void     SetRobotState(TowerMode shootingMode, bool isClimbing, bool isShooting, bool isOnTarget);

        void     SetMode(LedMode ledMode);

        LedMode  GetMode() const { return m_ledMode; }
        
        void     Periodic() override;

    private:

        void SetLeds(frc::LEDPattern turret, std::optional<frc::LEDPattern> underglow = std::nullopt);

        LedMode             m_ledMode       = LedMode::HvaColors;  // The LED mode

        int                 m_firstPixelHue = 0;                   // Store the hue of the first pixel for rainbow mode
        int                 m_cycleCounter  = 0;                   // Counter for dynamic LED modes

        // Create an LED pattern that will display a rainbow across all hues at maximum saturation and half brightness and
        // that scrolls the rainbow pattern across the LED strip, moving at a speed of 1 meter per second.
        frc::LEDPattern     m_scrollingRainbow  = frc::LEDPattern::Rainbow(255, 128).ScrollAtAbsoluteSpeed(0.1_mps, units::meter_t{1 / 120.0});


        frc::LEDPattern     m_hvaColors         = frc::LEDPattern::Steps({std::pair{0.0, frc::Color::kWhite}, std::pair{0.5, frc::Color::kBlue}}).
                                                                    ScrollAtRelativeSpeed(LedConstants::HvaDelay);  

        frc::LEDPattern     m_strobe            = frc::LEDPattern::Solid(frc::Color::kWhite).Blink(LedConstants::StrobeDelay);  

        frc::AddressableLED m_ledTurret   {ConstantsPwmPorts::LedTurretPort};
        // frc::AddressableLED m_ledUnderGlow{ConstantsPwmPorts::LedUnderGlowPort};

        std::array<frc::AddressableLED::LEDData, LedConstants::Length> m_ledTurretBuffer   {};
        std::array<frc::AddressableLED::LEDData, LedConstants::Length> m_ledUnderGlowBuffer{};

        frc::LEDPattern underGlowPattern = frc::LEDPattern::Solid(frc::Color::kBlack);
        frc::LEDPattern turretPattern   = frc::LEDPattern::Solid(frc::Color::kBlack);

        TowerMode m_towerState = TowerMode::Idle;
        bool      m_isOnTarget = false;
        bool      m_isClimbing = false;
        bool      m_isShooting = false;
        
        TowerMode lastTowerState = m_towerState;
        bool      lastIsShooting = m_isShooting;
        bool      lastIsClimbing = m_isClimbing;
        bool      lastIsOnTarget = m_isOnTarget;
         
};
