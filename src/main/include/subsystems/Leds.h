#pragma once

#include <array>
#include <optional>

#include <frc/RobotController.h>
#include <frc2/command/SubsystemBase.h>

#include <frc/AddressableLED.h>
#include "lib/Blinker.h"

#include <frc/LEDPattern.h>

#include "subsystems/Tower.h"

#include "Constants.h"
#include "ConstantsRoboRio.h"

#define isActiveShift() ([]() -> bool {                                   \
    auto alliance = frc::DriverStation::GetAlliance();                    \
    /* If we have no alliance, we cannot be enabled, therefore no hub. */ \
    if (!alliance) {                                                      \
      return false;                                                       \
    }                                                                     \
    /* Hub is always enabled in autonomous. */                            \
    if (frc::DriverStation::IsAutonomousEnabled()) {                      \
      return true;                                                        \
    }                                                                     \
    /* At this point, if we're not teleop enabled, there is no hub. */    \
    if (!frc::DriverStation::IsTeleopEnabled()) {                         \
      return false;                                                       \
    }                                                                     \
    /* We're teleop enabled, compute. Run 3 seconds ahead */              \
    auto matchTime = frc::DriverStation::GetMatchTime() - 3_s;            \
    auto gameData  = frc::DriverStation::GetGameSpecificMessage();        \
    /* If we have no game data, we cannot compute, assume hub is active,  \
    as its likely early in teleop.                                        \
     */                                                                   \
    if (gameData.empty()) {                                               \
      return true;                                                        \
    }                                                                     \
    bool redInactiveFirst = !(gameData.at(0) == 'B');                     \
    /* Shift was is active for blue if red won auto, or red if blue won auto. */ \
    bool shift1Active = (alliance.value() == frc::DriverStation::Alliance::kRed) ? !redInactiveFirst : redInactiveFirst; \
    if (matchTime > 130.0_s) {                                            \
      /* Transition shift, hub is active. */                              \
      return true;                                                        \
    } else if (matchTime > 105.0_s) {                                     \
      /* Shift 1 */                                                       \
      return shift1Active;                                                \
    } else if (matchTime > 80.0_s) {                                      \
      /* Shift 2 */                                                       \
      return !shift1Active;                                               \
    } else if (matchTime > 55.0_s) {                                      \
      /* Shift 3 */                                                       \
      return shift1Active;                                                \
    } else if (matchTime > 30.0_s) {                                      \
      /* Shift 4 */                                                       \
      return !shift1Active;                                               \
    } else {                                                              \
      /* End game, hub always active.  */                                 \
      return true;                                                        \
    }                                                                     \
})()

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
enum class LedMode
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

        void SetLeds(frc::LEDPattern turret, Blinker::Pattern underglow);

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
        Blinker             m_ledUnderGlow{ConstantsPwmPorts::LedUnderGlowPort};

        std::array<frc::AddressableLED::LEDData, LedConstants::Length> m_ledTurretBuffer{};

        Blinker::Pattern underGlowPattern = Blinker::Pattern::SolidColorsBlack;
        frc::LEDPattern  turretPattern    = frc::LEDPattern::Solid(frc::Color::kBlack);

        bool      m_isOnTarget = false;
        bool      m_isShooting = false;
        
        bool      lastIsShooting = m_isShooting;
        bool      lastIsOnTarget = m_isOnTarget;
};
