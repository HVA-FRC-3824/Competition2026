#pragma once

#include <frc/motorcontrol/Spark.h>
#include <frc/LEDPattern.h>

class Blinker
{
    public:
        
        enum class Pattern
        {
            FixedPaletteRainbowRainbowPalette = 199,
            FixedPaletteRainbowPartyPalette = 197,
            FixedPaletteRainbowOceanPalette = 195,
            FixedPaletteRainbowLavePalette = 193,
            FixedPaletteRainbowForestPalette = 191,
            FixedPaletteRainbowWithGlitter = 189,
            FixedPaletteConfetti = 187,
            FixedPaletteShotRed = 185,
            FixedPaletteShotBlue = 183,
            FixedPaletteShotWhite = 181,
            FixedPaletteSinelonRainbowPalette = 179,
            FixedPaletteSinelonPartyPalette = 177,
            FixedPaletteSinelonOceanPalette = 175,
            FixedPaletteSinelonLavaPalette = 173,
            FixedPaletteSinelonForestPalette = 171,
            FixedPaletteBeatsPerMinuteRainbowPalette = 169,
            FixedPaletteBeatsPerMinutePartyPalette = 167,
            FixedPaletteBeatsPerMinuteOceanPalette = 165,
            FixedPaletteBeatsPerMinuteLavaPalette = 163,
            FixedPaletteBeatsPerMinuteForestPalette = 161,
            FixedPaletteFireMedium = 159,
            FixedPaletteFireLarge = 157,
            FixedPaletteTwinklesRainbowPalette = 155,
            FixedPaletteTwinklesPartyPalette = 153,
            FixedPaletteTwinklesOceanPalette = 151,
            FixedPaletteTwinklesLavaPalette = 149,
            FixedPaletteTwinklesForestPalette = 147,
            FixedPaletteColorWavesRainbowPalette = 145,
            FixedPaletteColorWavesPartyPalette = 143,
            FixedPaletteColorWavesOceanPalette = 141,
            FixedPaletteColorWavesLavaPalette = 139,
            FixedPaletteColorWavesForestPalette = 137,
            FixedPaletteLarsonScannerRed = 135,
            FixedPaletteLarsonScannerGray = 133,
            FixedPaletteLightChaseRed = 131,
            FixedPaletteLightChaseBlue = 129,
            FixedPaletteLightChaseGray = 127,
            FixedPaletteHeartbeatRed = 125,
            FixedPaletteHeartbeatBlue = 123,
            FixedPaletteHeartbeatWhite = 121,
            FixedPaletteHeartbeatGray = 119,
            FixedPaletteBreathRed = 117,
            FixedPaletteBreathBlue = 115,
            FixedPaletteBreathGray = 113,
            FixedPaletteStrobeRed = 111,
            FixedPaletteStrobeBlue = 109,
            FixedPaletteStrobeGold = 107,
            FixedPaletteStrobeWhite = 105,
            ColorOneEndToEndBlendToBlack = 103,
            ColorOneLarsonScanner = 101,
            ColorOneLightChase = 1,
            ColorOneHeartbeatSlow = 3,
            ColorOneHeartbeatMedium = 5,
            ColorOneHeartbeatFast = 7,
            ColorOneBreathSlow = 9,
            ColorOneBreathFast = 11,
            ColorOneShot = 13,
            ColorOneStrobe = 15,
            ColorTwoEndToEndBlendToBlack = 17,
            ColorTwoLarsonScanner = 19,
            ColorTwoLightChase = 21,
            ColorTwoHeartbeatSlow = 23,
            ColorTwoHeartbeatMedium = 25,
            ColorTwoHeartbeatFast = 27,
            ColorTwoBreathSlow = 29,
            ColorTwoBreathFast = 31,
            ColorTwoShot = 33,
            ColorTwoStrobe = 35,
            ColorOneAndTwoSparkleColorOneOnColorTwo = 37,
            ColorOneAndTwoSparkleColorTwoOnColorOne = 39,
            ColorOneAndTwoColorGradientColorOneAndTwo = 41,
            ColorOneAndTwoBeatsPerMinuteColorOneAndTwo = 43,
            ColorOneAndTwoEndToEndBlendColorOneToTwo = 45,
            ColorOneAndTwoEndToEndBlend = 47,
            ColorOneAndTwoColorOneAndColorTwoNoBlending = 49,
            ColorOneAndTwoTwinklesColorOneAndTwo = 51,
            ColorOneAndTwoColorWavesColorOneAndTwo = 53,
            ColorOneAndTwoSinelonColorOneAndTwo = 55,
            SolidColorsHotPink = 57,
            SolidColorsDarkRed = 59,
            SolidColorsRed = 61,
            SolidColorsRedOrange = 63,
            SolidColorsOrange = 65,
            SolidColorsGold = 67,
            SolidColorsYellow = 69,
            SolidColorsLawnGreen = 71,
            SolidColorsLime = 73,
            SolidColorsDarkGreen = 75,
            SolidColorsGreen = 77,
            SolidColorsBlueGreen = 79,
            SolidColorsAqua = 81,
            SolidColorsSkyBlue = 83,
            SolidColorsDarkBlue = 85,
            SolidColorsBlue = 87,
            SolidColorsBlueViolet = 89,
            SolidColorsViolet = 91,
            SolidColorsWhite = 93,
            SolidColorsGray = 95,
            SolidColorsDarkGray = 97,
            SolidColorsBlack = 99,
        };
        
        explicit Blinker(int port) :
            m_led{port}
        {

        }

        void Set(Pattern pattern)
        {
            m_led.Set(PatternToBlink(pattern));
        }

    private:

        double PatternToBlink(Pattern pattern)
        {
           double pattern2 = (int) pattern;
           if (pattern2 > 100.0)
           {
            pattern2 -= 100.0;
            pattern2 /= 100.0;
            pattern2 *= -1.0;
            return pattern2;
           }
            return pattern2 / 100.0;
        }
        
        frc::Spark m_led;
};