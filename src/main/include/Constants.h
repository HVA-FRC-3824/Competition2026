#pragma once

#pragma region Includes
#include <array>

#include <units/base.h>
#include <units/angle.h>
#include <units/angular_velocity.h>
#include <units/velocity.h>
#include <units/length.h>

#include <frc/geometry/Transform3d.h>
#include <frc/apriltag/AprilTagFieldLayout.h>
#pragma endregion

typedef unsigned long long int Button_t;

constexpr Button_t operator "" _btn(unsigned long long int id)
{
    return Button_t{id};
}

namespace constants
{
    #pragma region Field
    namespace Field
    {
        /// *** Field Dimensions *** ///
        constexpr units::inch_t FieldLength  = 652.11_in; // 16.56 meters
        constexpr units::inch_t FieldWidth   = 317.69_in; //  8.07 meters

        constexpr units::inch_t AllianceWallToAllianceZone = 182.11_in;

        constexpr units::inch_t HubHeight = 72_in;

        /// *** Field Locations *** ///

        constexpr frc::Pose3d BlueHub{AllianceWallToAllianceZone,               FieldWidth / 2, HubHeight, frc::Rotation3d(0_deg)};
        constexpr frc::Pose3d RedHub {FieldLength - AllianceWallToAllianceZone, FieldWidth / 2, HubHeight, frc::Rotation3d(0_deg)};

        // For passing we want to aim towards the inside of our alliance zone or towards the neutral zone whichever is closer
        // Either way we want the balls to be going as close to our alliance zone as possible, so aim for that
        // - "Aim for the stars and maybe you'll reach the neutral zone" or something like that...

        constexpr frc::Pose2d BlueAllianceZoneClose{AllianceWallToAllianceZone, FieldWidth / 4, 0_rad};
        constexpr frc::Pose2d BlueAllianceZoneFar  {AllianceWallToAllianceZone, FieldWidth - (FieldWidth / 4), 0_rad};

        constexpr frc::Pose2d RedAllianceZoneClose{FieldLength - AllianceWallToAllianceZone, FieldWidth / 4, 0_rad};
        constexpr frc::Pose2d RedAllianceZoneFar  {FieldLength - AllianceWallToAllianceZone, FieldWidth - (FieldWidth / 4), 0_rad};
    }
    #pragma endregion

    #pragma region Controller
    namespace controller
    {
        constexpr double TranslationDeadZone = 0.06;
        constexpr double RotateDeadZone      = 0.06;
        constexpr double TriggerDeadZone     = 0.06;

        constexpr double ExponentForward     = 3.0;
        constexpr double ExponentStrafe      = 3.0;
        constexpr double ExponentAngle       = 3.0;

        // Buttons
        constexpr auto A                   =   1_btn;
        constexpr auto B                   =   2_btn;
        constexpr auto X                   =   3_btn;
        constexpr auto Y                   =   4_btn;
        constexpr auto LeftBumper          =   5_btn;
        constexpr auto RightBumper         =   6_btn;
        constexpr auto Back                =   7_btn;
        constexpr auto Start               =   8_btn;
        constexpr auto LeftStickButton     =   9_btn;
        constexpr auto RightStickButton    =  10_btn;

        constexpr auto Pov_0               =   0_btn;
        constexpr auto Pov_45              =  45_btn;
        constexpr auto Pov_90              =  90_btn;
        constexpr auto Pov_135             = 135_btn;
        constexpr auto Pov_180             = 180_btn;
        constexpr auto Pov_225             = 225_btn;
        constexpr auto Pov_270             = 270_btn;
        constexpr auto Pov_315             = 315_btn;
    }
    #pragma endregion
}
