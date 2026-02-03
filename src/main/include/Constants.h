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

typedef int Button;

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
        // Drive Input Configurations
        constexpr int    DrivePort           = 0;
        constexpr int    OperatorPort        = 1;

        constexpr double TranslationDeadZone = 0.06;
        constexpr double RotateDeadZone      = 0.06;
        constexpr double TriggerDeadZone     = 0.06;

        constexpr double ExponentForward     = 3.0;
        constexpr double ExponentStrafe      = 3.0;
        constexpr double ExponentAngle       = 3.0;

        // Buttons
        constexpr Button A                   =   1;
        constexpr Button B                   =   2;
        constexpr Button X                   =   3;
        constexpr Button Y                   =   4;
        constexpr Button LeftBumper          =   5;
        constexpr Button RightBumper         =   6;
        constexpr Button Back                =   7;
        constexpr Button Start               =   8;
        constexpr Button LeftStickButton     =   9;
        constexpr Button RightStickButton    =  10;

        constexpr Button Pov_0               =   0;
        constexpr Button Pov_45              =  45;
        constexpr Button Pov_90              =  90;
        constexpr Button Pov_135             = 135;
        constexpr Button Pov_180             = 180;
        constexpr Button Pov_225             = 225;
        constexpr Button Pov_270             = 270;
        constexpr Button Pov_315             = 315;
    }
    #pragma endregion
}
