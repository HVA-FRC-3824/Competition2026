#pragma once

#pragma region Includes
#include <array>
#include <numbers>

#include <units/base.h>
#include <units/angle.h>
#include <units/angular_velocity.h>
#include <units/velocity.h>
#include <units/length.h>
#include <units/voltage.h>

#include <frc/geometry/Transform3d.h>
#include <frc/apriltag/AprilTagFieldLayout.h>
#include <frc/smartdashboard/SmartDashboard.h>
#pragma endregion

typedef int Button;

namespace constants
{
    #pragma region Field
    namespace field
    {
        constexpr frc::Pose3d blueHub{182.11_in,             158.84_in, 72_in, frc::Rotation3d(0_deg)};
        constexpr frc::Pose3d redHub {325.61_in + 143.50_in, 158.84_in, 72_in, frc::Rotation3d(0_deg)};
    }
    #pragma endregion

    #pragma region Swerve
    namespace swerve
    {
        // NOTE: The absolute encoder range is 0.5 to -0.5

        // These are the angles that correspond to the wheels facing "forward"
        constexpr units::turn_t frontRightForwardAngle{-0.193604};
        constexpr units::turn_t frontLeftForwardAngle {-0.422119};
        constexpr units::turn_t rearRightForwardAngle {-0.174561};
        constexpr units::turn_t rearLeftForwardAngle  { 0.268555};

        // These make sure to limit how fast the robot can go
        constexpr units::meters_per_second_t                    maxSpeed          {4};
        constexpr units::angular_velocity::radians_per_second_t maxAngularVelocity{2 * std::numbers::pi};

        // The physical dimensions of the robot
        constexpr units::meter_t wheelBase {25.0};
        constexpr units::meter_t trackWidth{25.0};
    }
    #pragma endregion

    #pragma region Tower
    namespace tower
    {
        // This is -1 to 1, really 0 to 1
        constexpr double constantFlywheelSpeed = 0.8;
    
        // TODO: test these angles, likely isn't correct as we have a different than the bot, team 102 in 2022, I got it from
        constexpr units::degree_t MinAngle = 0_deg;
        constexpr units::degree_t MaxAngle = 50_deg;
    
        // TODO: test these lengths, they're most likely accurate
        // I got these from team 102 from 2022, they used the same actuator
        constexpr units::inch_t   MaxLength = 14.336_in;
        constexpr units::inch_t   MinLength = 8.946_in;

        // Comes from 102 too
        constexpr double          ActuatorLowerBound = -0.95;
        constexpr double          ActuatorUpperBound =  0.95;
    }
    #pragma endregion

    #pragma region Intake
    namespace intake
    {
        // TODO: These need to be tuned
        // Config for motor that will angle the intake between 0 and 90 degrees

        // TODO: These need to be tuned
        // Config for motor that will drive the intake
    }
    #pragma endregion

    #pragma region Controller
    namespace controller
    {
        // Drive Input Configurations
        constexpr int    DrivePort           = 0;

        constexpr double TranslationDeadZone = 0.06;
        constexpr double RotateDeadZone      = 0.06;
        constexpr double FlywheelDeadZone    = 0.06;

        constexpr double ExponentForward     = 3.0;
        constexpr double ExponentStrafe      = 3.0;
        constexpr double ExponentAngle       = 3.0;

        // BUTTONSSSSS
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

    #pragma region Vision
    namespace vision
    {
        constexpr std::string_view            CameraName{"PhotonCamera"};

        constexpr frc::Transform3d            RobotToCam{frc::Translation3d{0_m, 4_in, 15_in}, frc::Rotation3d{}};

        const     frc::AprilTagFieldLayout    TagLayout = frc::AprilTagFieldLayout::LoadField(frc::AprilTagField::k2025ReefscapeAndyMark);

        const     Eigen::Matrix<double, 3, 1> SingleTagStdDevs{4, 4, 8};
        const     Eigen::Matrix<double, 3, 1> MultiTagStdDevs{0.5, 0.5, 1};

        namespace AprilTagLocations
        {
            // These are kept because I really dont want to lose them. REEFSCAPE 2025
            constexpr frc::Pose2d Tags2d[22] = 
            { 
                { 657.37_in,  25.80_in, { 126_deg} }, { 657.37_in, 291.20_in, { 234_deg} },
                { 455.15_in, 317.15_in, { 270_deg} }, { 365.20_in, 241.64_in, {   0_deg} },
                { 365.20_in,  75.39_in, {   0_deg} }, { 530.49_in, 130.17_in, { 300_deg} },
                { 546.87_in, 158.50_in, {   0_deg} }, { 530.49_in, 186.83_in, {  60_deg} },
                { 497.77_in, 186.83_in, { 120_deg} }, { 481.39_in, 158.50_in, { 180_deg} },
                { 497.77_in, 130.17_in, { 240_deg} }, { 33.51_in,   25.80_in, {  54_deg} },
                { 33.51_in,  291.20_in, { 306_deg} }, { 325.68_in, 241.64_in, { 180_deg} },
                { 325.68_in,  75.39_in, { 180_deg} }, { 235.73_in,  -0.15_in, {  90_deg} },
                { 160.39_in, 130.17_in, { 240_deg} }, { 144.00_in, 158.50_in, { 180_deg} },
                { 160.39_in, 186.83_in, { 120_deg} }, { 193.10_in, 186.83_in, { 60_deg,} },
                { 209.49_in, 158.50_in, { 0_deg, } }, { 193.10_in, 130.17_in, { 300_deg} }
            };

            constexpr std::span<const frc::Pose2d> Pose2dTagsSpan{std::begin(Tags2d), std::end(Tags2d)};
        }
    }
    #pragma endregion
}
