#pragma once

typedef int CanId_t;

#pragma region ConstantsCanIds
namespace ConstantsCanIds
{
    const     auto    CanBus                  = "rio";

    // Swerve motor and encoders CAN IDs
    constexpr CanId_t FrontLeftDriveId        = 01; // Kraken X60
    constexpr CanId_t FrontLeftTurnId         = 02; // Kraken X44
    constexpr CanId_t FrontLeftEncoderId      = 03; // CANCoder

    constexpr CanId_t FrontRightDriveId       = 11; // Kraken X60
    constexpr CanId_t FrontRightTurnId        = 12; // Kraken X44
    constexpr CanId_t FrontRightEncoderId     = 13; // CANCoder

    constexpr CanId_t BackLeftDriveId         = 21; // Kraken X60
    constexpr CanId_t BackLeftTurnId          = 22; // Kraken X44
    constexpr CanId_t BackLeftEncoderId       = 23; // CANCoder
    
    constexpr CanId_t BackRightDriveId        = 31; // Kraken X60
    constexpr CanId_t BackRightTurnId         = 32; // Kraken X44
    constexpr CanId_t BackRightEncoderId      = 33; // CANCoder

    constexpr CanId_t IntakePositionMotorId   = 40; // Kraken X60
    constexpr CanId_t FuelIntakeMotorId       = 41; // Kraken X60

    constexpr CanId_t SpinnerMotorId          = 50; // Kraken X60
    constexpr CanId_t KickerMotorId           = 51; // Kraken X44
    constexpr CanId_t KickerFollowerMotorId   = 52; // Kraken X44
    constexpr CanId_t TurretMotorId           = 53; // Kraken X44
    constexpr CanId_t FlywheelMotorId         = 54; // Kraken X60
    constexpr CanId_t FlywheelFollowerMotorId = 55; // Kraken X60

    constexpr CanId_t ClimbMotorId            = 60; // Kraken X60
}
#pragma endregion

#pragma region ConstantsPwmPorts
namespace ConstantsPwmPorts
{
    // PWM Ports
    constexpr auto ActuatorPort     = 1;

    constexpr auto LedUnderGlowPort = 8;
    constexpr auto LedTurretPort    = 9;
}
#pragma endregion
