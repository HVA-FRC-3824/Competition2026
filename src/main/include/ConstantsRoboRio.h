#pragma once

typedef int CanId_t;

#pragma region ConstantsCanIds
namespace ConstantsCanIds
{
    // CAN IDs
    const     auto    CanBus                  = "rio";

    // Swerve motor and encoders CAN IDs
    constexpr CanId_t FrontLeftDriveId        = 01;
    constexpr CanId_t FrontLeftTurnId         = 02;
    constexpr CanId_t FrontLeftEncoderId      = 03;

    constexpr CanId_t FrontRightDriveId       = 11;
    constexpr CanId_t FrontRightTurnId        = 12;
    constexpr CanId_t FrontRightEncoderId     = 13;

    constexpr CanId_t BackLeftDriveId         = 21;
    constexpr CanId_t BackLeftTurnId          = 22;
    constexpr CanId_t BackLeftEncoderId       = 23;
    
    constexpr CanId_t BackRightDriveId        = 31;
    constexpr CanId_t BackRightTurnId         = 32;
    constexpr CanId_t BackRightEncoderId      = 33;

    constexpr CanId_t IntakePositionMotorId   = 40;
    constexpr CanId_t FuelIntakeMotorId       = 41;

    constexpr CanId_t SpinnerMotorId          = 50;
    constexpr CanId_t KickerMotorId           = 51;
    constexpr CanId_t KickerFollowerMotorId   = 52;
    constexpr CanId_t TurretMotorId           = 53;
    constexpr CanId_t FlywheelMotorId         = 54;
    constexpr CanId_t FlywheelFollowerMotorId = 55;

    constexpr CanId_t ClimbMotorId            = 60;
}
#pragma endregion

#pragma region ConstantsPwmPorts
namespace ConstantsPwmPorts
{
    // PWM Ports
    constexpr auto ActuatorPort              =  2;
    constexpr auto LedPort                   =  9;
}
#pragma endregion
