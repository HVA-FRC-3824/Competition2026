#pragma once

typedef unsigned long long int CanId_t;
typedef unsigned long long int PwmPort_t;
typedef unsigned long long int UsbPort_t;

constexpr CanId_t  operator "" _CAN(unsigned long long int id)
{
    return CanId_t{id};
}

constexpr PwmPort_t operator "" _PWM(unsigned long long int id)
{
    return CanId_t{id};
}

constexpr UsbPort_t operator "" _USB(unsigned long long int id)
{
    return CanId_t{id};
}

#pragma region ConstantsCanIds
namespace ConstantsCanIds
{
    const auto CanBus = "rio";

    // Swerve motor and encoders CAN IDs
    constexpr auto FrontLeftDriveId        = 01_CAN; // Kraken X60
    constexpr auto FrontLeftTurnId         = 02_CAN; // Kraken X44
    constexpr auto FrontLeftEncoderId      = 03_CAN; // CANCoder

    constexpr auto FrontRightDriveId       = 11_CAN; // Kraken X60
    constexpr auto FrontRightTurnId        = 12_CAN; // Kraken X44
    constexpr auto FrontRightEncoderId     = 13_CAN; // CANCoder

    constexpr auto BackLeftDriveId         = 21_CAN; // Kraken X60
    constexpr auto BackLeftTurnId          = 22_CAN; // Kraken X44
    constexpr auto BackLeftEncoderId       = 23_CAN; // CANCoder
    
    constexpr auto BackRightDriveId        = 31_CAN; // Kraken X60
    constexpr auto BackRightTurnId         = 32_CAN; // Kraken X44
    constexpr auto BackRightEncoderId      = 33_CAN; // CANCoder

    constexpr auto IntakePositionMotorId   = 40_CAN; // Kraken X60
    constexpr auto FuelIntakeMotorId       = 41_CAN; // Kraken X60

    constexpr auto SpinnerMotorId          = 50_CAN; // Kraken X60
    constexpr auto KickerMotorId           = 51_CAN; // Kraken X44
    constexpr auto KickerFollowerMotorId   = 52_CAN; // Kraken X44
    constexpr auto TurretMotorId           = 53_CAN; // Kraken X44
    constexpr auto FlywheelMotorId         = 54_CAN; // Kraken X60
    constexpr auto FlywheelFollowerMotorId = 55_CAN; // Kraken X60

    constexpr auto ClimbMotorId            = 60_CAN; // Kraken X60
}
#pragma endregion

#pragma region ConstantsPwmPorts
namespace ConstantsPwmPorts
{
    // PWM Ports
    constexpr auto ActuatorPort     = 1_PWM;

    constexpr auto LedUnderGlowPort = 9_PWM;
    constexpr auto LedTurretPort    = 7_PWM;
}
#pragma endregion

namespace ConstantsUsbPort
{
    
    // Drive Input Configurations
    constexpr auto DrivePort    = 0_USB;
    constexpr auto OperatorPort = 1_USB;
}