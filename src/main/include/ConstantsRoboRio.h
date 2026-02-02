#pragma once

typedef int CANid_t;

#pragma region ConstantsCanIds
namespace ConstantsCanIds
{
    // CAN IDs
    const     auto    CanBus                 = "rio";

    // Swerve motor and encoders CAN IDs
    constexpr CANid_t FrontLeftDriveId       = 01;
    constexpr CANid_t FrontLeftTurnId        = 02;
    constexpr CANid_t FrontLeftEncoderId     = 03;

    constexpr CANid_t FrontRightDriveId      = 11;
    constexpr CANid_t FrontRightTurnId       = 12;
    constexpr CANid_t FrontRightEncoderId    = 13;

    constexpr CANid_t BackLeftDriveId        = 21;
    constexpr CANid_t BackLeftTurnId         = 22;
    constexpr CANid_t BackLeftEncoderId      = 23;
    
    constexpr CANid_t BackRightDriveId       = 31;
    constexpr CANid_t BackRightTurnId        = 32;
    constexpr CANid_t BackRightEncoderId     = 33;

    constexpr CANid_t IntakePositionMotorId  = 40;
    constexpr CANid_t FuelIntakeMotorId      = 41;

    constexpr CANid_t SpinnerMotorId         = 50;
    constexpr CANid_t KickerMotorId          = 51;
    constexpr CANid_t TurretMotorId          = 52;
    constexpr CANid_t FlywheelMotorId        = 53;

    constexpr CANid_t ClimbMotorId           = 60;
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
