#pragma once

typedef int CANid_t;

#pragma region ConstantsCanIds
namespace ConstantsCanIds
{
    // CAN IDs
    const     auto    CanBus                 = "rio";

    // Swerve motor and encoders CAN IDs
    constexpr CANid_t frontLeftDriveId      = 01;
    constexpr CANid_t frontLeftTurnId       = 02;
    constexpr CANid_t frontLeftEncoderId    = 03;

    constexpr CANid_t frontRightDriveId     = 11;
    constexpr CANid_t frontRightTurnId      = 12;
    constexpr CANid_t frontRightEncoderId   = 13;

    constexpr CANid_t backLeftDriveId       = 21;
    constexpr CANid_t backLeftTurnId        = 22;
    constexpr CANid_t backLeftEncoderId     = 23;

    constexpr CANid_t backRightDriveId      = 31;
    constexpr CANid_t backRightTurnId       = 32;
    constexpr CANid_t backRightEncoderId    = 33;

    constexpr CANid_t intakePositionMotorId = 40;
    constexpr CANid_t fuelIntakeMotorId     = 41;

    constexpr CANid_t spinnerMotorId         = 50;
    constexpr CANid_t kickerMotorId          = 51;
    constexpr CANid_t turretMotorId          = 52;
    constexpr CANid_t flywheelMotorId        = 53;
}
#pragma endregion

#pragma region ConstantsPwmPorts
namespace ConstantsPwmPorts
{
    // PWM Ports
    constexpr auto actuatorPort              =  2;
    constexpr auto ledPort                   =  9;
}
#pragma endregion
