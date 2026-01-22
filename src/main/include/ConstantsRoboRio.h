#pragma once

typedef int CANid_t;

#pragma region ConstantsCanIds
namespace ConstantsCanIds
{
    // CAN IDs
    const     auto    CanBus                 = "rio";

    // Swerve motor and encoders CAN IDs
    constexpr CANid_t frontLeftDriveCANid    = 01;
    constexpr CANid_t frontLeftTurnCANid     = 02;
    constexpr CANid_t frontLeftEncoderCANid  = 03;

    constexpr CANid_t frontRightDriveCANid   = 11;
    constexpr CANid_t frontRightTurnCANid    = 12;
    constexpr CANid_t frontRightEncoderCANid = 13;

    constexpr CANid_t backLeftDriveCANid     = 21;
    constexpr CANid_t backLeftTurnCANid      = 22;
    constexpr CANid_t backLeftEncoderCANid   = 23;

    constexpr CANid_t backRightDriveCANid    = 31;
    constexpr CANid_t backRightTurnCANid     = 32;
    constexpr CANid_t backRightEncoderCANid  = 33;

    constexpr CANid_t intakeTurnMotorId      = 40;
    constexpr CANid_t intakeDriveMotorId     = 41;

    constexpr CANid_t spinnerMotorID         = 50;
    constexpr CANid_t kickerMotorID          = 51;
    constexpr CANid_t turretMotorID          = 52;
    constexpr CANid_t flywheelMotorID        = 53;
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
