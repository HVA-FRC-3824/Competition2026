#pragma once

typedef int CANid_t;

#pragma region ConstantsCanIds
namespace ConstantsCanIds
{
    // CAN IDs
    const     auto CanBus                    = "rio";

    // Swerve motor and encoders CAN IDs
    constexpr int frontLeftDriveCANid    = 01;
    constexpr int frontLeftTurnCANid     = 02;
    constexpr int frontLeftEncoderCANid  = 03;

    constexpr int frontRightDriveCANid   = 11;
    constexpr int frontRightTurnCANid    = 12;
    constexpr int frontRightEncoderCANid = 13;

    constexpr int backLeftDriveCANid     = 21;
    constexpr int backLeftTurnCANid      = 22;
    constexpr int backLeftEncoderCANid   = 23;

    constexpr CANid_t backRightDriveCANid    = 21;
    constexpr CANid_t backRightTurnCANid     = 22;
    constexpr CANid_t backRightEncoderCANid  = 23;

    constexpr CANid_t intakeTurnMotorId      = 30;
    constexpr CANid_t intakeDriveMotorId     = 31;

    constexpr CANid_t turretMotorID          = 40;

    constexpr CANid_t flywheelMotorID        = 51;

    constexpr CANid_t spinnerMotorID         = 60;
    constexpr CANid_t kickerMotorID          = 61;

    // PWM Ports
    constexpr int actuatorID             = 2;
}
#pragma endregion
