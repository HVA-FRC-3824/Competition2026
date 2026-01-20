#pragma once

typedef int CANid_t;

#pragma region ConstantsCanIds
namespace ConstantsCanIds
{
    const     auto CanBus                    = "rio";

    // Motor and encoders CAN IDs
    constexpr CANid_t frontLeftDriveCANid    = 01;
    constexpr CANid_t frontLeftTurnCANid     = 02;
    constexpr CANid_t frontLeftEncoderCANid  = 03;

    constexpr CANid_t frontRightDriveCANid   = 11;
    constexpr CANid_t frontRightTurnCANid    = 12;
    constexpr CANid_t frontRightEncoderCANid = 13;

    constexpr CANid_t backLeftDriveCANid     = 31;
    constexpr CANid_t backLeftTurnCANid      = 32;
    constexpr CANid_t backLeftEncoderCANid   = 33;

    constexpr CANid_t backRightDriveCANid    = 21;
    constexpr CANid_t backRightTurnCANid     = 22;
    constexpr CANid_t backRightEncoderCANid  = 23;

    constexpr CANid_t intakeTurnMotorId      = 30;
    constexpr CANid_t intakeDriveMotorId     = 31;

    constexpr CANid_t turretMotorID          = 40;

    constexpr CANid_t actuatorID             = 50;
    constexpr CANid_t flywheelMotorID        = 51;

    constexpr CANid_t indexerMotor1ID        = 60;
    constexpr CANid_t indexerMotor2ID        = 61;
}
#pragma endregion
