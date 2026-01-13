#pragma once

#pragma region Includes
#include <frc2/command/CommandPtr.h>

#include <frc2/command/InstantCommand.h>
#include <frc2/command/WaitCommand.h>

#include "subsystems/Flywheel.h"
#include "subsystems/Hood.h"
#include "subsystems/Indexer.h"
#include "subsystems/Turret.h"
#pragma endregion

#pragma region TurretAimHub
// Points towards the hub of the bots alliance
frc2::CommandPtr TurretAimHub(Turret* turret, std::function<frc::Pose2d> poseSupplier);
#pragma endregion

#pragma region TurretAimHome
// This points the turret towards home for passing
frc2::CommandPtr TurretAimHome(Turret* turret, std::function<frc::Pose2d> poseSupplier);
#pragma endregion

#pragma region TurretAimStatic
// This just points the turret in the direction of the robot
frc2::CommandPtr TurretAimStatic(Turret* turret);
#pragma endregion

#pragma region HoodSetAngle
// This controls the hood angle
frc2::CommandPtr HoodSetAngle(Hood* hood, units::degree_t angle);
#pragma endregion

#pragma region HoodAimHub
// This aims the hood to the hub based on bot location (could change this to bot distance?)
frc2::CommandPtr HoodAimHub(Hood* hood, std::function<frc::Pose2d> poseSupplier);
#pragma endregion

#pragma region IntakeOn
// This turns the intake on
frc2::CommandPtr IntakeOn(Intake* intake);
#pragma endregion

#pragma region IntakeOff
// This turns the intake off
frc2::CommandPtr IntakeOff(Intake* intake);
#pragma endregion

#pragma region FlyWheelSetSpeed
// This sets the flywheel speed
frc2::CommandPtr FlyWheelSetSpeed(Flywheel* flywheel, units::turns_per_second_t speed);
#pragma endregion