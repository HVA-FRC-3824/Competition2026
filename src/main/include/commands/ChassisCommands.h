#pragma once

#pragma region Includes
#include <pathplanner/lib/auto/AutoBuilder.h>

#include <frc/trajectory/TrajectoryGenerator.h>
#include <frc/controller/ProfiledPIDController.h>
#include <frc2/command/SwerveControllerCommand.h>

#include <frc2/command/CommandPtr.h>
#include <frc2/command/InstantCommand.h>
#include <frc2/command/WaitCommand.h>

#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Transform2d.h>
#include <frc/kinematics/ChassisSpeeds.h>

#include "subsystems/Chassis.h"
#pragma endregion

frc2::CommandPtr ChassisZeroHeading(Chassis* chassis);

frc2::CommandPtr ChassisXMode(Chassis* chassis);

frc2::CommandPtr ChassisDrive(Chassis* chassis, std::function<frc::ChassisSpeeds()> chassisSpeedsSupplier);

frc2::CommandPtr ChassisDrivePose(Chassis* chassis, frc::Pose2d targetPose);

frc2::CommandPtr ToggleFieldCentricity(Chassis* chassis);
