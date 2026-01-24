#include "subsystems/Tower.h"

#pragma region Tower
/// @brief Constructor for the Tower subsystem
/// @param poseSupplier 
Tower::Tower(std::function<frc::Pose2d()> poseSupplier, std::function<frc::ChassisSpeeds()> speedsSupplier) :
    m_poseSupplier  {poseSupplier},
    m_speedsSupplier{speedsSupplier}
{
    // Configure the tower motors
    TalonFXConfiguration(&m_turretMotor,
                          40_A,
                          true,
                          0.0,
                          0.0,
                          0.0,
                          0.0,
                          0.0,
                          0.0,
                          0_tps,
                          units::turns_per_second_squared_t{0});

    TalonFXConfiguration(&m_flywheelMotor,
                          40_A,
                          true,
                          0.0,
                          0.0,
                          0.0,
                          0.0,
                          0.0,
                          0.0,
                          0_tps,
                          units::turns_per_second_squared_t{0});

    m_hoodActuator.SetBounds(2.0_us, 1.8_us, 1.5_us, 1.2_us, 1.0_us);
}
#pragma endregion

#pragma region SetState
/// @brief Sets the current state of the Tower subsystem
/// @param newState The new state to set for the Tower subsystem
void Tower::SetState(TowerState newState)
{
    // Set the current state of the Tower subsystem
    m_state = newState;
}
#pragma endregion

#pragma region GetState
/// @brief Gets the current state of the Tower subsystem
/// @return TowerState The current state of the Tower subsystem
TowerState Tower::GetState()
{
    // Return the current state of the Tower subsystem
    return m_state;
}
#pragma endregion

#pragma region Periodic
/// @brief Periodic method for the Tower subsystem, called periodically by the CommandScheduler
void Tower::Periodic()
{
    bool isTurretRobotRelative = true;

    // Update the current pose and speed
    auto pose  = m_poseSupplier();
    auto speed = m_speedsSupplier();

    // Does not need to be initialized every cycle, 
    static bool isBlue = frc::DriverStation::GetAlliance().value_or(frc::DriverStation::Alliance::kBlue) == frc::DriverStation::Alliance::kBlue;

    switch (m_state.mode) 
    {
        case TowerMode::ShootingToHub:
        {
            isTurretRobotRelative = false;
            frc::Pose3d Hub = isBlue ? constants::field::blueHub : constants::field::redHub;

            auto relativeDistance = Hub.ToPose2d().Translation() - pose.Translation();
            // This is based off of the position of the robot based on vision and odo so it may be a little bit unreliable sometimes
            if (TowerConstants::usingTurretCamera)
            {
                std::vector<photon::PhotonPipelineResult> results = m_turretCam.GetAllUnreadResults();
                
                if (!results.empty())
                {
                    auto latestResult = results.back();
                    for (auto tag : latestResult.GetTargets())
                    {
                        // TODO: verify if these are right they might be the ones next to these
                        if (tag.fiducialId == 10 || tag.fiducialId == 26)
                        {
                            // TODO: remove magic numbers, this just converts radians to degrees
                            m_state.turretAngle -= 57.2958_deg * tag.altCameraToTarget.Rotation().Angle().value();
                        }
                    }
                }
            }
            m_state = CalculateShot(relativeDistance, speed);
            break;
        }

        case TowerMode::PassingToAdjacentZone:
        {
            isTurretRobotRelative = false;

            // Based on alliance color, find the nearest point in our alliance zone to pass to
            // We decide between the close and the far points so that we can avoid hitting the hub net
            auto targetPoint = pose.Translation().Nearest({isBlue ? constants::field::blueAllianceZoneClose.Translation() : constants::field::redAllianceZoneClose.Translation(),
                                                           isBlue ? constants::field::blueAllianceZoneFar.Translation()   : constants::field::redAllianceZoneFar.Translation()});
            
            auto relativeDistance = targetPoint - pose.Translation();

            if (TowerConstants::usingTurretCamera)
            {
                m_state.turretAngle = 57.2958_deg * std::atan2(relativeDistance.Y().value(), relativeDistance.X().value());
            }

            m_state = CalculateShot(relativeDistance, speed);
            break;
        }

        default:
        {
            break;
        }
    }

    // Apply the calculated state to the hardware
    if (isTurretRobotRelative)
    {
        SetTurret(m_state.turretAngle);
    } 
    else 
    {
        SetTurret(pose.Rotation().Degrees() - m_state.turretAngle);
    }

    // Set flywheel speed and hood actuator position
    SetFlywheel(m_state.flywheelSpeed);
    SetActuator(m_state.hoodActuatorPercentInput);
}
#pragma endregion

#pragma region SetFlywheel
/// @brief Spins up the flywheel motor
/// @param input The input value to set the flywheel motor speed
void Tower::SetFlywheel(double input)
{
    // Set the flywheel motor speed
    m_flywheelMotor.Set(input);
}
#pragma endregion

#pragma region SetActuator
/// @brief Activates the actuator which moves linearly to move the hood by some degrees
/// @param position The position input value (0-1) to set the hood actuator 
void Tower::SetActuator(double position)
{
    // range: 0-1
	position = std::clamp(position, 0.0, 1.0);

    // range: 0-2.0
    position *= 2.0;

    // range: -1, 1
    position -= 1;

    // Although this says SetSpeed, this actually does position
	m_hoodActuator.SetSpeed(std::clamp(position, TowerConstants::ActuatorLowerBound, TowerConstants::ActuatorUpperBound));
}
#pragma endregion

#pragma region SetTurret
/// @brief Sets the desired angle of the turret relative to the robot
/// @param angle The angle in degrees to set the turret to
void Tower::SetTurret(units::degree_t angle)
{
    // Range: any to any => min to max degrees
    // Do not allow turret to move past the min or max angles
    angle = 1_deg * std::fmod(angle.value(), TowerConstants::MaxAngle.value());
    while (angle < TowerConstants::MinAngle)
        angle += 360.0_deg;

    // Convert degrees to rotations (turns) for TalonFX
    units::angle::turn_t rotations{angle.value() / 360.0};

    // Set the motor to the desired position
    m_turretMotor.SetControl(ctre::phoenix6::controls::PositionDutyCycle{rotations});
}
#pragma endregion

#pragma region CalculateShot
/// @brief Changes the turret angle, flywheel speed, and hood actuator position based on distance and speed to target
/// @param relativeDistance The distance to the target from the bot
/// @param speed The speed of the robot comparative to the field
/// @return The calculated TowerState based on the input parameters
TowerState Tower::CalculateShot(frc::Translation2d relativeDistance, frc::ChassisSpeeds speed)
{
    TowerState newState{TowerMode::ManualControl, 0_deg, 0.0, 0.0};

    // Create a new Pose2d from the relative distance and apply speed based translations
    frc::Pose2d newRelativeDistance = frc::Pose2d{relativeDistance, 0_deg};
    // Predict where the target will be in 0.5 seconds using frc::Twist2d
    frc::Twist2d changeInPosition = speed.ToTwist2d(0.5_s); 

    // Add that to the distance to target
    newRelativeDistance = newRelativeDistance.TransformBy(frc::Transform2d{changeInPosition.dx, changeInPosition.dy, changeInPosition.dtheta});

    // Calculate the angle to the target point
    if (!TowerConstants::usingTurretCamera)
    {
        newState.turretAngle = 57.2958_deg * std::atan2(newRelativeDistance.Y().value(), newRelativeDistance.X().value());
    }

    // Adjust the angle based on the robot's movement
    newState.turretAngle += newRelativeDistance.Rotation().Degrees();

    // TODO: Implement a real calculation for shot parameters based on distance and speed

    return newState;
}
#pragma endregion
