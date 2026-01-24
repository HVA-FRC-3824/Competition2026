#include "subsystems/Tower.h"

#pragma region Tower
/// @brief Constructor for the Tower subsystem
/// @param chassisPoseSupplier 
Tower::Tower(std::function<frc::Pose2d()> chassisPoseSupplier, std::function<frc::ChassisSpeeds()> chassisSpeedsSupplier) :
    m_chassisPoseSupplier  {chassisPoseSupplier},
    m_chassisSpeedsSupplier{chassisSpeedsSupplier}
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

    // Initialize the pose with the current pose and timestamp
    m_hoodActuator.SetBounds(2.0_us, 1.8_us, 1.5_us, 1.2_us, 1.0_us);
}
#pragma endregion

#pragma region SetState
/// @brief Sets the current state of the Tower subsystem
/// @param newState The new state to set for the Tower subsystem
void Tower::SetState(TowerState newState)
{
    // Remember the state
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

    // Update the chassis current pose and speed
    auto chassisPose  = m_chassisPoseSupplier();
    auto chassisSpeed = m_chassisSpeedsSupplier();

    switch (m_state.mode) 
    {
        case TowerMode::Idle:
        {
            // Do nothing, just use the provided state
            m_state = TowerState{TowerMode::Idle, 0.0_deg, 0.0_tps, 0.0_in};
            break;
        }

        case TowerMode::ShootingToHub:
        {
            // If using turret camera, the turret angle is relative to the robot
            // Otherwise, it is relative to the field as its based on poses
            isTurretRobotRelative = false;

            frc::Translation2d relativeDistance;

            // When using the turret camera, relative distance is based on the turret
            if (m_usingTurretCamera)
            {
                std::vector<photon::PhotonPipelineResult> results = m_turretCam.GetAllUnreadResults();
                
                if (!results.empty())
                {
                    auto latestResult = results.back();
                    for (auto tag : latestResult.GetTargets())
                    {
                        if (tag.fiducialId == 10 || tag.fiducialId == 26)
                        {
                            // Extract the x and y distances to the target
                            relativeDistance = tag.GetBestCameraToTarget().Translation().ToTranslation2d();
             
                            // Offset from fiducials 10 and 26 to hub center
                            frc::Translation2d offset10And26ToHub{23.5_in, 0.0_m};

                            // Compensate for the offset from the turret camera to the hub center
                            auto ShootingDistance = (relativeDistance + offset10And26ToHub).Norm();

                            // Adjust turret angle based on target yaw
                            m_state.turretAngle += 1_deg * std::fmod((tag.GetYaw() + 180), 180);
                        }
                    }
                }
                else
                {
                    // No targets found, do not shoot
                    return;
                }
            } 
            else // If not using the turret camera, base relative distance on field pose
            {
                // Sets our hub based on our alliance
                frc::Pose3d Hub = m_isBlue ? constants::field::blueHub : constants::field::redHub;

                // TODO: Make constant, Make real/realistic
                frc::Transform3d offsetTurretFromRobotCenter{frc::Translation3d{0.0_m, 0.0_m, 0.0_m}, frc::Rotation3d{0_deg, 0_deg, 0_deg}};
                
                // Calculate the relative distance from the turret center to the hub
                relativeDistance = Hub.ToPose2d().Translation() - (chassisPose.Translation() + offsetTurretFromRobotCenter.Translation().ToTranslation2d());

                // TODO: How to handle Red and Blue hub positions relative to the robot?
 
            }

            // Calculate the shot parameters based on the relative distance and chassis speed
            m_state = CalculateShot(relativeDistance, chassisSpeed);
            break;
        }

        case TowerMode::PassingToAdjacentZone:
        {
            isTurretRobotRelative = false;

            // Based on alliance color, find the nearest point in our alliance zone to pass to
            // We decide between the close and the far points so that we can avoid hitting the hub net
            auto targetPoint = chassisPose.Translation().Nearest({m_isBlue ? constants::field::blueAllianceZoneClose.Translation() : constants::field::redAllianceZoneClose.Translation(),
                                                                  m_isBlue ? constants::field::blueAllianceZoneFar.Translation()   : constants::field::redAllianceZoneFar.Translation()});
            
            auto relativeDistance = targetPoint - chassisPose.Translation();

            m_state.turretAngle = 57.2958_deg * std::atan2(relativeDistance.Y().value(), relativeDistance.X().value());

            m_state = CalculateShot(relativeDistance, chassisSpeed);
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
        // Compensate for robot rotation
        // If we are on red, our gyro will be turned around, flip it
        SetTurret((chassisPose.Rotation().Degrees() - (m_isBlue ? 0_deg : 180_deg)) - m_state.turretAngle);
    }

    // Set flywheel speed and hood actuator position
    SetFlywheel(m_state.flywheelSpeed);
    SetActuator(m_state.hoodActuatorInches);
}
#pragma endregion

#pragma region SetFlywheel
/// @brief Spins up the flywheel motor
/// @param input The input value to set the flywheel motor speed
void Tower::SetFlywheel(units::turns_per_second_t input)
{
    // Set the flywheel motor speed
    m_flywheelMotor.SetControl(ctre::phoenix6::controls::VelocityDutyCycle{input});
}
#pragma endregion

#pragma region SetActuator
/// @brief Activates the actuator which moves linearly to move the hood by some degrees, check the manual https://andymark.com/products/linear-servo-actuators
/// @param position The position input value to set the hood actuator 
void Tower::SetActuator(units::inch_t position)
{
    position = std::clamp(position, TowerConstants::MinLength, TowerConstants::MaxLength);

    double actuatorPosition = (position.value() - TowerConstants::MinLength.value()) / TowerConstants::ActuatorDistanceConversionFactor.value();

    // range: 0-1
	actuatorPosition = std::clamp(actuatorPosition, 0.0, 1.0);

    // range: 0-2.0
    actuatorPosition *= 2.0;

    // range: -1, 1
    actuatorPosition -= 1;

    // Although this says SetSpeed, this actually does position
	m_hoodActuator.SetSpeed(std::clamp(actuatorPosition, TowerConstants::ActuatorLowerBound, TowerConstants::ActuatorUpperBound));
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
/// @param relativeDistance The relative distance to the target from the turret to the target
/// @param speed The chassis speeds of the robot relative to the field
TowerState Tower::CalculateShot(frc::Translation2d relativeDistance, frc::ChassisSpeeds speed)
{
    TowerState newState{TowerMode::ManualControl, 0_deg, 0.0_rpm, 0.0_in};

    // Create a new Pose2d from the relative distance and apply speed based translations
    // Predict where the target will be in 0.5 seconds using frc::Twist2d
    // Add that to the distance to target
    frc::Pose2d newRelativeDistance = frc::Pose2d{relativeDistance, 0_deg};
    frc::Twist2d changeInPosition = speed.ToTwist2d(0.5_s); 
    newRelativeDistance = newRelativeDistance.TransformBy(frc::Transform2d{changeInPosition.dx, changeInPosition.dy, changeInPosition.dtheta});

    if (!m_usingTurretCamera)
    {
        newState.turretAngle = 57.2958_deg * std::atan2(newRelativeDistance.Y().value(), newRelativeDistance.X().value());
    }

    // Adjust the angle based on the robot's movement
    newState.turretAngle += newRelativeDistance.Rotation().Degrees();

    // TODO: Implement a real calculation for shot parameters based on distance and speed
    auto distance = std::abs(newRelativeDistance.Translation().Norm().value());
    return newState;
}
#pragma endregion
