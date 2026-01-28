#include "subsystems/Tower.h"

#pragma region Tower
/// @brief Constructor for the Tower subsystem
/// @param chassisPoseSupplier Function that supplies the current chassis pose
/// @param chassisSpeedsSupplier Function that supplies the current chassis speeds
Tower::Tower(std::function<frc::Pose2d()> chassisPoseSupplier, std::function<frc::ChassisSpeeds()> chassisSpeedsSupplier) :
    m_chassisPoseSupplier  {chassisPoseSupplier},
    m_chassisSpeedsSupplier{chassisSpeedsSupplier}
{
    // Configure the tower motors
    TalonFXConfiguration(&m_turretMotor,
                          40_A,                                  // Current limit
                          true,                                  // Brake mode
                          false,                                 // Continuous wrap
                          0.0,                                   // P gain
                          0.0,                                   // I gain
                          0.0,                                   // D gain
                          0.0,                                   // S (static friction feedforward)
                          0.0,                                   // V (velocity feedforward)
                          0.0,                                   // A (acceleration feedforward)
                          0_tps,                                 // Velocity limit
                          units::turns_per_second_squared_t{0}); // Acceleration limit

    TalonFXConfiguration(&m_flywheelMotor,
                          40_A,                                  // Current limit
                          true,                                  // Brake mode
                          false,                                 // Continuous wrap
                          0.0,                                   // P gain
                          0.0,                                   // I gain
                          0.0,                                   // D gain
                          0.0,                                   // S (static friction feedforward)
                          0.0,                                   // V (velocity feedforward)
                          0.0,                                   // A (acceleration feedforward)
                          0_tps,                                 // Velocity limit
                          units::turns_per_second_squared_t{0}); // Acceleration limit

    // Initialize the pose with the current pose and timestamp
    m_hoodActuator.SetBounds(2.0_us, 1.8_us, 1.5_us, 1.2_us, 1.0_us);
    
    frc::SmartDashboard::PutData("Tower", &m_logMechanism);
}
#pragma endregion

#pragma region SetState
/// @brief Sets the current state of the Tower subsystem
/// @param newState The new state to set for the Tower subsystem
void Tower::SetState(TowerState newState)
{
    // // Don't do anything new if nothing new has happened
    // if (m_state.mode == newState.mode && 
    //     m_state.turretAngle        == newState.turretAngle        && 
    //     m_state.hoodActuatorInches == newState.hoodActuatorInches && 
    //     m_state.flywheelSpeed      == newState.flywheelSpeed)
    // {
    //     return;
    // }

    // Remember the state
    m_state = newState;

    // Log the new state
    Log("Tower", std::string_view{"Setting tower state to " + std::to_string(static_cast<int>(m_state.mode))});
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

    // range: 2, 1
    actuatorPosition += 1;

    // Although this says SetSpeed, this actually does position
	m_hoodActuator.SetSpeed(actuatorPosition);
}
#pragma endregion

#pragma region SetTurretAngle
/// @brief Sets the desired angle of the turret relative to the robot
/// @param angle The angle in degrees to set the turret to
void Tower::SetTurretAngle(units::degree_t angle)
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

#pragma region GetTurretAngle
/// @brief Method to get the Turret angle.
/// @return The current turret angle in degrees
units::degree_t Tower::GetTurretAngle()
{
    // Get the motor position in turns
    units::angle::turn_t turns = m_turretMotor.GetPosition().GetValue();

    // Convert turns to degrees (1 turn = 360 degrees)
    units::degree_t degrees = turns.convert<units::degrees>();
    
    // Return the current turret angle
    return degrees;
}
#pragma endregion

#pragma region CalculateShot
/// @brief Changes the turret angle, flywheel speed, and hood actuator position based on distance and speed to target
/// @param relativeDistance The relative distance to the target from the turret to the target
/// @param speed The chassis speeds of the robot relative to the field
TowerState Tower::CalculateShot(TowerMode towerMode, frc::Translation2d relativeDistance, frc::ChassisSpeeds speed)
{
    TowerState newState{towerMode, 0_deg, 0.0_rpm, 0.0_in};

    // Create a new Pose2d from the relative distance and apply speed based translations
    // Predict where the target will be in 0.5 seconds using frc::Twist2d
    // Add that to the distance to target
    frc::Pose2d  newRelativeDistance = frc::Pose2d{relativeDistance, 0_deg};
    frc::Twist2d changeInPosition    = speed.ToTwist2d(0.5_s); 
    newRelativeDistance = newRelativeDistance.TransformBy(frc::Transform2d{changeInPosition.dx, changeInPosition.dy, changeInPosition.dtheta});

    // Calculate turret angle
    if (m_usingTurretCamera)
    {
        // Adjust turret angle based on target yaw                  
        newState.turretAngle = 0.0_deg;
    }
    else
    {
        // Adjust turret angle based on predicted target position
        newState.turretAngle = 57.2958_deg * std::atan2(newRelativeDistance.Y().value(), newRelativeDistance.X().value());
    }

    // Adjust the angle based on the robot's movement
    newState.turretAngle += newRelativeDistance.Rotation().Degrees();

    auto distance = std::abs(newRelativeDistance.Translation().Norm().value());

    // Calculate hood actuator position based on distance
    newState.hoodActuatorInches = std::clamp(units::inch_t{0.1_in + (0.05_in * distance)}, TowerConstants::MinLength, TowerConstants::MaxLength);

    // Calculate the flywheel speed based on distance
    newState.flywheelSpeed = units::turns_per_second_t{(0.5_tps * distance) + 5.0_tps};

    // Return the new calculated state
    return newState;
}
#pragma endregion

#pragma region Periodic
/// @brief Periodic method for the Tower subsystem, called periodically by the CommandScheduler
///
///        Camera             AprilTag 
///
///           ^ X                |
///           |                  |
///           |                  |
///   Y <-----+------      ------+-----> Y
///           |                  |
///           |                  |
///           |                  V X
///   Z - up
///
void Tower::Periodic()
{
    // Update the chassis current pose and speed
    auto chassisPose  = m_chassisPoseSupplier();
    auto chassisSpeed = m_chassisSpeedsSupplier();
	
    // TODO: add actual Mechanism2d representation to compare against the setpoints here

    // TODO: For bench testing
    m_state.mode = TowerMode::ShootingToHub;
    
    switch (m_state.mode) 
    {
        case TowerMode::Idle:
        {
            // Do not power down flywheel, do not move turret, do not move hood, wait until further inputs 
            return;
        }

        case TowerMode::ShootingToHub:
        {
            frc::Translation2d relativeDistance;

            // When using the turret camera, relative distance is based on the turret
            if (m_usingTurretCamera)
            {
                photon::PhotonPipelineResult result = m_turretCamera.GetLatestResult();

                frc::SmartDashboard::PutBoolean("Has Targets", result.HasTargets());

                if (result.HasTargets())
                {
                    // Get a list of currently tracked targets.
                    for (auto target : result.GetTargets())
                    {
                        frc::SmartDashboard::PutNumber("ID",    target.fiducialId);
        
                        // Camera offset angles (small values - how far target is from camera center)
                        frc::SmartDashboard::PutNumber("Camera Offset Skew",  target.GetSkew());
                        frc::SmartDashboard::PutNumber("Camera Offset Pitch", target.GetPitch());
                        frc::SmartDashboard::PutNumber("Camera Offset Yaw",   target.GetYaw());
        
                        // Extract the x and y distances to the target
                        frc::Transform3d tracketTarget = target.GetBestCameraToTarget();
                        frc::SmartDashboard::PutNumber("Distance X", tracketTarget.X().value());
                        frc::SmartDashboard::PutNumber("Distance Y", tracketTarget.Y().value());
                        frc::SmartDashboard::PutNumber("Distance Z", tracketTarget.Z().value());
        
                        // Target orientation in space (what PhotonVision UI shows)
                        auto rotation = tracketTarget.Rotation();
                        frc::SmartDashboard::PutNumber("Target Roll (X)",  rotation.X().convert<units::degrees>().value());
                        frc::SmartDashboard::PutNumber("Target Pitch (Y)", rotation.Y().convert<units::degrees>().value());
                        frc::SmartDashboard::PutNumber("Target Yaw (Z)",   rotation.Z().convert<units::degrees>().value());
                    
                        // Additional debug info
                        frc::SmartDashboard::PutNumber("Area",      target.GetArea());
                        frc::SmartDashboard::PutNumber("Ambiguity", target.GetPoseAmbiguity());
        
                        // Translate the actual target to be behind the AprilTag
                        // The AprilTag's X-axis points out from the tag, so we translate along -X to go "behind" it
                        frc::Transform3d offsetToHub{frc::Translation3d{-23.5_in, 0_m, 0_m}, frc::Rotation3d{}};
                        frc::Transform3d cameraToHub = tracketTarget + offsetToHub;
        
                        // Get the 2D distance to the actual hub target
                        frc::Translation2d hubDistance = cameraToHub.Translation().ToTranslation2d();
                        frc::SmartDashboard::PutNumber("Hub Distance X",    hubDistance.X().value());
                        frc::SmartDashboard::PutNumber("Hub Distance Y",    hubDistance.Y().value());
                        frc::SmartDashboard::PutNumber("Hub Distance Norm", hubDistance.Norm().value());
        
                        // Calculate the turret angle needed to aim at the hub
                        // atan2(Y, X) gives the angle from turret centerline to the hub
                        units::degree_t turretAngle = units::math::atan2(hubDistance.Y(), hubDistance.X());
                        frc::SmartDashboard::PutNumber("Turret Angle to Hub (deg)", turretAngle.value());
        
                        // Rotate the turret
                        m_state.turretAngle = turretAngle - GetTurretAngle();

                        // Calculate the shot parameters based on the hub distance and chassis speed
                        m_state = CalculateShot(TowerMode::ShootingToHub, hubDistance, chassisSpeed);
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
                
                // Calculate the relative distance from the turret center to the hub
                relativeDistance = Hub.ToPose2d().Translation() - (chassisPose.Translation() + TowerConstants::OffsetTurretFromRobotCenter.Translation().ToTranslation2d());
            }

            // Calculate the shot parameters based on the relative distance and chassis speed
            m_state = CalculateShot(TowerMode::ShootingToHub, relativeDistance, chassisSpeed);
            break;
        }

        case TowerMode::PassingToAdjacentZone:
        {
            // Based on alliance color, find the nearest point in our alliance zone to pass to
            // We decide between the close and the far points so that we can avoid hitting the hub net
            auto targetPoint = chassisPose.Translation().Nearest({m_isBlue ? constants::field::blueAllianceZoneClose.Translation() : constants::field::redAllianceZoneClose.Translation(),
                                                                  m_isBlue ? constants::field::blueAllianceZoneFar.Translation()   : constants::field::redAllianceZoneFar.Translation()});
            
            auto relativeDistance = targetPoint - chassisPose.Translation();

            m_state.turretAngle = 57.2958_deg * std::atan2(relativeDistance.Y().value(), relativeDistance.X().value());

            m_state = CalculateShot(TowerMode::PassingToAdjacentZone, relativeDistance, chassisSpeed);
            break;
        }

        default:
        {
            break;
        }
    }

    // Apply the calculated state to the hardware
    if (m_usingTurretCamera)
    {
        SetTurretAngle(m_state.turretAngle);
    } 
    else 
    {
        // Compensate for robot rotation
        // If we are on red, our gyro will be turned around, flip it
        SetTurretAngle((chassisPose.Rotation().Degrees() - (m_isBlue ? 0_deg : 180_deg)) - m_state.turretAngle);
    }

    // Set flywheel speed and hood actuator position
    SetFlywheel(m_state.flywheelSpeed);
    SetActuator(m_state.hoodActuatorInches);

    /// *** Update logging *** ///

    Log("Hood Length ", m_state.hoodActuatorInches.value());
    Log("Flywheel Speed ", m_state.flywheelSpeed.value());
    Log("Turret Angle ", m_state.turretAngle.value());

    // Set the hood representation
    m_logHoodFlywheel->SetAngle(std::clamp(m_state.hoodActuatorInches.value(), TowerConstants::MinLength.value(), TowerConstants::MaxLength.value()) * 1_deg);
    
    // Set the flywheel representation
    // I assume that the flywheelSpeed will be 7000-3000 rpm
    m_logHoodFlywheel->SetLength((m_state.flywheelSpeed.value() / 1000));

    // Set the turret angle representation
    m_logTurret->SetAngle(m_state.turretAngle);
}
#pragma endregion
