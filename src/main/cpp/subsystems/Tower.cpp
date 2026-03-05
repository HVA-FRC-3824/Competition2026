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
                         40_A,            // Current limit
                         true,            // Inverted
                         true,            // Brake mode
                         false,           // Continuous wrap
                         0.0,             // P gain
                         0.0,             // I gain
                         0.0,             // D gain
                         0.0,             // S (static friction feedforward)
                         0.0,             // V (velocity feedforward)
                         0.0,             // A (acceleration feedforward)
                         0.5_tps * TowerConstants::TurretGearReduction, // Velocity limit
                         4_tr_per_s_sq);  // Acceleration limit

    TalonFXConfiguration(&m_flywheelMotor,
                         40_A,            // Current limit
                         true,            // Inverted
                         false,            // Brake mode
                         false,           // Continuous wrap
                         0.32,            // P gain
                         0.0,             // I gain
                         0.0,             // D gain
                         0.0,             // S (static friction feedforward)
                         0.13,            // V (velocity feedforward)
                         1.61,            // A (acceleration feedforward)
                         0_tps,           // Velocity limit
                         0_tr_per_s_sq);  // Acceleration limit

        // Set the second follower motor to be the *inverse* of the other flywheel motor
        m_flywheelFollowerMotor.SetControl(
            ctre::phoenix6::controls::Follower(m_flywheelMotor.GetDeviceID(), 
                                               ctre::phoenix6::signals::MotorAlignmentValue::Opposed)
        );

    // Initialize the pose with the current pose and timestamp
    m_hoodActuator.SetBounds(2.0_us, 1.8_us, 1.5_us, 1.2_us, 1.0_us);

    m_turretMotor.SetPosition(0.0_tr);
    
    frc::SmartDashboard::PutData("Tower", &m_logMechanism);
}
#pragma endregion

#pragma region SetState
/// @brief Sets the current state of the Tower subsystem
/// @param newState The new state to set for the Tower subsystem
void Tower::SetState(TowerState newState)
{
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

#pragma region IsOnTarget
/// @brief Checks if the flywheel is spun up to the desired speed within tolerance
/// @return true if the flywheel is spun up, false otherwise
bool Tower::IsOnTarget()
{
    auto flywheelPidError = m_flywheelMotor.GetClosedLoopError().GetValueAsDouble();
    auto flywheelErrorTolerance = m_flywheelMotor.GetClosedLoopReference().GetValueAsDouble() * TowerConstants::TargetTolerance;
    auto isSpunUp = std::abs(flywheelPidError) < flywheelErrorTolerance;

    auto turretPidError = m_turretMotor.GetClosedLoopError().GetValueAsDouble();
    auto turretErrorTolerance = m_turretMotor.GetClosedLoopReference().GetValueAsDouble() * TowerConstants::TargetTolerance;
    auto isAimed = std::abs(turretPidError) < turretErrorTolerance;

    // Return whether the PID error is within the tolerance
    return isSpunUp && isAimed;
}
#pragma endregion

#pragma region SetFlywheel
/// @brief Spins up the flywheel motor
/// @param input The input value to set the flywheel motor speed
void Tower::SetFlywheel(units::turns_per_second_t input)
{
    // Set the flywheel motor speed
    m_flywheelMotor.SetControl(ctre::phoenix6::controls::VelocityVoltage{input});
    // Set the second follower motor to be the *inverse* of the other flywheel motor
    m_flywheelFollowerMotor.SetControl(
        ctre::phoenix6::controls::Follower(m_flywheelMotor.GetDeviceID(), 
                                        ctre::phoenix6::signals::MotorAlignmentValue::Opposed)
    );
}
#pragma endregion

#pragma region SetActuator
/// @brief Activates the actuator which moves linearly to move the hood by some degrees, check the manual https://andymark.com/products/linear-servo-actuators
/// @param position The position input value to set the hood actuator 
void Tower::SetActuator(units::inch_t position)
{
    // Do not allow actuator to move past the min or max lengths
    position = std::clamp(position, TowerConstants::MinLength, TowerConstants::MaxLength);

    // Convert position in inches to actuator position (0-1)
    double actuatorPosition = (position.value() - TowerConstants::MinLength.value()) / TowerConstants::ActuatorDistanceConversionFactor.value();
	actuatorPosition = std::clamp(actuatorPosition, 0.0, 1.0);

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

    angle = 1_deg * std::clamp(angle.value(), TowerConstants::MinAngle.value(), TowerConstants::MaxAngle.value());

    // Convert degrees to rotations (turns) for TalonFX
    units::angle::turn_t rotations{angle.value() / 360.0};

    // Set the motor to the desired position
    m_turretMotor.SetControl(ctre::phoenix6::controls::PositionVoltage{rotations * TowerConstants::TurretGearReduction});
}
#pragma endregion

#pragma region GetTurretAngle
/// @brief Method to get the Turret angle.
/// @return The current turret angle in degrees
units::degree_t Tower::GetTurretAngle()
{
    // Get the motor position in turns
    units::angle::turn_t turns = m_turretMotor.GetPosition().GetValue() / TowerConstants::TurretGearReduction;

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

    units::meter_t distance = units::meter_t{std::abs(newRelativeDistance.Translation().Norm().value())};

    // TODO: remove after testing
    distance = 20_m;

    // Calculate hood actuator position based on distance
    newState.hoodActuatorInches = 1_in * CalculatePolynomial(distance, TowerConstants::HoodA, TowerConstants::HoodB, TowerConstants::HoodC);
    // newState.hoodActuatorInches = std::clamp(newState.hoodActuatorInches, TowerConstants::MinLength, TowerConstants::MaxLength);

    // Calculate the flywheel speed based on distance
    newState.flywheelSpeed = 1_rpm * CalculatePolynomial(distance, TowerConstants::FlywheelA, TowerConstants::FlywheelB, TowerConstants::FlywheelC);

    // Return the new calculated state
    return newState;
}
#pragma endregion

#pragma region CalculatePolynomial
/// @brief  Calculates a polynomial value based on the given distance and coefficients
/// @param distance The distance value in meters to the HUD
/// @param a The a coefficient of the polynomial
/// @param b The b coefficient of the polynomial
/// @param c The c coefficient of the polynomial
/// @return The calculated polynomial value
double Tower::CalculatePolynomial(units::meter_t distance, double a, double b, double c)
{
    // Calculate the polynomial value
    return a * std::pow(distance.value(), 2) + b * distance.value() + c;
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
    frc::SmartDashboard::PutNumber("Turret Mode", m_state.mode);

    // Update the chassis current pose and speed
    auto chassisPose  = m_chassisPoseSupplier();
    auto chassisSpeed = m_chassisSpeedsSupplier();

    // We'll assign the state based on our location. 
    // After the calculations, we'll reassign the state to Automatic for the next cycle
    bool isAutomatic = m_state.mode == TowerMode::Automatic;
    if (isAutomatic)
    {
        // If were in the neutral zone (or opposing zone), pass fuel to our alliance zone
        if (m_isBlue ? chassisPose.X() < constants::Field::AllianceWallToAllianceZone :
                       chassisPose.X() < constants::Field::FieldLength - constants::Field::AllianceWallToAllianceZone)
        {
            m_state.mode = TowerMode::PassingToAdjacentZone;
        }
        // Otherwise shoot to the hub
        else
        {
            m_state.mode = TowerMode::ShootingToHub;
        }
    }

    switch (m_state.mode) 
    {
        case TowerMode::Idle:
        {
            // Do not power down flywheel, do not move turret, do not move hood, wait until further inputs
            m_state.flywheelSpeed      = 0_rpm;
            m_state.hoodActuatorInches = 0_in;
            m_state.turretAngle        = 0_deg;
            break;
        }

        case TowerMode::ShootingToHub:
        {
            frc::Translation2d relativeDistance;

            // When using the turret camera, relative distance is based on the turret
            if (m_usingTurretCamera)
            {
                std::vector<photon::PhotonPipelineResult> results = m_turretCamera.GetAllUnreadResults();

                photon::PhotonPipelineResult result;
                if (results.empty())
                {
                    // There are no results
                    return;
                }
                else
                {
                    result = results.back();
                }

                if (result.HasTargets())
                {
                    // Get a list of currently tracked targets.
                    for (auto target : result.GetTargets())
                    {
                        frc::SmartDashboard::PutNumber("ID", target.fiducialId);
        
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
                frc::Pose3d Hub = m_isBlue ? constants::Field::BlueHub : constants::Field::RedHub;
                
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
            auto targetPoint = chassisPose.Translation().Nearest({m_isBlue ? constants::Field::BlueAllianceZoneClose.Translation() : constants::Field::RedAllianceZoneClose.Translation(),
                                                                  m_isBlue ? constants::Field::BlueAllianceZoneFar.Translation()   : constants::Field::RedAllianceZoneFar.Translation()});
            
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

    // If its in automatic mode, prepare the state for the next cycle
    if (isAutomatic)
        m_state.mode = TowerMode::Automatic;
}
#pragma endregion
