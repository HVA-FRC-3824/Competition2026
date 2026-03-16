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
                         40_A,  // Current limit
                         true,  // Inverted
                         false, // Brake mode
                         false, // Continuous wrap
                         1.0,  // P gain
                         0.35,  // I gain
                         0.05,  // D gain
                         0.0,   // S (static friction feedforward)
                         0.0,   // V (velocity feedforward)
                         0.0,   // A (acceleration feedforward)
                         12_tps, // Velocity limit
                         12_tr_per_s_sq); // Acceleration limit

    TalonFXConfiguration(&m_flywheelMotor,
                         40_A,            // Current limit
                         true,            // Inverted
                         false,           // Brake mode
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
    m_hoodActuator.SetBounds(2.0_ms, 1.8_ms, 1.5_ms, 1.2_ms, 1.0_ms);
}
#pragma endregion

#pragma region SetState
/// @brief Sets the current state of the Tower subsystem
/// @param newState The new state to set for the Tower subsystem
void Tower::SetState(TowerState newState)
{
    // Set the second follower motor to be the *inverse* of the other flywheel motor
    m_flywheelFollowerMotor.SetControl(
        ctre::phoenix6::controls::Follower(m_flywheelMotor.GetDeviceID(), 
                                        ctre::phoenix6::signals::MotorAlignmentValue::Opposed)
    );

    Log("Set State Turret", newState.turretAngle.value());
    Log("Set State Flywheel", newState.flywheelSpeed.value());
    Log("Set State Actuator", newState.hoodActuatorDistance);
    
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
    auto flywheelErrorTolerance = m_flywheelMotor.GetClosedLoopReference().GetValueAsDouble() * TowerConstants::TargetTolerance + TowerConstants::TargetTolerance;
    auto isSpunUp = std::abs(flywheelPidError) < flywheelErrorTolerance;

    auto turretPidError = m_turretMotor.GetClosedLoopError().GetValueAsDouble();
    auto turretErrorTolerance = m_turretMotor.GetClosedLoopReference().GetValueAsDouble() * TowerConstants::TargetTolerance + TowerConstants::TargetTolerance;
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
    if (input == 0_tps)
        m_flywheelMotor.SetControl(ctre::phoenix6::controls::VoltageOut{0_V});


    // Set the flywheel motor speed
    m_flywheelMotor.SetControl(ctre::phoenix6::controls::VelocityVoltage{input});
}
#pragma endregion

#pragma region SetActuator
/// @brief Activates the actuator which moves linearly to move the hood by some degrees, check the manual https://andymark.com/products/linear-servo-actuators
/// @param position The position input value to set the hood actuator 0,1
void Tower::SetActuator(double position)
{
    //0,1
    position;

    //-0.5,0.5
    position -= 0.5;

    // -0.95, 0.95
    position *= 1.9;

    position = std::clamp(position, -0.95, 0.95);

    // Although this says SetSpeed, this actually does position
	m_hoodActuator.SetSpeed(position);
}
#pragma endregion

#pragma region SetTurretAngle
/// @brief Sets the desired angle of the turret relative to the robot
/// @param angle The angle in degrees to set the turret to
void Tower::SetTurretAngle(units::degree_t angle)
{

    angle += 180_deg + m_turretOffset;
    // Normalize to [0, 360), then shift into [MinAngle, MinAngle + 360)
    double normalized = std::fmod(angle.value() - TowerConstants::MinAngle.value(), 360.0);
    if (normalized < 0) normalized += 360.0;
    angle = TowerConstants::MinAngle + 1_deg * normalized;

    angle = std::clamp(angle, TowerConstants::MinAngle, TowerConstants::MaxAngle);
    
    Log("turret real setpoint ", angle.value());

    // Convert degrees to rotations (turns) for TalonFX
    units::angle::turn_t rotations{angle.value() / 360.0};

    // Set the motor to the desired position
    m_turretMotor.SetControl(ctre::phoenix6::controls::MotionMagicVoltage{rotations * TowerConstants::TurretGearReduction});
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
    units::degree_t degrees{turns.value() * 360};
    
    // Return the current turret angle
    return degrees;
}
#pragma endregion

#pragma region CalculateShot
/// @brief Changes the turret angle, flywheel speed, and hood actuator position based on distance and speed to target
/// @param targetFieldPos The position of the target in field coordinates
/// @param speed The chassis speeds of the robot relative to the robot itself
/// @param chassisPose The current pose of the chassis in field coordinates
TowerState Tower::CalculateShot(TowerMode towerMode, frc::Translation2d targetFieldPos, frc::ChassisSpeeds speed, frc::Pose2d chassisPose)
{
    TowerState newState{towerMode, 0_deg, 0.0_rpm, 0.0};

    // Predict where the chassis will be in 0.5 seconds
    frc::Twist2d changeInPosition = speed.ToTwist2d(0.5_s);
    frc::Pose2d futureChassisPose = chassisPose.Exp(changeInPosition);
    
    // Calculate where the turret will be in field coordinates in 0.5s
    // The offset is robot relative, so we rotate it by the future chassis rotation
    frc::Translation2d turretRobotOffset = TowerConstants::OffsetTurretFromRobotCenter.Translation().ToTranslation2d();
    frc::Translation2d futureTurretPos = futureChassisPose.Translation() + turretRobotOffset.RotateBy(futureChassisPose.Rotation());
    
    // The vector from the future turret to the field target
    frc::Translation2d futureTurretToTarget = targetFieldPos - futureTurretPos;

    if (m_usingTurretCamera)
    {
        // Adjust turret angle based on target yaw                  
        newState.turretAngle = 0.0_deg; // Handled outside in Periodic
    }
    else
    {
        // Adjust turret angle based on predicted target position
        units::degree_t angleToTargetInField = units::math::atan2(futureTurretToTarget.Y(), futureTurretToTarget.X());
        newState.turretAngle = angleToTargetInField - futureChassisPose.Rotation().Degrees();
        Log("desired turret angle ", newState.turretAngle.value());
    }

    auto distance = futureTurretToTarget.Norm();
    Log("Measured Distance", distance.value());

    // Handle single or empty vector
    if (TowerConstants::knownDataPoints.empty()) 
    {
        return m_state;
    }
    else if (TowerConstants::knownDataPoints.size() == 1) 
    {
        const auto &[dist, flywheel, hood] = TowerConstants::knownDataPoints[0];
        
        m_state.flywheelSpeed        = flywheel;
        m_state.hoodActuatorDistance = hood;
    }

    // Handle edge cases (distance below minimum or above maximum)
    else if (distance <= std::get<0>(TowerConstants::knownDataPoints.front())) 
    {
        const auto& [dist, flywheel, hood] = TowerConstants::knownDataPoints.front();
        m_state.flywheelSpeed = flywheel;
        m_state.hoodActuatorDistance = hood;
    } 
    else if (distance >= std::get<0>(TowerConstants::knownDataPoints.back())) 
    {
        const auto& [dist, flywheel, hood] = TowerConstants::knownDataPoints.back();
        m_state.flywheelSpeed = flywheel;
        m_state.hoodActuatorDistance = hood;
    }

    // Linearly deduce partial data of a third point by the full data of two adjacent points
    else
    {
        for (int i = 0; i < TowerConstants::knownDataPoints.size() - 1; i++)
        {
            const auto &[lowerDist, lowerFlywheel, lowerHood]    = TowerConstants::knownDataPoints[i];
            const auto &[higherDist, higherFlywheel, higherHood] = TowerConstants::knownDataPoints[i + 1];
            
            if (lowerDist <= distance && distance <= higherDist)
            {
                // Calculate interpolation ratio
                auto ratio = (distance - lowerDist) / (higherDist - lowerDist);
                
                // Linear interpolation
                m_state.flywheelSpeed        = lowerFlywheel + (ratio * (higherFlywheel - lowerFlywheel));
                m_state.hoodActuatorDistance = lowerHood     + (ratio * (higherHood - lowerHood));
                
                break;
            }
        }
    }
    
    // Return the new calculated state
    return newState;
}
#pragma endregion

#define isActiveShift() ([]() -> bool {                                   \
    auto alliance = frc::DriverStation::GetAlliance();                    \
    /* If we have no alliance, we cannot be enabled, therefore no hub. */ \
    if (!alliance) {                                                      \
      return false;                                                       \
    }                                                                     \
    /* Hub is always enabled in autonomous. */                            \
    if (frc::DriverStation::IsAutonomousEnabled()) {                      \
      return true;                                                        \
    }                                                                     \
    /* At this point, if we're not teleop enabled, there is no hub. */    \
    if (!frc::DriverStation::IsTeleopEnabled()) {                         \
      return false;                                                       \
    }                                                                     \
    /* We're teleop enabled, compute. */                                  \
    auto matchTime = frc::DriverStation::GetMatchTime();                  \
    auto gameData  = frc::DriverStation::GetGameSpecificMessage();        \
    /* If we have no game data, we cannot compute, assume hub is active,  \
    as its likely early in teleop.                                        \
     */                                                                   \
    if (gameData.empty()) {                                               \
      return true;                                                        \
    }                                                                     \
    bool redInactiveFirst = !(gameData.at(0) == 'B');                     \
    /* Shift was is active for blue if red won auto, or red if blue won auto. */ \
    bool shift1Active = (alliance.value() == frc::DriverStation::Alliance::kRed) ? !redInactiveFirst : redInactiveFirst; \
    if (matchTime > 130.0_s) {                                            \
      /* Transition shift, hub is active. */                              \
      return true;                                                        \
    } else if (matchTime > 105.0_s) {                                     \
      /* Shift 1 */                                                       \
      return shift1Active;                                                \
    } else if (matchTime > 80.0_s) {                                      \
      /* Shift 2 */                                                       \
      return !shift1Active;                                               \
    } else if (matchTime > 55.0_s) {                                      \
      /* Shift 3 */                                                       \
      return shift1Active;                                                \
    } else if (matchTime > 30.0_s) {                                      \
      /* Shift 4 */                                                       \
      return !shift1Active;                                               \
    } else {                                                              \
      /* End game, hub always active.  */                                 \
      return true;                                                        \
    }                                                                     \
})()

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
    Log("Turret Mode", m_state.mode);

    // Update the chassis current pose and speed
    auto chassisPose  = m_chassisPoseSupplier();
    auto chassisSpeed = m_chassisSpeedsSupplier();
    
    Log("Is blue", m_isBlue);

    // We'll assign the state based on our location. 
    // After the calculations, we'll reassign the state to Automatic for the next cycle
    bool isAutomatic = m_state.mode == TowerMode::Automatic;
    if (isAutomatic)
    {
        // If were in our alliance zone, score fuel to our alliance zone
        if (m_isBlue ? chassisPose.X() < constants::Field::AllianceWallToAllianceZone :
                       constants::Field::FieldLength - constants::Field::AllianceWallToAllianceZone < chassisPose.X()
            &&
            isActiveShift())
        {
            m_state.mode = TowerMode::ShootingToHub;
        }
        else
        {
            // Otherwise shoot to the hub
            m_state.mode = TowerMode::PassingToAdjacentZone;
        }
    }

    if (m_state.mode == TowerMode::Idle)
    {
        m_state.turretAngle          = 0_deg;
        m_state.flywheelSpeed        = 0_rpm;
        m_state.hoodActuatorDistance = 0;
    }
    else if (m_state.mode != TowerMode::ManualControl)
    {
        auto targetFieldPos = m_state.mode == TowerMode::ShootingToHub ? 
            (m_isBlue ? constants::Field::BlueHub : constants::Field::RedHub).ToPose2d().Translation()
            :
            chassisPose.Translation().Nearest({
                m_isBlue ? constants::Field::BlueAllianceZoneClose.Translation() : constants::Field::RedAllianceZoneClose.Translation(), 
                m_isBlue ? constants::Field::BlueAllianceZoneFar.Translation()   : constants::Field::RedAllianceZoneFar.Translation()});

        m_state = CalculateShot(m_state.mode, targetFieldPos, chassisSpeed, chassisPose);
    }
    
    SetTurretAngle(m_state.turretAngle);
    SetFlywheel(m_state.flywheelSpeed);
    SetActuator(m_state.hoodActuatorDistance);
    
    // If its in automatic mode, prepare the state for the next cycle
    if (isAutomatic)
        m_state.mode = TowerMode::Automatic;

    Log("Desired Hood Length ", m_state.hoodActuatorDistance);
    Log("Desired Flywheel Speed ", m_state.flywheelSpeed.value());
    Log("Desired Turret Angle ", m_state.turretAngle.value());

    // Log("Measured Hood Length ", ); TODO: simulate hood speeds
    Log("Measured Flywheel Speed ", m_flywheelMotor.GetVelocity().GetValueAsDouble());
    Log("Measured Turret Angle ", GetTurretAngle().value());
}
#pragma endregion
