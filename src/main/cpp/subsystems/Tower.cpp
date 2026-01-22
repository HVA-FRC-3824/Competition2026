#include "subsystems/Tower.h"

#pragma region Tower
/// @brief Constructor for the Tower subsystem
/// @param poseSupplier 
Tower::Tower(std::function<frc::Pose2d()> poseSupplier) :  m_poseSupplier{poseSupplier}
{
    // Configure the tower motors
    TalonFXConfiguration(&m_turretMotor,
                          TowerConstants::AngleMaximumAmperage,
                          true,
                          TowerConstants::AngleP,
                          TowerConstants::AngleI,
                          TowerConstants::AngleD,
                          0.0,
                          0.0,
                          0.0,
                          0_tps,
                          units::turns_per_second_squared_t{0});

    TalonFXConfiguration(&m_flywheelMotor,
                          TowerConstants::FlywheelMaximumAmperage,
                          true,
                          TowerConstants::FlywheelP,
                          TowerConstants::FlywheelI,
                          TowerConstants::FlywheelD,
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
    auto oldPose               = m_pose.first;
    auto oldTimestamp          = m_pose.second;

    // Update the current pose and timestamp
    m_pose = {m_poseSupplier(), frc::GetTime()};

    // Calculate the speed of the robot based on the change in pose over time
    auto speed = ((m_pose.first - oldPose).Translation() / (m_pose.second - oldTimestamp).value());

    // Does not need to be initialized every cycle, 
    static bool isBlue = frc::DriverStation::GetAlliance().value_or(frc::DriverStation::Alliance::kBlue) == frc::DriverStation::Alliance::kBlue;

    frc::Pose3d Hub = isBlue ? constants::field::blueHub : constants::field::redHub;

    switch (m_state.mode) 
    {
        case TowerMode::Static:
        {
            // Keep turret at 0 degrees relative to robot
            isTurretRobotRelative            = true;
            m_state.turretAngle              = 0_deg;
            m_state.hoodActuatorPercentInput = TowerConstants::constantFlywheelSpeed;
            break;
        }

        case TowerMode::Hub:
        {
            // Aim at the Hub
            isTurretRobotRelative = false;
            m_state = CalculateShot(
                frc::Translation3d{m_pose.first.Translation()}.Distance(Hub.Translation()), 
                speed);

            auto relativeDistance = Hub.ToPose2d().Translation() - m_pose.first.Translation();

            m_state.turretAngle = 57.2958_deg * std::atan2(relativeDistance.Y().value(), relativeDistance.X().value());
            break;
        }

        case TowerMode::Passing:
        {
            // Point straight towards our alliance zone
            isTurretRobotRelative = false;
            // m_state = CalculateShot(frc::Translation2d);
            m_state.turretAngle   = 180_deg; // Gyro SHOULD be 0_deg when facing opposite side, so 180_deg when facing ours
            break;
        }

        default:
            break;
    }

    // Apply the calculated state to the hardware
    if (isTurretRobotRelative)
    {
        SetTurret(m_state.turretAngle);
    } 
    else 
    {
        SetTurret(m_state.turretAngle, m_pose.first.Rotation().Degrees());
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
    // Do not allow turret to move past 360 degrees either way
    angle = (units::degree_t) fmod(angle.value(), 360.0);
    while (angle.value() < 0) 
        angle += 360.0_deg;

    // Convert degrees to rotations (turns) for Phoenix 6
    units::angle::turn_t rotations{angle.value() / 360.0};

    // Set the motor to the desired position
    m_turretMotor.SetControl(ctre::phoenix6::controls::PositionDutyCycle{rotations});
}
#pragma endregion

#pragma region SetTurret
/// @brief Sets the desired angle of the turret relative to the field
/// @param angle The angle in degrees to set the turret to
void Tower::SetTurret(units::degree_t angle, units::degree_t gyroAngle)
{
    // Set the turret relative to the robot's gyro angle
    SetTurret(gyroAngle - angle);
}
#pragma endregion

#pragma region CalculateShot
/// @brief Calculates the optimal TowerState for shooting based on distance and speed   
/// @param distance The distance to the target in meters
/// @param speed The speed of the target or robot
TowerState Tower::CalculateShot(units::meter_t distance, frc::Translation2d speed)
{
    return TowerState{TowerMode::Manual, 0_deg, 0.0, 0.0};
}
#pragma endregion
