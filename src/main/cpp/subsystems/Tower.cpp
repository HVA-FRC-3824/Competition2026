#include "subsystems/Tower.h"

Tower::Tower(std::function<frc::Pose2d()> poseSupplier) :
    m_poseSupplier{poseSupplier}
{
    m_hoodActuator.SetBounds(2.0_us, 1.8_us, 1.5_us, 1.2_us, 1.0_us);
}
 
void Tower::SetState(TowerState newState)
{
    m_state = newState;
}

TowerState Tower::GetState()
{
    return m_state;
}

void Tower::Periodic()
{
    bool isTurretRobotRelative = true;
    auto oldPose      = m_pose.first;
    auto oldTimestamp = m_pose.second;

    m_pose = {m_poseSupplier(), frc::GetTime()};

    auto speed = ((m_pose.first - oldPose).Translation() / (m_pose.second - oldTimestamp).value());

    frc::Pose3d hub = frc::DriverStation::GetAlliance().value_or(frc::DriverStation::Alliance::kBlue) 
         == frc::DriverStation::Alliance::kBlue ? constants::field::blueHub : constants::field::redHub;

    switch (m_state.mode) 
    {
        case TowerMode::STATIC:
            isTurretRobotRelative = true;
            m_state.turretAngle = 0_deg;
            m_state.hoodActuatorPercentInput = constants::tower::constantFlywheelSpeed;
            break;
        case TowerMode::HUB:
            isTurretRobotRelative = false;
            m_state = CalculateShot(
                frc::Translation3d{m_pose.first.Translation()}.Distance(hub.Translation()), 
                speed);
            break;

        case TowerMode::PASSING:
            // We want to point straight towards our alliance zone
            isTurretRobotRelative = false;
            break;

        default:
            break;
    }

    if (isTurretRobotRelative)
    {
        SetTurret(m_state.turretAngle);
    } else 
    {
        SetTurret(m_state.turretAngle, m_pose.first.Rotation().Degrees());
    }
    SetFlywheel(m_state.flyWheelSpeed);
    SetActuator(m_state.hoodActuatorPercentInput);
}

// Spins up the flywheel motor
void Tower::SetFlywheel(double input)
{
    m_flywheelMotor.SetReferenceState(input, hardware::motor::MotorInput::ARBITRARY);
}

// Activates the actuator which moves linearly to move the hood by some degreesInput 0-1
void Tower::SetActuator(double position)
{
    // range: 0-1
	position = std::clamp(position, 0.0, 1.0);

    // range: 0-2.0
    position *= 2.0;

    // range: -1, 1
    position -= 1;

    // Although this says SetSpeed, this actually does position
	m_hoodActuator.SetSpeed(std::clamp(position, constants::tower::ActuatorLowerBound, constants::tower::ActuatorUpperBound));
}

// Sets the desired angle of the turret relative to the robot
void Tower::SetTurret(units::degree_t angle)
{
    // Do not allow turret to move past 360 degrees either way
    angle = (units::degree_t) fmod(angle.value(), 360.0);
    while (angle.value() < 0) 
        angle += 360.0_deg;

    m_turretMotor.SetReferenceState(angle.value() / 360, hardware::motor::MotorInput::POSITION);
}

// Sets the desired angle of the turret relative to the field
void Tower::SetTurret(units::degree_t angle, units::degree_t gyroAngle)
{
    SetTurret(gyroAngle - angle);
}

// We're doing this via a polynomial that I dont think has been calculated yet
TowerState Tower::CalculateShot(units::meter_t distance, frc::Translation2d speed)
{
    return TowerState{TowerMode::MANUAL, 0_deg, 0.0, 0.0};
}