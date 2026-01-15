#include "subsystems/Turret.h"

void Turret::SetState(TurretState newState)
{
    // TODO: Either change the way calculations are done for aiming
    // For this we'll need a pose supplier to know which way the turret
    // should be facing irrespective of just the hub tags.

    switch (newState.mode) 
    {
        case TurretMode::STATIC:
            SetTurret(0_deg);
            break;
        case TurretMode::MANUAL:
            SetTurret(newState.robotRelativeAngle);
            break;
        case TurretMode::HUB:{
            std::vector<photon::PhotonPipelineResult> results = m_turretCam.GetAllUnreadResults();
            
            if (!results.empty())
            {
                auto latestResult = results.back();
                for (auto tag : latestResult.GetTargets())
                {
                    // TODO: verify if these are right they might be the ones next to these
                    if (tag.fiducialId == 10 || tag.fiducialId == 26)
                    {
                        // TODO: remove magic numbers
                        m_state.robotRelativeAngle = m_state.robotRelativeAngle - (units::degree_t) tag.altCameraToTarget.Rotation().Angle().value() * 57.2958;
                    }
                }
            }
            }
            break;
        case TurretMode::PASSING:
            // We want to point straight towards our alliance zone
            break;
        default:
            break;
    }
}

// Sets the desired angle of the turret relative to the robot
void Turret::SetTurret(units::degree_t angle)
{
    // Do not allow turret to move past 360 degrees either way
    angle = (units::degree_t) fmod(angle.value(), 360.0);
    if (angle.value() < 0) {
        angle += 360.0_deg;
    }

    m_turretMotor.SetReferenceState(units::turn_t{angle});
}