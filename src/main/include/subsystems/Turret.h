#pragma once

#pragma region Includes
#include <units/angle.h>

#include <frc2/command/SubsystemBase.h>

#include "lib/hardware/motors/TalonFX.h"

#include <photon/PhotonCamera.h>

#include "Constants.h"
#pragma endregion

#pragma region StateStructs
enum TurretMode
{
    HUB,
    PASSING,
    STATIC,
    MANUAL
};

struct TurretState
{
    TurretMode mode;

    units::degree_t robotRelativeAngle;
};
#pragma endregion

class Turret : public frc2::SubsystemBase
{
    public:

        explicit Turret();

        void        SetState(TurretState newState);

        TurretState GetState();

    private:
    
        // Sets the desired angle of the turret relative to the robot
        void SetTurret(units::degree_t angle);
        
        hardware::motor::TalonFX m_turretMotor{constants::tower::turretMotorID, constants::tower::turretConfig};

        // Do not use this to do pose estimatation. Because its on a turret, it is unreliable
        photon::PhotonCamera     m_turretCam{"turretCam"};

        TurretState m_state{TurretMode::STATIC, 0_deg};
};