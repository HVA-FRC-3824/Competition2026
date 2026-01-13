#pragma once

#pragma region Includes
#include <functional>

#include <frc2/command/SubsystemBase.h>
#include <frc/geometry/Pose2d.h>

#include <units/angle.h>

#include "lib/hardware/motors/TalonFX.h"

#include "Constants.h"
#pragma endregion


class Turret : public frc2::SubsystemBase
{
    public:
        
        Turret();

        // Sets the desired angle of the turret relative to the robot
        void SetAngle(units::degree_t angle);

    private:

        hardware::motor::TalonFX m_angleMotor;

};