#pragma once

#pragma region Includes
#include <functional>

#include <frc2/command/Subsystem.h>
#include <frc/geometry/Pose2d.h>

#include "lib/hardware/motors/TalonFX.h"

#include "Constants.h"
#pragma endregion


class Flywheel : public frc2::Subsystem
{
    public:
        
        Flywheel();

        // Starts to spin up the flywheel motor
        // This may need to be a -1,1 or some other kind of input
        void SetMotor(units::turns_per_second_t input);

    private:

        hardware::motor::TalonFX m_spinUpMotor;

};