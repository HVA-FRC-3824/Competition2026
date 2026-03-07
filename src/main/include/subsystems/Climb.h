#pragma once

#pragma region Includes
#include <functional>

#include <frc2/command/SubsystemBase.h>
#include <ctre/phoenix6/TalonFX.hpp>

#include "lib/TalonFXConfiguration.h"
#include "lib/SparkMaxConfiguration.h"

#include "lib/Logging.h"

#include "Constants.h"
#include "ConstantsRoboRio.h"
#pragma endregion

enum ClimbState
{
    Deployed,
    Retracted
};

namespace ClimbConstants
{
    constexpr auto ClimbMotorMaxRotations = 67_tr;
}

class Climb : public frc2::SubsystemBase
{
    public:

        explicit   Climb();
        
        void       SetState(ClimbState state);

        ClimbState GetState() const { return m_climbState; }

    private:

        ctre::phoenix6::hardware::TalonFX m_climbMotor{ConstantsCanIds::ClimbMotorId};

        ClimbState                        m_climbState;        
};
