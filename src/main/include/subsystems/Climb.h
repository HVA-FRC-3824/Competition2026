#pragma once

#pragma region Includes
#include <functional>

#include <frc2/command/SubsystemBase.h>

#include "Constants.h"
#pragma endregion

enum ClimbState
{
    Deployed,
    Retracted
};

class Climb : public frc2::SubsystemBase
{
    public:

        explicit Climb();

        void SetState(ClimbState state);

        ClimbState GetState() const { return m_climbState; }

    private:

        // TODO: add hardware components here (TalonFX controller?)

        ClimbState m_climbState;
};