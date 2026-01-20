#pragma once

#pragma region Includes
#include <functional>

#include <frc2/command/SubsystemBase.h>

#include "Constants.h"
#pragma endregion

enum CurrentClimbState
{
    Ground,
    Level1,
    Level2,
    Level3
};

class Climb : public frc2::SubsystemBase
{
    public:

        explicit Climb();

    private:

        CurrentClimbState m_currentClimbState;
};