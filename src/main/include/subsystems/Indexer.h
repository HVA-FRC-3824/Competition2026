#pragma once

#pragma region Includes
#include <functional>

#include <frc2/command/SubsystemBase.h>

#include "lib/hardware/motors/TalonFX.h"

#include "Constants.h"
#pragma endregion

class Indexer : public frc2::SubsystemBase
{
    public:
        
        explicit Indexer();

        // Sets the indexing motors
        void SetMotors(double input);

    private:

        // array of motors for the indexers, hopefully they're all the same, 4U is how many motors there are, replace with constant
        // TODO: update this to the real design
        std::array<hardware::motor::TalonFX, constants::indexer::numMotors> m_motors;
};