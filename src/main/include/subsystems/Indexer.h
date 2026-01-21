#pragma once

#pragma region Includes
#include <functional>

#include <frc2/command/SubsystemBase.h>

#include <ctre/phoenix6/TalonFX.hpp>

#include "Constants.h"
#include "ConstantsCanIds.h"
#pragma endregion

#pragma region IndexerConstants
namespace IndexerConstants
{
    constexpr size_t numMotors = 2U;

    constexpr std::array<CANid_t, numMotors> motorIDs = 
    {
        ConstantsCanIds::indexerMotor1ID,
        ConstantsCanIds::indexerMotor2ID
    };
}
#pragma endregion

class Indexer : public frc2::SubsystemBase
{
    public:
        
        explicit Indexer();

        // Sets the indexing motors
        void SetMotors(double input);

    private:

        void ConfigureMotor(ctre::phoenix6::hardware::TalonFX& motor);

        // array of motors for the indexers, hopefully they're all the same, 4U is how many motors there are, replace with constant
        // TODO: update this to the real design
        std::array<ctre::phoenix6::hardware::TalonFX, IndexerConstants::numMotors> m_motors{
            ctre::phoenix6::hardware::TalonFX{IndexerConstants::motorIDs[0]},
            ctre::phoenix6::hardware::TalonFX{IndexerConstants::motorIDs[1]}
        };
};
