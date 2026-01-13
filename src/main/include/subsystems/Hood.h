#pragma once

#pragma region Includes
#include <functional>

#include <frc2/command/SubsystemBase.h>


#include <units/length.h>
#pragma endregion


class Hood : public frc2::SubsystemBase
{
    public:
        
        Hood();

        // Sets the actuator, I'm not sure how this is going to be reliably controlled
        // I think its just power for input to the actuator, in which case i would need
        // to find what the current actuator extension is.
        void SetActuator(units::inch_t angle);

    private:

        // Here you use actuator
        
};