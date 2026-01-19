#pragma once

#include <cstdint>

/// @brief Type alias for CAN device IDs
using CANid_t = int;

/// @brief Base class for hardware components
class Hardware
{
public:
    virtual ~Hardware() = default;
};
