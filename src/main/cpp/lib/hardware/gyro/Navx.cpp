#include "lib/hardware/gyro/Navx.h"


using namespace hardware::gyro;

#pragma region Navx
/// @brief Constructor for the Navx gyro class.
Navx::Navx()
{
    
}
#pragma endregion

#pragma region GetRotation
/// @brief Method to retrieve the current rotation with offset applied.
/// @return The current rotation with offset applied.
frc::Rotation3d Navx::GetRotation()
{
    return m_gyro.GetRotation3d() + m_offset;
}
#pragma endregion

#pragma region GetOffset
/// @brief Method to retrieve the current offset rotation.
/// @return The current offset rotation.
frc::Rotation3d Navx::GetOffset()
{
    // Return the offset rotation
    return m_offset;
}
#pragma endregion

#pragma region ResetYaw
/// @brief Method to reset the yaw angle to zero.
void Navx::ResetYaw()
{
    // Reset the gyro yaw angle
    m_gyro.Reset();
}
#pragma endregion

#pragma region SetOffset
/// @brief Method to set the offset rotation.
/// @param offset The offset rotation to set.
void Navx::SetOffset(frc::Rotation3d offset)
{
    // Set the offset rotation
    m_offset = offset;
}
#pragma endregion
