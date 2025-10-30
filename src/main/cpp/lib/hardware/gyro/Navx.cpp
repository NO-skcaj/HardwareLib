#include "lib/hardware/gyro/Navx.h"
#include <frc/smartdashboard/SmartDashboard.h>

using namespace hardware::gyro;


frc::Rotation3d Navx::GetRotation()
{
        return m_gyro.GetRotation3d() + m_offset;
}

frc::Rotation3d Navx::GetOffset()
{
    return m_offset;
}

void Navx::ResetYaw()
{
    m_gyro.Reset();
    m_simYaw = 0_rad;
}

void Navx::SetOffset(frc::Rotation3d offset)
{
    m_offset = offset;
}

void Navx::SimPeriodic(units::radians_per_second_t rate)
{
    m_simRate = rate;
    m_simYaw += units::radian_t{m_simRate.value()}; // assuming 20ms loop time
    SetOffset(frc::Rotation3d{m_simYaw, 0_rad, 0_rad});
}