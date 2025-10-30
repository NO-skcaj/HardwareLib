#pragma once

#include "lib/hardware/gyro/Navx.h"


// Wrapper
class Gyro : public hardware::gyro::Navx, public frc2::SubsystemBase
{
    public:

        static Gyro* GetInstance()
        {
            static Gyro instance;
            static Gyro* instancePtr = &instance;
            return instancePtr;
        }

    private:

        Gyro()
            : Navx()
        {
        }
};