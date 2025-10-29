#pragma once

#include "lib/hardware/gyro/Navx.h"


// Wrapper
class Gyro : public hardware::gyro::Navx, public frc2::SubsystemBase
{
    public:

        static hardware::gyro::Navx* GetInstance()
        {
            static Gyro instance;
            static Gyro* instancePtr;
            return instancePtr;
        }

    private:

        Gyro()
        {
        }
};