#pragma once

#include <units/angle.h>

#include "lib/hardware/hardware.h"


namespace hardware
{

namespace encoder
{

    class Encoder : public Hardware
    {
        public:

            virtual units::turn_t GetTurns() { return 0_tr; };
    };

}

}