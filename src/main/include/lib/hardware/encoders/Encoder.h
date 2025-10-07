#pragma once

#include <units/angle.h>


namespace hardware
{

namespace encoder
{

    class Encoder
    {
        public:

            virtual units::turn_t GetTurns() { return 0_tr; };
    };

}

}