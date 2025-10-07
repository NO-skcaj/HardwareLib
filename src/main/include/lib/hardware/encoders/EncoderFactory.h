#pragma once

#include "lib/hardware/encoders/Encoder.h"
#include "lib/hardware/encoders/CANCoder.h"


namespace hardware
{

namespace encoder
{

    Encoder Get(int CanId)
    {
        return CANCoder{CanId};
    }

}

}