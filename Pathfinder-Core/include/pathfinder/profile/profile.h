#pragma once

#include "pathfinder/segments.h"
#include <inttypes.h>

namespace Pathfinder {
    namespace Profile {
        enum Status {
            STATUS_ACCEL,
            STATUS_DECEL,
            STATUS_LEVEL,
            STATUS_DONE
        };

        struct Profile {
            virtual uint8_t calculate(Pathfinder::Segment *segment_out, Pathfinder::Segment *last_segment, float time) = 0;
        };
    }
}