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
            // Setpoing Get/Set
            void setpoint(float newsetpoint) { _setpoint = newsetpoint; }
            float setpoint() { return _setpoint; }

            virtual uint8_t calculate(Pathfinder::Segment *segment_out, Pathfinder::Segment *last_segment, float time) = 0;

            float _setpoint;
        };
    }
}