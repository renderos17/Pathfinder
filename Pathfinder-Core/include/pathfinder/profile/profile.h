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

        struct ShiftLevel {
            float threshold_velocity;
            float max_velocity, acceleration;
        };

        struct Profile {
            // Setpoint Get/Set
            void setpoint(float newsetpoint) { _setpoint = newsetpoint; }
            float setpoint() { return _setpoint; }

            virtual uint8_t calculate(Pathfinder::Segment *segment_out, Pathfinder::Segment *last_segment, float time) = 0;

            float _setpoint;
        };

        struct Shiftable {
            virtual void configure_shift(ShiftLevel *levels, int level_count) = 0;
            // Beware, shift_level() is set in calculate(), therefore if you want to shift at <time> you must first call
            // calculate() for this to make any sense. 
            virtual int shift_level() = 0;
            virtual void set_shift(int level) = 0;
        };
    }
}