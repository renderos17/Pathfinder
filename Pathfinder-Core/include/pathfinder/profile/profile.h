#pragma once

#include "pathfinder/segments.h"
#include <inttypes.h>

#define PF_DEFAULT_TOLERANCE 0.05

// 1/1000th time difference seems rediculously small,
// however it helps a lot with generating the speed
// up and speed down portions of the profile, as
// it helps reduce the effect of a speed up/down being
// required in between two generations.
#define PF_DEFAULT_TIMESCALE 0.001

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

            // Setpoint Tolerance Get/Set
            void tolerance(float newtolerance) { _tolerance = newtolerance; }
            float tolerance() { return _tolerance; }

            // Setpoint Timescale Get/Set
            void timescale(float newtimescale) { _timescale = newtimescale; }
            float timescale() { return _timescale; }

            virtual uint8_t calculate_single(Pathfinder::Segment *segment_out, Pathfinder::Segment *last_segment, float time) = 0;
            virtual uint8_t calculate(Pathfinder::Segment *segment_out, Pathfinder::Segment *last_segment, float time);

            float _setpoint = 0, _tolerance = PF_DEFAULT_TOLERANCE, _timescale = PF_DEFAULT_TIMESCALE;
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