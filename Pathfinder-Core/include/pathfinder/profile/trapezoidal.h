#pragma once

#include "pathfinder/profile/profile.h"

// Base on current { pos, vel, time } such that it can recalculate itself
// Allow precalc, with optional regeneration?

namespace Pathfinder {
    namespace Profile {
        struct Trapezoidal : Pathfinder::Profile::Profile {
            Trapezoidal() {};
            Trapezoidal(float max_velocity, float acceleration) {
                configure(max_velocity, acceleration);
            }
            void configure(float max_velocity, float acceleration);

            uint8_t calculate(Pathfinder::Segment *segment_out, Pathfinder::Segment *last_segment, float time);
        
            float _max_velocity, _acceleration;
        };
    }
}