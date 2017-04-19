#pragma once

#include "pathfinder/profile/profile.h"

// Base on current { pos, vel, time } such that it can recalculate itself
// Allow precalc, with optional regeneration?

namespace Pathfinder {
    namespace Profile {
        struct Trapezoidal : Pathfinder::Profile::Profile, Pathfinder::Profile::Shiftable {
            Trapezoidal() {};
            Trapezoidal(float max_velocity, float acceleration) {
                configure(max_velocity, acceleration);
            }
            Trapezoidal(ShiftLevel *levels, int level_count) {
                configure_shift(levels, level_count);
            }
            void configure(float max_velocity, float acceleration);

            uint8_t calculate(Pathfinder::Segment *segment_out, Pathfinder::Segment *last_segment, float time);

            void configure_shift(ShiftLevel *levels, int level_count);
            int shift_level();
            void set_shift(int level);

            float _max_velocity, _acceleration;
            ShiftLevel *_slvls;
            int _slcount, _slcurrent;
            bool _slconfigured;
        };
    }
}