#pragma once

#include "pathfinder/profile/profile.h"

namespace Pathfinder {
    namespace Profile {
        struct Trapezoidal : Pathfinder::Profile::Profile, Pathfinder::Profile::Shiftable {
            Trapezoidal() {};
            Trapezoidal(float max_velocity, float acceleration, float target_timescale=PF_DEFAULT_TIMESCALE, float tolerance=PF_DEFAULT_TOLERANCE) {
                configure(max_velocity, acceleration, target_timescale, tolerance);
            }
            Trapezoidal(ShiftLevel *levels, int level_count, float target_timescale=PF_DEFAULT_TIMESCALE, float tolerance=PF_DEFAULT_TOLERANCE) {
                configure(0, 0, target_timescale, tolerance);   // Configure timescale and tolerance
                configure_shift(levels, level_count);
            }
            void configure(float max_velocity, float acceleration, float target_timescale=PF_DEFAULT_TIMESCALE, float tolerance=PF_DEFAULT_TOLERANCE);

            uint8_t calculate_single(Pathfinder::Segment *segment_out, Pathfinder::Segment *last_segment, float time);

            void configure_shift(ShiftLevel *levels, int level_count);
            int shift_level();
            void set_shift(int level);

            float _max_velocity, _acceleration;
            float _distance_integral = 0;   // This is used internally for SCurve profile generation
            ShiftLevel *_slvls;
            int _slcount, _slcurrent;
            bool _slconfigured;
        };
    }
}