#pragma once

#include "pathfinder/profile/profile.h"
#include "pathfinder/profile/trapezoidal.h"

namespace Pathfinder {
    namespace Profile {
        struct SCurve : Pathfinder::Profile::Profile {
            SCurve() {};
            SCurve(float max_velocity, float max_acceleration, float jerk, float timescale=PF_DEFAULT_TIMESCALE, float tolerance=PF_DEFAULT_TOLERANCE) {
                configure(max_velocity, max_acceleration, jerk, timescale, tolerance);
            }
            void configure(float max_velocity, float max_acceleration, float jerk, float timescale=PF_DEFAULT_TIMESCALE, float tolerance=PF_DEFAULT_TOLERANCE);

            uint8_t calculate_single(Pathfinder::Segment *segment_out, Pathfinder::Segment *last_segment, float time);

            float _max_velocity, _max_acceleration, _jerk, _jerk_out;
            Pathfinder::Profile::Trapezoidal _velocity_profile;
        };
    }
}