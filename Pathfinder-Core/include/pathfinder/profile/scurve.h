#pragma once

#include "pathfinder/profile/profile.h"
#include "pathfinder/profile/trapezoidal.h"

namespace Pathfinder {
    namespace Profile {
        struct SCurve : Pathfinder::Profile::Profile {
            SCurve() {};
            SCurve(float max_velocity, float max_acceleration, float jerk, float tolerance=0.05) {
                configure(max_velocity, max_acceleration, jerk, tolerance);
            }
            void configure(float max_velocity, float max_acceleration, float jerk, float tolerance=0.05);

            uint8_t calculate(Pathfinder::Segment *segment_out, Pathfinder::Segment *last_segment, float time);

            float _max_velocity, _max_acceleration, _jerk, _jerk_out;
            Pathfinder::Profile::Trapezoidal _velocity_profile;
        };
    }
}