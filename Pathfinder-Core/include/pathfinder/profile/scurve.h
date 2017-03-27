#pragma once

#include "pathfinder/segments.h"

namespace Pathfinder {
    namespace Profile {
        int scurve(Pathfinder::Segment *out, float timescale, float distance, float max_velocity, float max_acceleration, float jerk);
    }
}