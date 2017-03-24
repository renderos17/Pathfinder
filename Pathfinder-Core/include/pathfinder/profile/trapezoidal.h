#pragma once

#include "pathfinder/segments.h"

namespace Pathfinder {
    namespace Profile {
        // Calculate a trapezoidal linear motion profile
        int trapezoidal(Pathfinder::Segment *out, float timescale, float distance, float max_velocity, float acceleration);
    }
}