#pragma once

#include "pathfinder/segments.h"

namespace Pathfinder {
    struct ShiftLevel {
        int level;
        float velocity_threshold;
        float max_velocity;
        float acceleration;
    };

    namespace Profile {
        // Calculate a trapezoidal linear motion profile
        int trapezoidal(Pathfinder::Segment *out, float timescale, float distance, float max_velocity, float acceleration);
        int trapezoidal(Pathfinder::Segment *out, float timescale, float pos_0, float pos_1, float vel_off, float max_velocity, float acceleration);
        int trapezoidal(Pathfinder::Segment *out, int *shiftlevel_out, Pathfinder::ShiftLevel *shiftlevels, unsigned int shiftlevel_count, float timescale, float distance);
    }
}