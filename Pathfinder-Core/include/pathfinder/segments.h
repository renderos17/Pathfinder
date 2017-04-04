#pragma once

namespace Pathfinder {
    #pragma pack(push, 4)
    struct Segment {
        float time;
        float distance;
        float velocity;
        float acceleration;
    };
    #pragma pack(pop)

    #pragma pack(push, 4)
    struct Segment2D {
        float x;
        float y;
        float angle;
    };
    #pragma pack(pop)
}