#pragma once

#include "pathfinder/spline/spline.h"
#include "pathfinder/segments.h"

#include <cstddef>

namespace Pathfinder {
    void trajectory(Pathfinder::Spline::Spline *splines, unsigned int spline_count, size_t spline_size, unsigned int samples, Pathfinder::Segment *segments, unsigned int segment_count, Pathfinder::Segment2D *seg2d_ext);
    template<class T>
    void trajectory(T *splines, unsigned int spline_count, unsigned int samples, Pathfinder::Segment *segments, unsigned int segment_count, Pathfinder::Segment2D *seg2d_ext) {
        trajectory(splines, spline_count, sizeof(T), samples, segments, segment_count, seg2d_ext);
    }
}