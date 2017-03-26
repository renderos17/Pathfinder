#pragma once

#include "pathfinder/segments.h"
#include <cstdio>

namespace Pathfinder {
    namespace IO {
        void csv_profile_write(FILE *fp, Pathfinder::Segment *profile, unsigned int segment_count);
        unsigned int csv_profile_read(FILE *fp, Pathfinder::Segment *profile);

        void csv_trajectory_write(FILE *fp, Pathfinder::Segment *segments_1d, Pathfinder::Segment2D *segments_2d, unsigned int segment_count);
        unsigned int csv_trajectory_read(FILE *fp, Pathfinder::Segment *segments_1d, Pathfinder::Segment2D *segments_2d);
    }
}