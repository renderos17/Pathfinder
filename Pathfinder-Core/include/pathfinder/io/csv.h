#pragma once

#include "pathfinder/segments.h"
#include <cstdio>

namespace Pathfinder {
    namespace IO {
        void csv_profile_write(FILE *fp, Pathfinder::Segment *profile, int segment_count);
        int csv_profile_read(FILE *fp, Pathfinder::Segment *profile);
    }
}