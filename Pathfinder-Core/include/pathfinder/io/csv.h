#pragma once

#include "pathfinder/segments.h"
#include <cstdio>

namespace Pathfinder {
    namespace IO {
        void csv_profile_write(FILE *fp, Pathfinder::Segment *profile, unsigned int segment_count);
        unsigned int csv_profile_read(FILE *fp, Pathfinder::Segment *profile);
    }
}