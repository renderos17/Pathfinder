#include "pathfinder/io/csv.h"

void Pathfinder::IO::csv_profile_write(FILE *fp, Pathfinder::Segment *profile, unsigned int segment_count) {
    fputs("time,distance,velocity,acceleration\n", fp);
    int i;
    for (i = 0; i < segment_count; i++) {
        Pathfinder::Segment seg = profile[i];
        fprintf(fp, "%.3f,%.3f,%.3f,%.3f\n", seg.time, seg.distance, seg.velocity, seg.acceleration);
    }
}

unsigned int Pathfinder::IO::csv_profile_read(FILE *fp, Pathfinder::Segment *profile) {
    return 0;
}