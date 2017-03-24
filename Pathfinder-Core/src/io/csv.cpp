#include "pathfinder/io/csv.h"

void Pathfinder::IO::csv_profile_write(FILE *fp, Pathfinder::Segment *profile, int segment_count) {
    fputs("time,distance,velocity,acceleration\n", fp);
    int i;
    char buf[1024];
    float time = 0;
    for (i = 0; i < segment_count; i++) {
        Pathfinder::Segment seg = profile[i];
        sprintf(buf, "%.3f,%.3f,%.3f,%.3f\n", time, seg.distance, seg.velocity, seg.acceleration);
        fputs(buf, fp);
        time += seg.timescale;
    }
}

int Pathfinder::IO::csv_profile_read(FILE *fp, Pathfinder::Segment *profile) {
    return 0;
}