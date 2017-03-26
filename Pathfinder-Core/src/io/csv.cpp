#include "pathfinder/io/csv.h"

void Pathfinder::IO::csv_profile_write(FILE *fp, Pathfinder::Segment *profile, unsigned int segment_count) {
    fputs("time,distance,velocity,acceleration\n", fp);
    int i;
    float time = 0;
    for (i = 0; i < segment_count; i++) {
        Pathfinder::Segment seg = profile[i];
        fprintf(fp, "%.3f,%.3f,%.3f,%.3f\n", time, seg.distance, seg.velocity, seg.acceleration);
        time += seg.timescale;
    }
}

unsigned int Pathfinder::IO::csv_profile_read(FILE *fp, Pathfinder::Segment *profile) {
    return 0;
}

void Pathfinder::IO::csv_trajectory_write(FILE *fp, Pathfinder::Segment *segments_1d, Pathfinder::Segment2D *segments_2d, unsigned int segment_count) {
    double time;
    fputs("time,x,y,angle,velocity,acceleration\n", fp);
    for (int i = 0; i < segment_count; i++) {
        Pathfinder::Segment *s = &segments_1d[i];
        Pathfinder::Segment2D *s2 = &segments_2d[i];
        fprintf(fp, "%f,%f,%f,%f,%f,%f\n", time, s2->x, s2->y, s2->angle, s->velocity, s->acceleration);
        time += s->timescale;
    }
}

unsigned int Pathfinder::IO::csv_trajectory_read(FILE *fp, Pathfinder::Segment *segments_1d, Pathfinder::Segment2D *segments_2d) {
    return 0;
}