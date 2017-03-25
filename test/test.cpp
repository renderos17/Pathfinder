#include <pathfinder.h>
#include <stdio.h>
#include <string.h>

void csv_profile_write_shift(FILE *fp, Pathfinder::Segment *profile, int segment_count, int *shiftlevels) {
    fputs("time,distance,velocity,acceleration,shift\n", fp);
    int i;
    char buf[1024];
    float time = 0;
    for (i = 0; i < segment_count; i++) {
        Pathfinder::Segment seg = profile[i];
        sprintf(buf, "%.3f,%.3f,%.3f,%.3f,%d\n", time, seg.distance, seg.velocity, seg.acceleration, *(shiftlevels+i));
        fputs(buf, fp);
        time += seg.timescale;
    }
}

Pathfinder::Segment segments_1d[8092], segments_cache[8092];
int shiftlevels[8092];

int main() {
    // Shift Level - Velocity Threshold - Max Velocity - Max Acceleration
    Pathfinder::ShiftLevel levels[3] = {
        { 0, 0,     3,  4 },
        { 1, 1.5,   6,  2 },
        { 2, 3,     7, 6 }
    };

    int segs = Pathfinder::Profile::trapezoidal(&segments_1d[0], &shiftlevels[0], &levels[0], 3, 0.01, 20);

    printf("Segments: %d\n", segs);

    FILE *fp = fopen("out/profile/trapezoidal.csv", "w");
    // Pathfinder::IO::csv_profile_write(fp, &segments_1d[0], segs);
    csv_profile_write_shift(fp, &segments_1d[0], segs, &shiftlevels[0]);
    fclose(fp);

    return 0;
}