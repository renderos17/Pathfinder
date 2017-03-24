#include <pathfinder.h>
#include <stdio.h>

#define POINT_LENGTH 2

Pathfinder::Segment segments_1d[1024];

int main() {
    int segs = Pathfinder::Profile::trapezoidal(&segments_1d[0], 0.01, 12, 6, 3);

    printf("Segment Count (Trapezoidal): %d\n", segs);

    FILE *fp = fopen("out/profile/trapezoidal.csv", "w");
    Pathfinder::IO::csv_profile_write(fp, &segments_1d[0], segs);
    fclose(fp);

    return 0;
}