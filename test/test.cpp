#include <pathfinder.h>

#define BUF_SIZE 4096

Pathfinder::Segment segments_1d[BUF_SIZE];      // Holds Distance, Velocity, Acceleration
Pathfinder::Segment2D segments_2d[BUF_SIZE];    // Holds X, Y, Angle

Pathfinder::Spline::Hermite splines[32];    // Holds our 2D position splines
int samples = 10000;                        // High Sample Rate for a smooth curve
double timescale = 0.001;                    // Generate points for every 0.01 seconds

int main() {
    float setpoint = 10;
    float max_vel = 3;
    float accel = 1.5;
    float time = 0;

    FILE *fp = fopen("out/trap.csv", "w");
    fputs("time,distance,velocity,acceleration\n", fp);

    Pathfinder::Segment seg = { 0,0,0,0 };
    Pathfinder::Profile::Trapezoidal profile(setpoint, max_vel, accel);
    bool done = false;
    while (!done) {
        uint8_t result = profile.calculate(&seg, &seg, time);
        time += timescale;
        fprintf(fp, "%.3f,%.3f,%.3f,%.3f\n", seg.time, seg.distance, seg.velocity, seg.acceleration);
        printf("Time: %.3f Dist: %.3f Vel: %.3f Acc: %.3f\n", seg.time, seg.distance, seg.velocity, seg.acceleration);
        done = result == Pathfinder::Profile::STATUS_DONE;
        // if (time/timescale >= 50) break;
    }
    fclose(fp);
    return 0;
}