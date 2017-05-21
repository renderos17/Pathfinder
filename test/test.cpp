#include <pathfinder.h>

#define BUF_SIZE 4096

Pathfinder::Segment s1;

int samples = 10000;                        // High Sample Rate for a smooth curve
double timescale = 0.01;                   // Generate points for every 0.01 seconds

int main() {
    float setpoint = 9;
    float max_vel = 3;
    float accel = 6;
    float time = 0;

    Pathfinder::Profile::SCurve profile(max_vel, accel, 10);
    profile.setpoint(setpoint);
    
    FILE *fp = fopen("out/path.csv", "w");
    fputs("time,distance,velocity,acceleration\n", fp);
    int done = 0;
    while (done != Pathfinder::Profile::STATUS_DONE && time < 7) {
        done = profile.calculate(&s1, &s1, time);
        fprintf(fp, "%.3f,%.3f,%.3f,%.3f\n", time, s1.distance, s1.velocity, s1.acceleration);
        time += timescale;
    }
    fclose(fp);
    return 0;
}