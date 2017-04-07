#include <pathfinder.h>

#define BUF_SIZE 4096

Pathfinder::Segment segments_1d[BUF_SIZE];      // Holds Distance, Velocity, Acceleration
Pathfinder::Segment2D segments_2d[BUF_SIZE];    // Holds X, Y, Angle
Pathfinder::Trajectory::CoupledSegment cs;

Pathfinder::Spline::Hermite splines[32];    // Holds our 2D position splines
int samples = 10000;                        // High Sample Rate for a smooth curve
double timescale = 0.01;                   // Generate points for every 0.01 seconds

int main() {
    float setpoint = 10;
    float max_vel = 3;
    float accel = 6;
    float time = 0;

    Pathfinder::Spline::Hermite::Waypoint wps[3] = {
        { 0, 0, 0 },
        { 4, 0 ,0 },
        { 9, 4, d2r(90) }
    };

    Pathfinder::Profile::Trapezoidal profile(max_vel, accel);
    int spline_count = Pathfinder::Spline::hermite(Pathfinder::Spline::HermiteCubic, wps, 3, splines);

    Pathfinder::Trajectory::Coupled trajectory(0.5);
    trajectory.configure_path(splines, spline_count, samples);
    trajectory.configure_profile(&profile);

    FILE *fp = fopen("out/traj2.csv", "w");
    fputs("time,x,yc,yl,yr,vc,vl,vr,angle,ac,al,ar\n", fp);
    bool done = false;
    while (!done) {
        done = trajectory.calculate(&cs, &cs, time);
        fprintf(fp, "%.3f,%.3f,%.3f,,,%.3f,,,%.3f,%.3f,,\n", time, cs.center_2.x, cs.center_2.y, cs.center.velocity, r2d(cs.center_2.angle), cs.center.acceleration);
        fprintf(fp, "%.3f,%.3f,,%.3f,,,%.3f,,%.3f,,%.3f,\n", time, cs.left_2.x, cs.left_2.y, cs.left.velocity, r2d(cs.left_2.angle), cs.left.acceleration);
        fprintf(fp, "%.3f,%.3f,,,%.3f,,,%.3f,%.3f,,,%.3f\n", time, cs.right_2.x, cs.right_2.y, cs.right.velocity, r2d(cs.right_2.angle), cs.right.acceleration);
        time += timescale;
    }
    fclose(fp);
    return 0;
}