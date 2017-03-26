#include <pathfinder.h>

Pathfinder::Segment segments_1d[1024];      // Holds Distance, Velocity, Acceleration
Pathfinder::Segment2D segments_2d[1024];    // Holds X, Y, Angle
int shiftlevels[1024];                      // Holds what shifting level our gearbox should be at

Pathfinder::Spline::Hermite splines[32];    // Holds our 2D position splines
int samples = 10000;                        // High Sample Rate for a smooth curve
double timescale = 0.01;                    // Generate points for every 0.01 seconds

int main() {
    Pathfinder::Spline::Hermite::Waypoint waypoints[3] = {
        { 0, 0, d2r(15) },      // <0,0>m  @ 15deg
        { 4, 6, d2r(90) },      // <4,6>m  @ 90deg (vertical)
        { 6, 10, d2r(0) }       // <6,10>m @ 0deg (horizontal)
    };

    // Generate the smooth curve between our waypoints (xy position)
    int spline_count = Pathfinder::Spline::hermite(Pathfinder::Spline::HermiteCubic, waypoints, 3, splines);
    // Find the total arc length of our spline (i.e. the distance if we were to flatten to spline to 1D instead of 2D)
    double distance = Pathfinder::Spline::distance(splines, spline_count, samples);

    // Configure our Robot Gearbox (can be ommitted if your gearbox doesn't shift)
    Pathfinder::ShiftLevel shifting[2] = {
        { 0, 0,   10, 5 },      // 10m/s @ 5m/s/s acceleration
        { 1, 1.5, 20, 3 }       // 20m/s @ 3m/s/s acceleration, engages at 1.5m/s
    };
    // Generate our Trapezoidal Velocity profile based on our flat spline (also includes acceleration)
    int segment_count = Pathfinder::Profile::trapezoidal(segments_1d, shiftlevels, shifting, 2, timescale, distance);
    // Pop the spline back out into 2D with velocities in-tact (also adds heading)
    Pathfinder::trajectory(splines, spline_count, samples, segments_1d, segment_count, segments_2d);

    // Write to file.
    FILE *fp = fopen("out/traj/trajectory.csv", "w");
    Pathfinder::IO::csv_trajectory_write(fp, segments_1d, segments_2d, segment_count);
    fclose(fp);
}