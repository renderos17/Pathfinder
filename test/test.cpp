#include <pathfinder.h>

#define POINT_LENGTH 2

int main() {
    Waypoint points[POINT_LENGTH];

    Waypoint p1 = { 0, 0, d2r(0) };
    Waypoint p2 = { -192, 0, d2r(0) };
    // Waypoint p3 = {  0, 0, 0 };
    points[0] = p1;
    points[1] = p2;
    // points[2] = p3;
    
    TrajectoryCandidate candidate;
    
    pathfinder_prepare(points, POINT_LENGTH, FIT_HERMITE_CUBIC, PATHFINDER_SAMPLES_LOW, 0.05, 15.0, 10.0, 60.0, &candidate);

    int length = candidate.length;

    // Array of Segments (the trajectory points) to store the trajectory in
    Segment *trajectory = malloc(length * sizeof(Segment));

    // Generate the trajectory
    int result = pathfinder_generate(&candidate, trajectory);
    printf("%d\n", result);
    
    FILE *fp = fopen("out.csv", "w");
    pathfinder_serialize_csv(fp, trajectory, result);
    fclose(fp);

    return 0;
}