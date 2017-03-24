#include "pathfinder/profile/trapezoidal.h"
#include <cmath>

static void pf_trapezoid_generate_c(Pathfinder::Segment *out, float timescale, float acceleration, int segments, float initial_distance, float initial_velocity) {
    int i;
    for (i = 0; i <= segments; i++) {
        float time = timescale * i;
        // v = u + at
        float velocity = initial_velocity + acceleration * time;
        // s = s0 + ut + 0.5at^2
        float distance = initial_distance + initial_velocity * time + 0.5 * acceleration * time * time;

        out[i].timescale = timescale;
        out[i].distance = distance;
        out[i].velocity = velocity;
        out[i].acceleration = acceleration;
    }
}

int Pathfinder::Profile::trapezoidal(Pathfinder::Segment *out, float timescale, float distance, float max_velocity, float acceleration) {
    float accel_dist = 0.5 * max_velocity * max_velocity / acceleration;
    if (accel_dist * 2 > distance) {
        // Triange Profile - no velocity hold
        int accel_segments = (int)(sqrt(distance / acceleration) / timescale);

        // Generate the speed up
        pf_trapezoid_generate_c(out, timescale, acceleration, accel_segments, 0, 0);
        // Generate the speed down
        pf_trapezoid_generate_c(out + accel_segments, timescale, -acceleration, accel_segments, out[accel_segments].distance, out[accel_segments].velocity);

        return accel_segments * 2;
    } else {
        // Trapezoidal Profile - velocity hold
        float remaining_distance = distance - (accel_dist * 2);
        float hold_time = remaining_distance / max_velocity;
        
        int accel_segments = (int)(max_velocity / acceleration / timescale);
        int hold_segments = hold_time / timescale;
        
        // Generate the Speed Up
        pf_trapezoid_generate_c(out, timescale, acceleration, accel_segments, 0, 0);
        // Generate the Speed Hold
        pf_trapezoid_generate_c(out + accel_segments, timescale, 0, hold_segments, out[accel_segments].distance, out[accel_segments].velocity);
        // Generate the Speed Down
        pf_trapezoid_generate_c(out + accel_segments + hold_segments, timescale, -acceleration, accel_segments, out[accel_segments+hold_segments].distance, out[accel_segments+hold_segments].velocity);
        
        return accel_segments * 2 + hold_segments;
    }
}