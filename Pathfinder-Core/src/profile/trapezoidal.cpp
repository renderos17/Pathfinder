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

int Pathfinder::Profile::trapezoidal(Pathfinder::Segment *out, float timescale, float pos_0, float pos_1, float vel_off, float max_velocity, float acceleration) {
    float accel_dist = 0.5 * (max_velocity * max_velocity - vel_off * vel_off) / acceleration;
    if (accel_dist * 2 > (pos_1 - pos_0)) {
        // Triange Profile - no velocity hold
        float time = (sqrt(acceleration*(pos_1-pos_0)+vel_off*vel_off)-vel_off)/acceleration;
        int accel_segments = (int)(time / timescale);

        // Generate the speed up
        pf_trapezoid_generate_c(out, timescale, acceleration, accel_segments, pos_0, vel_off);
        // Generate the speed down
        pf_trapezoid_generate_c(out + accel_segments, timescale, -acceleration, accel_segments, out[accel_segments].distance, out[accel_segments].velocity);

        return accel_segments * 2;
    } else {
        // Trapezoidal Profile - velocity hold
        float remaining_distance = (pos_1 - pos_0) - (accel_dist * 2);
        float hold_time = remaining_distance / max_velocity;
        
        int accel_segments = (int)((max_velocity - vel_off) / acceleration / timescale);
        int hold_segments = hold_time / timescale;
        
        // Generate the Speed Up
        pf_trapezoid_generate_c(out, timescale, acceleration, accel_segments, pos_0, vel_off);
        // Generate the Speed Hold
        pf_trapezoid_generate_c(out + accel_segments, timescale, 0, hold_segments, out[accel_segments].distance, out[accel_segments].velocity);
        // Generate the Speed Down
        pf_trapezoid_generate_c(out + accel_segments + hold_segments, timescale, -acceleration, accel_segments, out[accel_segments+hold_segments].distance, out[accel_segments+hold_segments].velocity);
        
        return accel_segments * 2 + hold_segments;
    }
}

int Pathfinder::Profile::trapezoidal(Pathfinder::Segment *out, float timescale, float distance, float max_velocity, float acceleration) {
    return Pathfinder::Profile::trapezoidal(out, timescale, 0, distance, 0, max_velocity, acceleration);
}

int Pathfinder::Profile::trapezoidal(Pathfinder::Segment *out, int *shiftlevel_out, Pathfinder::ShiftLevel *shiftlevels, int shiftlevel_count, float timescale, float distance) {
    if (shiftlevel_count == 0) return -1;
    if (shiftlevel_count == 1) {
        return Pathfinder::Profile::trapezoidal(out, timescale, distance, shiftlevels[0].max_velocity, shiftlevels[0].acceleration);
    } else {
        int i, j, segments = 0, segs_needed, max_shift;
        double velocity, position;
        float t, d;
        Pathfinder::ShiftLevel *s0, *s1;
        Pathfinder::ShiftLevel *sf = shiftlevels;

        float t0 = shiftlevels[1].velocity_threshold / sf->acceleration;
        float d0 = 0.5 * sf->acceleration * t0 * t0;
        int segs_needed_0 = (int)(t0/timescale);

        if (d0*2 > distance) {
            // We'll never have an oppotunity to shift up
            segments = Pathfinder::Profile::trapezoidal(out, timescale, distance, sf->max_velocity, sf->acceleration);
            for (j = 0; j < segments; j++) {
                *(shiftlevel_out + j) = sf->level;
            }
        } else {
            // We shift up at least once.
            pf_trapezoid_generate_c(out, timescale, sf->acceleration, segs_needed_0, 0, 0);
            for (j = 0; j < segs_needed_0; j++) {
                *(shiftlevel_out + j + segments) = sf->level;
            }

            segments += segs_needed_0;
            velocity += shiftlevels[1].velocity_threshold;
            position += d0;
            max_shift = 1;

            // Generate Shift Acceleration
            for (i = 1; i < shiftlevel_count - 1; i++) {
                s0 = shiftlevels + i;
                s1 = shiftlevels + i+1;

                t = (s1->velocity_threshold - s0->velocity_threshold) / s0->acceleration;
                d = 0.5 * s0->acceleration * t * t;
                segs_needed = (int)(t/timescale);

                if (d*2 + position*2 + s0->velocity_threshold*t*2 > distance) {
                    // This shift will be our peak, as we'll go over peak otherwise
                    max_shift = i;
                    break;
                } else {
                    // This is an intermediary shift.
                    pf_trapezoid_generate_c(out + segments, timescale, s0->acceleration, segs_needed, position, velocity);
                    for (j = 0; j < segs_needed; j++) {
                        *(shiftlevel_out + j + segments) = s0->level;
                    }
                    segments += segs_needed;
                    position = out[segments].distance;
                    velocity = out[segments].velocity;
                }
                max_shift++;
            }

            // Calculate Peak Shift Level
            Pathfinder::ShiftLevel *sl = shiftlevels + max_shift;
            segs_needed = Pathfinder::Profile::trapezoidal(out+segments, timescale, position, distance-position, velocity, sl->max_velocity, sl->acceleration);
            for (j = 0; j < segs_needed; j++) {
                *(shiftlevel_out + j + segments) = sl->level;
            }
            segments += segs_needed;
            position = out[segments].distance;
            // Velocity will be the same on both sides of the top trapezoid/triangle

            // Calculate other shift level decel
            for (i = max_shift; i > 1; i--) {
                s0 = shiftlevels + i-1;
                s1 = shiftlevels + i;

                t = (s1->velocity_threshold - s0->velocity_threshold) / s0->acceleration;
                d = 0.5 * s0->acceleration * t * t;
                segs_needed = (int)(t/timescale);

                pf_trapezoid_generate_c(out + segments, timescale, -s0->acceleration, segs_needed, position, velocity);
                for (j = 0; j < segs_needed; j++) {
                    *(shiftlevel_out + j + segments) = s0->level;
                }
                segments += segs_needed;
                position = out[segments].distance;
                velocity = out[segments].velocity;
            }

            // Calculate Shift Level 0
            pf_trapezoid_generate_c(out + segments, timescale, -sf->acceleration, segs_needed_0, position, velocity);
            for (j = 0; j < segs_needed_0; j++) {
                *(shiftlevel_out + j + segments) = sf->level;
            }
            segments += segs_needed_0;
        }
    }
}